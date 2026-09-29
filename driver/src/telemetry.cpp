// Telemetry: driver log, status.json snapshot, events.csv and coverage.csv under
// %LOCALAPPDATA%\SVREnhance. All file I/O happens on this module's own thread, never in the pose callback.
#include "common.h"

#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <string>
#include <thread>

#ifndef SVR_VERSION
#define SVR_VERSION "0.2.0"
#endif

namespace svr {

static std::wstring g_dir;
static std::mutex g_logM;
static FILE *g_log = nullptr;
static DWORD g_mainThread = 0;

static std::thread g_thread;
static std::atomic<bool> g_run{ false };
static double g_tStart = 0;

static const uint64_t kCsvCap = 20ull * 1024 * 1024;

struct EventRec
{
	SYSTEMTIME wall;
	Ev kind;
	uint32_t idx;
	V3 world;
	double mag;
	double dur;
};

static std::mutex g_evM;
static const uint32_t kEvCap = 2048;
static EventRec g_ev[kEvCap];
static uint32_t g_evHead = 0, g_evCount = 0;
static uint64_t g_evDropped = 0;

static const char *EvName(Ev k)
{
	switch (k)
	{
	case Ev::Glitch: return "glitch_rejected";
	case Ev::Relocation: return "relocation_eased";
	case Ev::BridgeStart: return "dropout_bridge_start";
	case Ev::BridgeEnd: return "dropout_bridge_end";
	case Ev::BridgeExpired: return "dropout_too_long";
	case Ev::Disconnect: return "disconnect";
	case Ev::Reconnect: return "reconnect";
	case Ev::Gap: return "update_gap";
	case Ev::TrackingLost: return "tracking_lost";
	case Ev::StationMoved: return "station_moved";
	case Ev::StationLost: return "station_lost";
	case Ev::StationBack: return "station_back";
	}
	return "?";
}

static const char *ClassName(int cls)
{
	switch (cls)
	{
	case vr::TrackedDeviceClass_HMD: return "hmd";
	case vr::TrackedDeviceClass_Controller: return "controller";
	case vr::TrackedDeviceClass_GenericTracker: return "tracker";
	case vr::TrackedDeviceClass_TrackingReference: return "base_station";
	case vr::TrackedDeviceClass_DisplayRedirect: return "display_redirect";
	default: return "unknown";
	}
}

static const char *ModeName(Mode m)
{
	switch (m)
	{
	case Mode::Idle: return "passthrough";
	case Mode::Track: return "tracking";
	case Mode::Suspect: return "holding_suspect_jump";
	case Mode::Bridge: return "bridging_dropout";
	case Mode::Blend: return "easing_correction";
	case Mode::Lost: return "lost";
	}
	return "?";
}

void PushEvent(Ev kind, uint32_t idx, V3 world, double magnitude, double durationS)
{
	EventRec r;
	GetLocalTime(&r.wall);
	r.kind = kind;
	r.idx = idx;
	r.world = world;
	r.mag = magnitude;
	r.dur = durationS;
	std::lock_guard<std::mutex> lk(g_evM);
	if (g_evCount == kEvCap)
	{
		g_evDropped++;
		return;
	}
	g_ev[(g_evHead + g_evCount) % kEvCap] = r;
	g_evCount++;
}

void Log(const char *fmt, ...)
{
	char buf[1024];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof buf, fmt, ap);
	va_end(ap);
	{
		std::lock_guard<std::mutex> lk(g_logM);
		if (g_log)
		{
			SYSTEMTIME t;
			GetLocalTime(&t);
			fprintf(g_log, "%04d-%02d-%02d %02d:%02d:%02d.%03d %s\n", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond,
				t.wMilliseconds, buf);
			fflush(g_log);
		}
	}
	// vrserver's own log only from the thread that called Init/RunFrame.
	if (GetCurrentThreadId() == g_mainThread && vr::VRDriverLog())
	{
		char line[1100];
		snprintf(line, sizeof line, "%s\n", buf);
		vr::VRDriverLog()->Log(line);
	}
}

// JSON-escapes and caps a string: Appendf's line buffer is 1 KB, so an absurdly long property value
// must not be able to cut a line (and the JSON) short.
static std::string Esc(const std::string &in)
{
	std::string s = in.size() > 120 ? in.substr(0, 117) + "..." : in;
	std::string o;
	o.reserve(s.size() + 2);
	for (unsigned char c : s)
	{
		if (c == '"' || c == '\\') { o += '\\'; o += char(c); }
		else if (c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", c); o += b; }
		else o += char(c);
	}
	return o;
}

static void Appendf(std::string &s, const char *fmt, ...)
{
	char buf[1024];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof buf, fmt, ap);
	va_end(ap);
	if (n > 0) s.append(buf, size_t(n) < sizeof buf ? size_t(n) : sizeof buf - 1);
}

static FILE *OpenCsv(const wchar_t *name, const char *header, uint64_t &size)
{
	std::wstring path = g_dir + L"\\" + name;
	FILE *f = _wfsopen(path.c_str(), L"ab", _SH_DENYWR);
	if (!f) return nullptr;
	_fseeki64(f, 0, SEEK_END);
	size = uint64_t(_ftelli64(f));
	if (size == 0)
	{
		fputs(header, f);
		size = strlen(header);
	}
	return f;
}

struct Snap
{
	int info;
	int cls;
	bool lighthouse;
	std::string serial, model, system, modeLabel;
	uint64_t firmware;
	Mode mode;
	bool haveRaw;
	vr::DriverPose_t raw;
	DeviceStats st;
	StationMon stn;
};

// Keeps the time spent holding d.m (which pose threads wait on) to a few plain copies: the identity
// strings are immutable once infoState is 2, so they are copied outside the lock.
static void TakeSnap(uint32_t i, Snap &s)
{
	Device &d = g_dev[i];
	s.info = d.infoState.load(std::memory_order_acquire);
	if (s.info != 2) return;
	s.cls = d.cls;
	s.lighthouse = d.lighthouse;
	s.serial = d.serial;
	s.model = d.model;
	s.system = d.system;
	s.modeLabel = d.modeLabel;
	s.firmware = d.firmware.load(std::memory_order_relaxed);
	std::lock_guard<std::mutex> lk(d.m);
	s.mode = d.mode;
	s.haveRaw = d.haveRaw;
	s.raw = d.lastRaw;
	s.st = d.st;
	s.stn = d.stn;
}

static Snap g_snaps[kMaxDevices];

static void SnapAll()
{
	for (uint32_t i = 0; i < kMaxDevices; i++) TakeSnap(i, g_snaps[i]);
}

// ---------------------------------------------------------------------------------------------
// Base stations. A bumped station changes its distance and relative orientation to every other
// station; a universe re-alignment (which moves every reported station pose at once, as happens while
// SteamVR starts up) changes none of them. So each station is compared pairwise with the others in
// driver space and flagged when a strict majority of its pairs changed: with four stations the bumped
// one is named and the other three stay clear; with only two, both are flagged.

struct StationTrack
{
	bool seen = false;
	double tFirstValid = 0;
	bool basePair[kMaxDevices] = {};
	double baseDist[kMaxDevices] = {};
	Q baseRel[kMaxDevices] = {};
	int basePairs = 0;
	int pairs = 0;
	bool moved = false;
	uint32_t moveEvents = 0;
	double driftM = 0, driftRad = 0, maxDriftM = 0;
};
static StationTrack g_stn[kMaxDevices];

static bool StationReady(const Snap &s, const StationTrack &t, double now, double baselineS)
{
	return s.info == 2 && s.cls == vr::TrackedDeviceClass_TrackingReference && s.stn.validNow && s.stn.samples > 0 && t.seen &&
		now - t.tFirstValid >= baselineS;
}

static void AnalyzeStationsFromSnaps(double now)
{
	const Settings &S = CurSettings();
	for (uint32_t i = 0; i < kMaxDevices; i++)
	{
		const Snap &a = g_snaps[i];
		StationTrack &t = g_stn[i];
		if (a.info == 2 && a.cls == vr::TrackedDeviceClass_TrackingReference && a.stn.validNow && a.stn.samples > 0 && !t.seen)
		{
			t.seen = true;
			t.tFirstValid = now;
		}
	}
	for (uint32_t i = 0; i < kMaxDevices; i++)
	{
		const Snap &a = g_snaps[i];
		StationTrack &t = g_stn[i];
		if (!StationReady(a, t, now, S.stationBaselineS)) continue;
		int changed = 0, pairs = 0;
		double driftM = 0, driftRad = 0;
		for (uint32_t k = 0; k < kMaxDevices; k++)
		{
			if (k == i || !StationReady(g_snaps[k], g_stn[k], now, S.stationBaselineS)) continue;
			const StationMon &b = g_snaps[k].stn;
			double dist = Len(a.stn.posD - b.posD);
			Q rel = QNorm(QMul(QConj(a.stn.rotD), b.rotD));
			if (!t.basePair[k])
			{
				t.basePair[k] = true;
				t.baseDist[k] = dist;
				t.baseRel[k] = rel;
				t.basePairs++;
			}
			double dd = std::fabs(dist - t.baseDist[k]);
			double dr = QAngle(rel, t.baseRel[k]);
			pairs++;
			if (dd > driftM) driftM = dd;
			if (dr > driftRad) driftRad = dr;
			if (dd > S.stationMoveM || dr > S.stationMoveRad) changed++;
		}
		t.pairs = pairs;
		t.driftM = driftM;
		t.driftRad = driftRad;
		if (driftM > t.maxDriftM) t.maxDriftM = driftM;
		bool moved = pairs > 0 && changed * 2 > pairs;
		if (moved && !t.moved)
		{
			t.moveEvents++;
			PushEvent(Ev::StationMoved, i, a.stn.posW, driftM, 0);
		}
		t.moved = moved;
	}
}

void AnalyzeStations(double now)
{
	SnapAll();
	AnalyzeStationsFromSnaps(now);
}

StationReport StationStatus(uint32_t idx)
{
	const StationTrack &t = g_stn[idx];
	StationReport r;
	r.baseline = t.basePairs > 0;
	r.moved = t.moved;
	r.moveEvents = t.moveEvents;
	r.driftM = t.driftM;
	r.driftRad = t.driftRad;
	r.maxDriftM = t.maxDriftM;
	r.pairs = t.pairs;
	return r;
}

// ---------------------------------------------------------------------------------------------

static uint64_t g_prevUpdates[kMaxDevices];
static double g_prevRateT = 0;

static void WriteStatus(Snap *snaps)
{
	const Settings &S = CurSettings();
	SYSTEMTIME t;
	GetLocalTime(&t);
	// Update rate as a plain count per elapsed second; a mean of 1/dt is dominated by burst updates.
	double nowR = Now();
	double elapsed = g_prevRateT > 0 ? nowR - g_prevRateT : 0;
	g_prevRateT = nowR;
	std::string j;
	j.reserve(16384);
	Appendf(j, "{\n\"version\": \"%s\",\n\"time\": \"%04d-%02d-%02dT%02d:%02d:%02d\",\n\"uptime_s\": %.1f,\n", SVR_VERSION, t.wYear,
		t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond, Now() - g_tStart);
	int eff = HookEffective();
	Appendf(j, "\"hook\": {\"installed\": %s, \"calls\": %llu, \"effective\": %s},\n", HookInstalled() ? "true" : "false",
		static_cast<unsigned long long>(HookCalls()), eff < 0 ? "null" : (eff ? "true" : "false"));
	Appendf(j, "\"settings\": {\"filter_enable\": %s, \"lighthouse_only\": %s},\n", S.filterEnable ? "true" : "false",
		S.lighthouseOnly ? "true" : "false");
	{
		// What the world correction is doing right now (the quick-fix tool reads this back).
		const WorldFix &wf = g_worldFix[g_worldFixIdx.load(std::memory_order_acquire)];
		Q r = QIdentity();
		V3 tr;
		if (wf.active) WorldFixAt(wf, Now(), r, tr);
		Appendf(j, "\"world_fix\": {\"active\": %s, \"tilt_deg\": %.3f, \"requested\": %s},\n", QAngle(r, QIdentity()) > 1e-9 ? "true" : "false",
			QAngle(r, QIdentity()) * 57.29577951308232, S.worldFixEnable ? "true" : "false");
	}

	std::string warnings;
	auto warn = [&](const std::string &w) {
		if (!warnings.empty()) warnings += ",\n  ";
		warnings += "\"" + Esc(w) + "\"";
	};
	if (eff == 0) warn("Pose hook is installed but no poses reach it: filtering is not active on this SteamVR version.");

	j += "\"devices\": [";
	bool first = true;
	for (uint32_t i = 0; i < kMaxDevices; i++)
	{
		Snap &s = snaps[i];
		if (s.info != 2 || s.cls == vr::TrackedDeviceClass_TrackingReference) continue;
		V3 w = WorldPos(s.raw);
		const DeviceStats &st = s.st;
		double rate = (elapsed > 0 && st.updates >= g_prevUpdates[i]) ? double(st.updates - g_prevUpdates[i]) / elapsed : 0.0;
		g_prevUpdates[i] = st.updates;
		Appendf(j, "%s\n  {\"index\": %u, \"serial\": \"%s\", \"class\": \"%s\", \"system\": \"%s\", \"model\": \"%s\", ", first ? "" : ",", i,
			Esc(s.serial).c_str(), ClassName(s.cls), Esc(s.system).c_str(), Esc(s.model).c_str());
		Appendf(j, "\"state\": \"%s\", \"connected\": %s, \"pose_valid\": %s, \"result\": %d, ", ModeName(s.mode),
			s.raw.deviceIsConnected ? "true" : "false", s.raw.poseIsValid ? "true" : "false", int(s.raw.result));
		Appendf(j, "\"rate_hz\": %.1f, \"updates\": %llu, \"gaps_50ms\": %u, \"max_gap_ms\": %.1f, ", rate,
			static_cast<unsigned long long>(st.updates), st.gaps50ms, st.maxGapS * 1000);
		Appendf(j, "\"glitches_rejected\": %u, \"relocations_eased\": %u, \"dropouts_bridged\": %u, \"dropouts_too_long\": %u, ", st.glitches,
			st.relocations, st.bridges, st.bridgesExpired);
		Appendf(j, "\"bridged_s\": %.2f, \"held_suspect_s\": %.2f, \"disconnects\": %u, \"tracking_lost\": %u, \"jitter_mm\": %.3f, ",
			st.bridgedS, st.suspectS, st.disconnects, st.trackingLost, st.jitterMm);
		Appendf(j, "\"pos\": [%.3f, %.3f, %.3f]}", w.x, w.y, w.z);
		first = false;
	}
	j += "\n],\n\"stations\": [";
	first = true;
	for (uint32_t i = 0; i < kMaxDevices; i++)
	{
		Snap &s = snaps[i];
		if (s.info != 2 || s.cls != vr::TrackedDeviceClass_TrackingReference) continue;
		const StationMon &m = s.stn;
		const StationTrack &tr = g_stn[i];
		Appendf(j, "%s\n  {\"index\": %u, \"serial\": \"%s\", \"model\": \"%s\", \"mode_label\": \"%s\", \"firmware\": %llu, ", first ? "" : ",",
			i, Esc(s.serial).c_str(), Esc(s.model).c_str(), Esc(s.modeLabel).c_str(), static_cast<unsigned long long>(s.firmware));
		Appendf(j, "\"valid\": %s, \"baseline\": %s, \"pairs\": %d, \"drift_mm\": %.2f, \"drift_deg\": %.3f, \"max_drift_mm\": %.2f, \"pose_changes\": %u, ",
			m.validNow ? "true" : "false", tr.basePairs > 0 ? "true" : "false", tr.pairs, tr.driftM * 1000, tr.driftRad * 57.29577951308232,
			tr.maxDriftM * 1000, m.poseChanges);
		Appendf(j, "\"move_events\": %u, \"lost_events\": %u, \"pos\": [%.3f, %.3f, %.3f]}", tr.moveEvents, m.invalidTransitions, m.posW.x,
			m.posW.y, m.posW.z);
		first = false;

		if (tr.moved)
			warn("Base station " + s.serial + " has moved relative to the other stations since its baseline (a bump, vibration or a loose mount).");
		if (m.invalidTransitions > 3) warn("Base station " + s.serial + " dropped out " + std::to_string(m.invalidTransitions) + " times.");
		for (uint32_t k = i + 1; k < kMaxDevices; k++)
		{
			const Snap &o = snaps[k];
			if (o.info == 2 && o.cls == vr::TrackedDeviceClass_TrackingReference && !s.modeLabel.empty() && o.modeLabel == s.modeLabel)
				warn("Base stations " + s.serial + " and " + o.serial + " share channel/mode '" + s.modeLabel + "': they will interfere.");
		}
	}
	j += "\n],\n\"warnings\": [\n  " + warnings + "\n],\n";
	uint64_t dropped;
	{
		std::lock_guard<std::mutex> lk(g_evM);
		dropped = g_evDropped;
	}
	Appendf(j, "\"events_dropped\": %llu\n}\n", static_cast<unsigned long long>(dropped));

	std::wstring tmp = g_dir + L"\\status.json.tmp", fin = g_dir + L"\\status.json";
	FILE *f = _wfopen(tmp.c_str(), L"wb");
	if (!f) return;
	fwrite(j.data(), 1, j.size(), f);
	fclose(f);
	MoveFileExW(tmp.c_str(), fin.c_str(), MOVEFILE_REPLACE_EXISTING);
}

static void ThreadMain()
{
	// This thread briefly holds locks that vrserver's high-priority pose threads wait on. It sleeps
	// almost all the time, so running it high keeps those waits short instead of scheduler-length.
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
	uint64_t evSize = 0, covSize = 0;
	FILE *ev = OpenCsv(L"events.csv", "time,serial,class,event,x,y,z,magnitude_m,duration_ms\n", evSize);
	FILE *cov = OpenCsv(L"coverage.csv", "time,serial,class,x,y,z\n", covSize);
	V3 lastCov[kMaxDevices];
	double lastCovT[kMaxDevices] = {};
	bool haveCov[kMaxDevices] = {};
	double tStatus = 0, tCov = 0;

	while (g_run.load())
	{
		Sleep(250);
		double now = Now();
		if (now - g_tStart > 5.0) EnsurePoseHook(); // in case RunFrame never ran

		// Drain events.
		EventRec batch[256];
		uint32_t n = 0;
		{
			std::lock_guard<std::mutex> lk(g_evM);
			while (g_evCount && n < 256)
			{
				batch[n++] = g_ev[g_evHead];
				g_evHead = (g_evHead + 1) % kEvCap;
				g_evCount--;
			}
		}
		for (uint32_t k = 0; k < n && ev && evSize < kCsvCap; k++)
		{
			const EventRec &r = batch[k];
			if (g_dev[r.idx].infoState.load(std::memory_order_acquire) != 2) continue;
			const std::string &serial = g_dev[r.idx].serial; // immutable once published
			int cls = g_dev[r.idx].cls;
			int w = fprintf(ev, "%04d-%02d-%02dT%02d:%02d:%02d.%03d,%s,%s,%s,%.3f,%.3f,%.3f,%.4f,%.1f\n", r.wall.wYear, r.wall.wMonth, r.wall.wDay,
				r.wall.wHour, r.wall.wMinute, r.wall.wSecond, r.wall.wMilliseconds, serial.c_str(), ClassName(cls), EvName(r.kind), r.world.x,
				r.world.y, r.world.z, r.mag, r.dur * 1000);
			if (w > 0) evSize += uint64_t(w);
		}
		if (ev && n) fflush(ev);

		bool doStatus = now - tStatus >= 1.0, doCov = now - tCov >= 0.5;
		if (!doStatus && !doCov) continue;
		SnapAll();

		if (doCov && cov && covSize < kCsvCap)
		{
			tCov = now;
			SYSTEMTIME t;
			GetLocalTime(&t);
			for (uint32_t i = 0; i < kMaxDevices; i++)
			{
				Snap &s = g_snaps[i];
				if (s.info != 2 || !s.haveRaw || !s.raw.poseIsValid || s.cls == vr::TrackedDeviceClass_TrackingReference) continue;
				// A row when the device moved 5 cm, and at least one every 5 s while it rests, so the
				// row count per location approximates time spent there.
				V3 w = WorldPos(s.raw);
				if (haveCov[i] && Len(w - lastCov[i]) < 0.05 && now - lastCovT[i] < 5.0) continue;
				lastCov[i] = w;
				lastCovT[i] = now;
				haveCov[i] = true;
				int c = fprintf(cov, "%04d-%02d-%02dT%02d:%02d:%02d,%s,%s,%.3f,%.3f,%.3f\n", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute,
					t.wSecond, s.serial.c_str(), ClassName(s.cls), w.x, w.y, w.z);
				if (c > 0) covSize += uint64_t(c);
			}
			fflush(cov);
		}
		if (doStatus)
		{
			tStatus = now;
			AnalyzeStationsFromSnaps(now);
			WriteStatus(g_snaps);
		}
	}
	if (ev) fclose(ev);
	if (cov) fclose(cov);
}

void StartTelemetry()
{
	g_mainThread = GetCurrentThreadId();
	g_tStart = Now();
	for (StationTrack &t : g_stn) t = StationTrack{};
	for (uint64_t &u : g_prevUpdates) u = 0;
	g_prevRateT = 0;
	wchar_t base[MAX_PATH] = {};
	DWORD len = GetEnvironmentVariableW(L"LOCALAPPDATA", base, MAX_PATH);
	g_dir = (len > 0 && len < MAX_PATH) ? std::wstring(base) + L"\\SVREnhance" : L".";
	CreateDirectoryW(g_dir.c_str(), nullptr);
	{
		std::lock_guard<std::mutex> lk(g_logM);
		std::wstring logPath = g_dir + L"\\driver.log";
		g_log = _wfsopen(logPath.c_str(), L"a", _SH_DENYWR);
	}
	g_run.store(true);
	g_thread = std::thread(ThreadMain);
}

void StopTelemetry()
{
	g_run.store(false);
	if (g_thread.joinable()) g_thread.join();
	std::lock_guard<std::mutex> lk(g_logM);
	if (g_log)
	{
		fclose(g_log);
		g_log = nullptr;
	}
}

} // namespace svr
