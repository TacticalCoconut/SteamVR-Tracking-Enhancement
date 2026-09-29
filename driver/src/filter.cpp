// Per-device pose pipeline: reflection-glitch gate, dropout bridge, correction easing, jitter filter,
// and base-station monitoring. Runs inside vrserver on every TrackedDevicePoseUpdated call, so it
// must stay allocation-free and cheap.
#include "common.h"

#include <windows.h>

namespace svr {

Device g_dev[kMaxDevices];
Settings g_settings[2];
std::atomic<int> g_settingsIdx{ 0 };

#ifdef SVR_TEST_CLOCK
double g_testNow = 0; // tests/filter_test.cpp drives time explicitly
double Now() { return g_testNow; }
#else
double Now()
{
	static const double inv = [] { LARGE_INTEGER f; QueryPerformanceFrequency(&f); return 1.0 / double(f.QuadPart); }();
	LARGE_INTEGER c;
	QueryPerformanceCounter(&c);
	return double(c.QuadPart) * inv;
}
#endif

// ---------------------------------------------------------------------------------------------
// Settings

static const double kDeg = 3.14159265358979323846 / 180.0;

static Settings Defaults()
{
	Settings s;
	// HMD: gate only. No bridging (hiding real HMD tracking loss is unsafe) and no smoothing (latency).
	s.hmd.gate = true;
	s.hmd.jumpM = 0.04;
	s.hmd.rotJumpRad = 15 * kDeg;
	s.hmd.confirmS = 0.04;
	s.hmd.suspectMaxS = 0.12;
	s.hmd.blendS = 0.12;

	s.controller.gate = true;
	s.controller.bridge = true;
	s.controller.jumpM = 0.03;
	s.controller.rotJumpRad = 20 * kDeg;
	s.controller.confirmS = 0.05;
	s.controller.suspectMaxS = 0.2;
	s.controller.bridgeMaxS = 0.3;
	s.controller.blendS = 0.15;

	s.tracker.gate = true;
	s.tracker.bridge = true;
	s.tracker.smooth = true;
	s.tracker.jumpM = 0.03;
	s.tracker.rotJumpRad = 20 * kDeg;
	s.tracker.confirmS = 0.06;
	s.tracker.suspectMaxS = 0.25;
	s.tracker.bridgeMaxS = 1.0;
	s.tracker.blendS = 0.2;
	return s;
}

static bool g_worldFixLoaded = false; // reset with the devices at every Init
static bool g_wfWarned = false;
static Settings g_wfLast;

static int GetI(const char *key, int def)
{
	vr::EVRSettingsError e = vr::VRSettingsError_None;
	int32_t v = vr::VRSettings()->GetInt32(kSection, key, &e);
	return e == vr::VRSettingsError_None ? int(v) : def;
}

static bool GetB(const char *key, bool def)
{
	vr::EVRSettingsError e = vr::VRSettingsError_None;
	bool v = vr::VRSettings()->GetBool(kSection, key, &e);
	return e == vr::VRSettingsError_None ? v : def;
}

static double GetF(const char *key, double def)
{
	vr::EVRSettingsError e = vr::VRSettingsError_None;
	float v = vr::VRSettings()->GetFloat(kSection, key, &e);
	return (e == vr::VRSettingsError_None && std::isfinite(v)) ? double(v) : def;
}

static void LoadClass(const char *prefix, ClassCfg &c)
{
	char k[96];
	auto key = [&](const char *name) { snprintf(k, sizeof k, "%s_%s", prefix, name); return k; };
	c.gate = GetB(key("gate"), c.gate);
	c.bridge = GetB(key("bridge"), c.bridge);
	c.smooth = GetB(key("smooth"), c.smooth);
	c.jumpM = Clamp(GetF(key("jump_m"), c.jumpM), 0.005, 1.0);
	c.rotJumpRad = Clamp(GetF(key("rot_jump_deg"), c.rotJumpRad / kDeg), 1.0, 180.0) * kDeg;
	c.maxAccel = Clamp(GetF(key("max_accel"), c.maxAccel), 0.0, 2000.0);
	c.confirmS = Clamp(GetF(key("confirm_ms"), c.confirmS * 1000) / 1000, 0.0, 1.0);
	c.suspectMaxS = Clamp(GetF(key("suspect_max_ms"), c.suspectMaxS * 1000) / 1000, 0.0, 2.0);
	c.bridgeMaxS = Clamp(GetF(key("bridge_max_ms"), c.bridgeMaxS * 1000) / 1000, 0.0, 5.0);
	c.blendS = Clamp(GetF(key("blend_ms"), c.blendS * 1000) / 1000, 0.001, 2.0);
	c.blendMaxM = Clamp(GetF(key("blend_max_m"), c.blendMaxM), 0.05, 10.0);
	c.minCutoff = Clamp(GetF(key("smooth_min_cutoff"), c.minCutoff), 0.05, 100.0);
	c.beta = Clamp(GetF(key("smooth_beta"), c.beta), 0.0, 1000.0);
	c.rotBeta = Clamp(GetF(key("smooth_rot_beta"), c.rotBeta), 0.0, 1000.0);
}

void LoadSettings()
{
	Settings s = Defaults();
	int revBefore = 0;
	if (vr::VRSettings())
	{
		s.filterEnable = GetB("filter_enable", s.filterEnable);
		s.lighthouseOnly = GetB("lighthouse_only", s.lighthouseOnly);
		LoadClass("hmd", s.hmd);
		LoadClass("controller", s.controller);
		LoadClass("tracker", s.tracker);
		s.stationMoveM = Clamp(GetF("station_move_mm", s.stationMoveM * 1000) / 1000, 0.001, 1.0);
		s.stationMoveRad = Clamp(GetF("station_move_deg", s.stationMoveRad / kDeg), 0.05, 45.0) * kDeg;
		s.stationBaselineS = Clamp(GetF("station_baseline_s", s.stationBaselineS), 5.0, 600.0);
		revBefore = GetI("world_fix_rev", 0);
		s.worldFixEnable = GetB("world_fix_enable", s.worldFixEnable);
		s.worldFixNx = GetF("world_fix_nx", s.worldFixNx);
		s.worldFixNz = GetF("world_fix_nz", s.worldFixNz);
		s.worldFixPivot[0] = GetF("world_fix_pivot_x", s.worldFixPivot[0]);
		s.worldFixPivot[1] = GetF("world_fix_pivot_y", s.worldFixPivot[1]);
		s.worldFixPivot[2] = GetF("world_fix_pivot_z", s.worldFixPivot[2]);
		s.worldFixEaseS = Clamp(GetF("world_fix_ease_ms", s.worldFixEaseS * 1000) / 1000, 0.0, 5.0);
		s.worldFixAllSystems = GetB("world_fix_all_systems", s.worldFixAllSystems);
		s.worldFixRev = GetI("world_fix_rev", 0);
	}
	int next = 1 - g_settingsIdx.load(std::memory_order_acquire);
	g_settings[next] = s;
	g_settingsIdx.store(next, std::memory_order_release);

	// The world correction is applied only as a complete set (see WorldFixReady). What is stored at
	// start-up takes effect at once; a change made while running is eased in.
	if (revBefore != s.worldFixRev || (s.worldFixRev & 1))
	{
		if (!g_wfWarned) Log("world correction: settings are being written or were left incomplete (rev %d/%d); not applying them", revBefore, s.worldFixRev);
		g_wfWarned = true;
	}
	else
	{
		g_wfWarned = false;
	}
	if (WorldFixReady(g_worldFixLoaded, revBefore, s.worldFixRev, g_wfLast.worldFixRev, s.worldFixEnable, g_wfLast.worldFixEnable))
	{
		double h2 = s.worldFixNx * s.worldFixNx + s.worldFixNz * s.worldFixNz;
		V3 n{ s.worldFixNx, h2 < 1.0 ? std::sqrt(1.0 - h2) : 0.0, s.worldFixNz };
		V3 pivot{ s.worldFixPivot[0], s.worldFixPivot[1], s.worldFixPivot[2] };
		bool ok = SetWorldFix(s.worldFixEnable, n, pivot, g_worldFixLoaded ? s.worldFixEaseS : 0.0, Now());
		if (s.worldFixEnable)
			Log(ok ? "world correction: levelling a %.2f deg tilt about (%.2f, %.2f, %.2f)" : "world correction REFUSED: %.2f deg tilt about (%.2f, %.2f, %.2f) is out of range",
				std::asin(Clamp(std::sqrt(h2), 0.0, 1.0)) / kDeg, pivot.x, pivot.y, pivot.z);
		else if (g_worldFixLoaded)
			Log("world correction: off");
		g_wfLast = s;
		g_worldFixLoaded = true;
	}
}

// ---------------------------------------------------------------------------------------------
// World correction

WorldFix g_worldFix[2];
std::atomic<int> g_worldFixIdx{ 0 };

Q LevelingRotation(V3 n)
{
	// Axis n x Y = (-n.z, 0, n.x), angle = the angle between n and Y.
	double s = std::sqrt(n.x * n.x + n.z * n.z);
	if (s < 1e-12) return QIdentity();
	double a = std::atan2(s, n.y);
	double k = std::sin(a / 2) / s;
	return QNorm({ std::cos(a / 2), -n.z * k, 0.0, n.x * k });
}

void WorldFixAt(const WorldFix &w, double now, Q &r, V3 &t)
{
	double u = w.easeS > 1e-6 ? SmoothStep((now - w.tChange) / w.easeS) : 1.0;
	if (u >= 1.0)
	{
		r = w.toR;
		t = w.toT;
		return;
	}
	r = QNorm(QSlerp(w.fromR, w.toR, u));
	t = w.fromT + (w.toT - w.fromT) * u;
}

bool SetWorldFix(bool enable, V3 n, V3 pivot, double easeS, double now)
{
	int cur = g_worldFixIdx.load(std::memory_order_acquire);
	const WorldFix &old = g_worldFix[cur];
	WorldFix w;
	if (old.active) WorldFixAt(old, now, w.fromR, w.fromT);
	bool ok = true;
	if (enable)
	{
		double len = Len(n);
		bool finite = Finite(n.x) && Finite(n.y) && Finite(n.z) && Finite(pivot.x) && Finite(pivot.y) && Finite(pivot.z);
		// A pivot a hundred metres away is not a play space; with it a small rotation would throw
		// every device out of the room.
		if (!finite || len < 1e-9 || Len(pivot) > 100.0)
		{
			ok = false;
		}
		else
		{
			n = n * (1.0 / len);
			if (n.y <= 0 || std::acos(Clamp(n.y, -1.0, 1.0)) > kWorldFixMaxTiltRad)
			{
				ok = false;
			}
			else
			{
				w.toR = LevelingRotation(n);
				w.toT = pivot - QRotate(w.toR, pivot);
			}
		}
	}
	w.tChange = now;
	w.easeS = Finite(easeS) ? Clamp(easeS, 0.0, 5.0) : 0.8;
	if (w.easeS <= 1e-6)
	{
		// Applied at once: there is no "from" left to be active about.
		w.fromR = w.toR;
		w.fromT = w.toT;
	}
	const Q id = QIdentity();
	w.active = QAngle(w.fromR, id) > 1e-12 || QAngle(w.toR, id) > 1e-12 || Len(w.fromT) > 1e-12 || Len(w.toT) > 1e-12;
	int next = 1 - cur;
	g_worldFix[next] = w;
	g_worldFixIdx.store(next, std::memory_order_release);
	return ok;
}

// Rotates the device's world-from-driver transform; the pose in driver space, which the filter
// works on, is left alone.
static void ApplyWorldFix(const Device &d, vr::DriverPose_t &pose, double now)
{
	const WorldFix &w = g_worldFix[g_worldFixIdx.load(std::memory_order_acquire)];
	if (!w.active) return;
	if (!d.lighthouse && !CurSettings().worldFixAllSystems) return;
	Q r;
	V3 t;
	WorldFixAt(w, now, r, t);
	V3 wt = FromArr(pose.vecWorldFromDriverTranslation);
	const Q &wq = pose.qWorldFromDriverRotation;
	if (!Finite(wt.x) || !Finite(wt.y) || !Finite(wt.z) || !Finite(wq.w) || !Finite(wq.x) || !Finite(wq.y) || !Finite(wq.z)) return;
	ToArr(QRotate(r, wt) + t, pose.vecWorldFromDriverTranslation);
	pose.qWorldFromDriverRotation = QNorm(QMul(r, wq));
}

// A device the driver has not identified yet cannot be corrected, and showing it uncorrected for a
// few frames would make it jump once it is. While a correction is in force such a device is reported
// as still calibrating, for at most two seconds.
static void HoldUnidentified(const Device &d, vr::DriverPose_t &pose, double now)
{
	const WorldFix &w = g_worldFix[g_worldFixIdx.load(std::memory_order_acquire)];
	if (!w.active || now - d.tFirstSeen > 2.0) return;
	Q r;
	V3 t;
	WorldFixAt(w, now, r, t);
	if (QAngle(r, QIdentity()) < 1e-9 && Len(t) < 1e-9) return; // nothing is being corrected right now
	pose.poseIsValid = false;
	pose.result = vr::TrackingResult_Calibrating_InProgress;
}

// ---------------------------------------------------------------------------------------------
// Device identity (read on the RunFrame thread, never in the pose callback)

void ResolveDeviceInfo()
{
	vr::CVRPropertyHelpers *props = vr::VRProperties();
	if (!props) return;
	for (uint32_t i = 0; i < kMaxDevices; i++)
	{
		Device &d = g_dev[i];
		int state = d.infoState.load(std::memory_order_acquire);
		if (state == 2)
		{
			// A station's OOTX data (firmware version) is decoded after activation; poll for it.
			if (d.cls == vr::TrackedDeviceClass_TrackingReference && d.firmware.load(std::memory_order_relaxed) == 0 && ++d.fwTries % 450 == 0)
			{
				vr::PropertyContainerHandle_t h = props->TrackedDeviceToPropertyContainer(i);
				vr::ETrackedPropertyError e = vr::TrackedProp_Success;
				uint64_t fw = h == vr::k_ulInvalidPropertyContainer ? 0 : props->GetUint64Property(h, vr::Prop_FirmwareVersion_Uint64, &e);
				if (e == vr::TrackedProp_Success && fw) d.firmware.store(fw, std::memory_order_relaxed);
			}
			continue;
		}
		if (state != 1) continue;

		vr::PropertyContainerHandle_t h = props->TrackedDeviceToPropertyContainer(i);
		vr::ETrackedPropertyError e = vr::TrackedProp_Success;
		int cls = h == vr::k_ulInvalidPropertyContainer ? 0 : props->GetInt32Property(h, vr::Prop_DeviceClass_Int32, &e);
		if (h == vr::k_ulInvalidPropertyContainer || e != vr::TrackedProp_Success || cls == 0)
		{
			if (++d.infoTries > 2000)
			{
				d.infoState.store(3, std::memory_order_release);
				Log("device %u: could not read its properties, leaving it untouched", i);
			}
			continue;
		}
		std::string serial = props->GetStringProperty(h, vr::Prop_SerialNumber_String);
		std::string system = props->GetStringProperty(h, vr::Prop_TrackingSystemName_String);
		std::string model = props->GetStringProperty(h, vr::Prop_ModelNumber_String);
		std::string mode;
		uint64_t fw = 0;
		if (cls == vr::TrackedDeviceClass_TrackingReference)
		{
			mode = props->GetStringProperty(h, vr::Prop_ModeLabel_String);
			fw = props->GetUint64Property(h, vr::Prop_FirmwareVersion_Uint64);
		}
		// No lock: nothing reads these until infoState == 2 is published below.
		d.cls = cls;
		d.serial = serial;
		d.system = system;
		d.model = model;
		d.modeLabel = mode;
		d.firmware.store(fw, std::memory_order_relaxed);
		d.lighthouse = system == "lighthouse";
		d.infoState.store(2, std::memory_order_release);
		Log("device %u: class %d serial '%s' system '%s' model '%s'%s%s", i, cls, serial.c_str(), system.c_str(),
			model.c_str(), mode.empty() ? "" : " mode ", mode.c_str());
	}
}

void ResetDevices()
{
	g_worldFixLoaded = false;
	for (Device &d : g_dev)
	{
		std::lock_guard<std::mutex> lk(d.m);
		d.infoState.store(0, std::memory_order_release);
		d.infoTries = 0;
		d.cls = 0;
		d.lighthouse = false;
		d.serial.clear();
		d.model.clear();
		d.system.clear();
		d.modeLabel.clear();
		d.firmware.store(0, std::memory_order_relaxed);
		d.fwTries = 0;
		d.mode = Mode::Idle;
		d.tLast = d.tLastWall = d.tFirstSeen = 0;
		d.lastConnected = d.lastOk = d.haveGood = d.haveRaw = d.jitterInit = false;
		d.euro = OneEuro{};
		d.st = DeviceStats{};
		d.stn = StationMon{};
	}
}

// ---------------------------------------------------------------------------------------------
// Pose pipeline

static const ClassCfg *CfgFor(const Settings &s, int cls)
{
	switch (cls)
	{
	case vr::TrackedDeviceClass_HMD: return &s.hmd;
	case vr::TrackedDeviceClass_Controller: return &s.controller;
	case vr::TrackedDeviceClass_GenericTracker: return &s.tracker;
	default: return nullptr;
	}
}

static V3 ToWorld(const vr::DriverPose_t &p, V3 driverPos)
{
	return QRotate(p.qWorldFromDriverRotation, driverPos) + FromArr(p.vecWorldFromDriverTranslation);
}

static void SetPose(vr::DriverPose_t &p, V3 pos, Q rot)
{
	ToArr(pos, p.vecPosition);
	p.qRotation = rot;
}

// Held positions coast on the last velocity, which decays with this time constant, so a hold
// can never run away (max travel = speed * tau).
static const double kHoldTau = 0.04;

static void BeginHold(Device &d, double now)
{
	d.tHold = now;
	d.holdPos = d.outPos;
	d.holdVel = d.vel;
	d.holdRot = d.outRot;
}

// Writes the held pose. Orientation keeps following the device's own IMU-driven rotation when
// that is trustworthy; otherwise it is frozen as well.
static void ApplyHold(Device &d, vr::DriverPose_t &pose, double now, bool rotFromRaw)
{
	double t = std::fmax(0.0, now - d.tHold); // exp(-t/tau) must only ever decay
	V3 p = d.holdPos + d.holdVel * (kHoldTau * (1.0 - std::exp(-t / kHoldTau)));
	Q r = rotFromRaw ? pose.qRotation : d.holdRot;
	SetPose(pose, p, r);
	ToArr(d.holdVel * std::exp(-t / kHoldTau), pose.vecVelocity);
	ToArr({}, pose.vecAcceleration);
	if (!rotFromRaw)
	{
		ToArr({}, pose.vecAngularVelocity);
		ToArr({}, pose.vecAngularAcceleration);
	}
	d.outPos = p;
	d.outRot = r;
}

// Starts easing out the difference between what we last output and the new raw pose, so a
// correction never shows up as a snap. Raw motion keeps passing through during the blend.
static void BeginBlend(Device &d, V3 rawPos, Q rawRot, double now, double blendS)
{
	d.blendOffPos = d.outPos - rawPos;
	d.blendOffRot = QNorm(QMul(d.outRot, QConj(rawRot)));
	d.tBlend = now;
	d.blendS = blendS;
	d.mode = Mode::Blend;
	d.euro.Reset(rawPos, rawRot);
}

static const double kVelTau = 0.02; // velocity estimate smoothing

// Rejoins the raw pose after a hold. A small difference is eased in; a large one means the device was
// lost and re-acquired somewhere else, and sliding a metre in 150 ms reads as a whip while the user
// has already seen the device stop, so it snaps.
static void Rejoin(Device &d, V3 raw, Q rawRot, double now, const ClassCfg &c)
{
	if (Len(raw - d.outPos) > c.blendMaxM)
	{
		d.outPos = raw;
		d.outRot = rawRot;
		d.euro.Reset(raw, rawRot);
		d.mode = Mode::Track;
		return;
	}
	BeginBlend(d, raw, rawRot, now, c.blendS);
}

static void Accept(Device &d, V3 raw, Q rawRot, double now, bool resetVel)
{
	double dt = now - d.tGood;
	if (resetVel || dt >= 0.1)
	{
		d.vel = {};
	}
	else if (dt > 1e-4)
	{
		V3 inst = (raw - d.goodPos) * (1.0 / dt);
		d.vel = d.vel + (inst - d.vel) * (1.0 - std::exp(-dt / kVelTau));
		double sp = Len(d.vel);
		if (sp > 20.0) d.vel = d.vel * (20.0 / sp);
	}
	d.goodPos = raw;
	d.goodRot = rawRot;
	d.tGood = now;
}

// Jitter = mean distance of raw samples from a 50 ms low-pass, measured only while the device is
// still. Stillness is judged from two low-passes (50 ms vs 500 ms) agreeing within 5 mm, i.e. drift
// under ~1 cm/s; a finite-difference velocity is useless for this because the noise swamps it.
static void UpdateJitter(Device &d, V3 raw, double dt)
{
	if (!d.jitterInit || dt <= 0 || dt > 0.1)
	{
		d.jitterFast = d.jitterSlow = raw;
		d.jitterInit = true;
		return;
	}
	d.jitterFast = d.jitterFast + (raw - d.jitterFast) * (1.0 - std::exp(-dt / 0.05));
	d.jitterSlow = d.jitterSlow + (raw - d.jitterSlow) * (1.0 - std::exp(-dt / 0.5));
	if (Len(d.jitterFast - d.jitterSlow) < 0.005)
	{
		double devMm = Len(raw - d.jitterFast) * 1000.0;
		d.st.jitterMm += (devMm - d.st.jitterMm) * (1.0 - std::exp(-dt / 2.0));
	}
}

// Records the station's latest pose; the movement analysis runs on the telemetry thread
// (AnalyzeStations), where it can compare stations with each other without nested locks.
static void ProcessStation(uint32_t idx, Device &d, const vr::DriverPose_t &pose)
{
	StationMon &s = d.stn;
	bool valid = pose.poseIsValid && pose.deviceIsConnected && PoseFinite(pose);
	if (valid != s.validNow)
	{
		if (!valid)
		{
			s.invalidTransitions++;
			PushEvent(Ev::StationLost, idx, s.posW, 0, 0);
		}
		else if (s.samples > 0)
		{
			PushEvent(Ev::StationBack, idx, s.posW, 0, 0);
		}
		s.validNow = valid;
	}
	if (!valid) return;

	V3 p = FromArr(pose.vecPosition);
	if (s.samples > 0 && Len(p - s.posD) > 0.002) s.poseChanges++;
	s.posD = p;
	s.rotD = pose.qRotation;
	s.posW = WorldPos(pose);
	s.samples++;
}

void ProcessPose(uint32_t idx, vr::DriverPose_t &pose)
{
	Device &d = g_dev[idx];
	double wall = Now();
	// poseTimeOffset is the driver's own estimate and may be NaN or step backwards between samples;
	// the pipeline's time must only move forward or every exp(-t/tau) hold turns into growth.
	double off = std::isfinite(pose.poseTimeOffset) ? Clamp(pose.poseTimeOffset, -0.5, 0.5) : 0.0;

	std::lock_guard<std::mutex> lk(d.m);
	double now = std::fmax(wall + off, d.tLast);
	double dt = d.tLast > 0 ? now - d.tLast : 0;
	d.tLast = now;
	// Stream statistics use the plain clock so offset changes don't look like stalls.
	double dtWall = d.tLastWall > 0 ? wall - d.tLastWall : 0;
	d.tLastWall = wall;
	bool connected = pose.deviceIsConnected;

	int info = d.infoState.load(std::memory_order_acquire);
	if (info == 0)
	{
		d.infoState.store(1, std::memory_order_release);
		d.lastConnected = connected;
		d.tFirstSeen = wall;
		HoldUnidentified(d, pose, wall);
		return;
	}
	if (info == 1) HoldUnidentified(d, pose, wall);
	if (info != 2) return;

	// First, so that everything recorded below (events, coverage, station positions) is in the same
	// corrected space that applications see.
	ApplyWorldFix(d, pose, wall);

	// Stream statistics (everything except base stations, which update irregularly by design).
	d.st.updates++;
	d.lastRaw = pose;
	d.haveRaw = true;
	if (d.cls == vr::TrackedDeviceClass_TrackingReference)
	{
		ProcessStation(idx, d, pose);
		d.lastConnected = connected;
		return;
	}
	// A silent stretch while the device was tracking: radio or USB, not acquisition.
	if (dtWall >= 0.05 && connected && d.lastConnected && d.lastOk)
	{
		d.st.gaps50ms++;
		if (dtWall > d.st.maxGapS) d.st.maxGapS = dtWall;
		PushEvent(Ev::Gap, idx, WorldPos(pose), 0, dtWall);
	}
	if (d.lastConnected && !connected)
	{
		d.st.disconnects++;
		PushEvent(Ev::Disconnect, idx, WorldPos(pose), 0, 0);
	}
	else if (!d.lastConnected && connected && d.st.updates > 1)
	{
		PushEvent(Ev::Reconnect, idx, WorldPos(pose), 0, 0);
	}
	d.lastConnected = connected;

	const Settings &S = CurSettings();
	const ClassCfg *c = CfgFor(S, d.cls);
	if (!S.filterEnable || !c || (S.lighthouseOnly && !d.lighthouse) || !(c->gate || c->bridge || c->smooth) || !connected || !PoseFinite(pose))
	{
		d.mode = Mode::Idle;
		d.haveGood = false;
		d.lastOk = false;
		return; // pass through untouched
	}

	bool ok = pose.poseIsValid && pose.result == vr::TrackingResult_Running_OK;
	if (d.lastOk && !ok)
	{
		d.st.trackingLost++;
		PushEvent(Ev::TrackingLost, idx, ToWorld(pose, d.outPos), 0, 0);
	}
	d.lastOk = ok;

	V3 raw = FromArr(pose.vecPosition);
	Q rawRot = pose.qRotation;
	double angSpeed = Len(FromArr(pose.vecAngularVelocity)); // magnitude is frame-independent
	if (!std::isfinite(angSpeed)) angSpeed = 0;

	// ---- optical tracking lost: bridge briefly, then let SteamVR's own behaviour through ----
	if (!ok)
	{
		if (c->bridge && d.haveGood && (d.mode == Mode::Track || d.mode == Mode::Blend || d.mode == Mode::Suspect))
		{
			// A suspect hold is already coasting from the last good pose; restarting it from the
			// coasted position with the undecayed velocity would double the travel bound.
			if (d.mode != Mode::Suspect) BeginHold(d, now);
			d.mode = Mode::Bridge;
			d.st.bridges++;
			PushEvent(Ev::BridgeStart, idx, ToWorld(pose, d.holdPos), 0, 0);
		}
		if (d.mode == Mode::Bridge)
		{
			double t = now - d.tHold;
			if (t <= c->bridgeMaxS)
			{
				ApplyHold(d, pose, now, pose.poseIsValid);
				pose.poseIsValid = true;
				pose.result = vr::TrackingResult_Running_OK;
				if (dt > 0 && dt < 0.1) d.st.bridgedS += dt;
				return;
			}
			d.st.bridgesExpired++;
			PushEvent(Ev::BridgeExpired, idx, ToWorld(pose, d.holdPos), 0, t);
		}
		d.mode = Mode::Lost;
		d.haveGood = false;
		return;
	}

	// ---- tracking OK ----
	if (!d.haveGood || d.mode == Mode::Idle || d.mode == Mode::Lost)
	{
		d.haveGood = true;
		d.goodPos = raw;
		d.goodRot = rawRot;
		d.tGood = now;
		d.vel = {};
		d.outPos = raw;
		d.outRot = rawRot;
		d.euro.Reset(raw, rawRot);
		d.jitterInit = false;
		d.mode = Mode::Track;
		return;
	}

	if (d.mode == Mode::Bridge)
	{
		PushEvent(Ev::BridgeEnd, idx, ToWorld(pose, raw), Len(raw - d.outPos), now - d.tHold);
		Accept(d, raw, rawRot, now, true);
		Rejoin(d, raw, rawRot, now, *c);
	}
	else
	{
		// Prediction from the last accepted sample. Its error is bounded, for any acceleration up to
		// maxAccel, by the velocity estimate's lag (a * kVelTau * dt) plus the unmodelled a*dt^2/2.
		// At the normal ~1 kHz update rate that leaves just jumpM; after radio stalls it widens.
		double dtg = Clamp(now - d.tGood, 0.0, 0.1);
		V3 pred = d.goodPos + d.vel * dtg;
		double err = Len(raw - pred);
		double rotErr = QAngle(rawRot, d.goodRot);
		double posAllow = c->jumpM + c->maxAccel * (kVelTau * dtg + 0.5 * dtg * dtg);
		bool rotOk = rotErr <= c->rotJumpRad + angSpeed * dtg * 1.5;
		bool plausible = !c->gate || (err <= posAllow && rotOk);

		if (d.mode == Mode::Suspect)
		{
			double tS = now - d.tSuspect;
			if (dt > 0 && dt < 0.1) d.st.suspectS += dt;
			// Continuity is judged sample-to-sample against the candidate path, so the growing
			// prediction allowance can never turn a persistent offset into a "return".
			bool consistent = Len(raw - d.candPos) <= c->jumpM && QAngle(rawRot, d.candRot) <= c->rotJumpRad + angSpeed * dt * 1.5;
			bool backOnOld = (err <= c->jumpM && rotOk) || (!consistent && plausible);
			if (backOnOld)
			{
				// Back on the old trajectory: the jump was transient (typically a reflection). Discard it.
				d.st.glitches++;
				PushEvent(Ev::Glitch, idx, ToWorld(pose, d.goodPos), d.st.lastJumpM, tS);
				Accept(d, raw, rawRot, now, false);
				BeginBlend(d, raw, rawRot, now, c->blendS * 0.5);
			}
			else
			{
				if (!consistent) d.tCandStart = now;
				d.candPos = raw;
				d.candRot = rawRot;
				if ((consistent && now - d.tCandStart >= c->confirmS) || tS >= c->suspectMaxS)
				{
					// The new position persisted: it's real (re-acquisition, drift correction). Ease it in.
					d.st.relocations++;
					PushEvent(Ev::Relocation, idx, ToWorld(pose, raw), d.st.lastJumpM, tS);
					Accept(d, raw, rawRot, now, true);
					Rejoin(d, raw, rawRot, now, *c);
				}
				else
				{
					ApplyHold(d, pose, now, rotOk);
					return;
				}
			}
		}
		else if (!plausible)
		{
			d.mode = Mode::Suspect;
			d.tSuspect = now;
			d.candPos = raw;
			d.candRot = rawRot;
			d.tCandStart = now;
			d.st.lastJumpM = err;
			BeginHold(d, now);
			ApplyHold(d, pose, now, rotOk);
			return;
		}
		else
		{
			Accept(d, raw, rawRot, now, false);
		}
	}

	// ---- output ----
	UpdateJitter(d, raw, dt);
	V3 out = raw;
	Q outR = rawRot;
	if (c->smooth)
	{
		OneEuro &e = d.euro;
		if (!e.init) e.Reset(raw, rawRot);
		double dts = Clamp(dt, 1e-4, 0.1);
		V3 dv = (raw - e.x) * (1.0 / dts);
		e.dx = e.dx + (dv - e.dx) * Alpha(1.0, dts);
		e.x = e.x + (raw - e.x) * Alpha(c->minCutoff + c->beta * Len(e.dx), dts);
		e.q = QNorm(QSlerp(e.q, rawRot, Alpha(c->minCutoff + c->rotBeta * angSpeed, dts)));
		out = e.x;
		outR = e.q;
	}
	if (d.mode == Mode::Blend)
	{
		double s = (now - d.tBlend) / d.blendS;
		if (s >= 1.0)
		{
			d.mode = Mode::Track;
		}
		else
		{
			double w = 1.0 - SmoothStep(s);
			out = out + d.blendOffPos * w;
			outR = QNorm(QMul(QSlerp(QIdentity(), d.blendOffRot, w), outR));
		}
	}
	SetPose(pose, out, outR);
	d.outPos = out;
	d.outRot = outR;
}

} // namespace svr
