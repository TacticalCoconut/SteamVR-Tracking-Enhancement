// SteamVR Tracking Enhancement, Room Setup: an alternative to SteamVR's Room Setup built on the driver's filtered poses
// and telemetry. Floor from a multi-point plane fit (with a tilt readout), boundary from a
// trigger-held trace that is simplified and optionally snapped to right-angled walls, play area as
// the largest rectangle that fits, forward either as you faced or square to the walls. Everything is
// written through SteamVR's own IVRChaperoneSetup after a backup; no file is edited by hand.
#include "chaperone.h"
#include "geom.h"
#include "overlay.h"
#include "telemetry_read.h"
#include "vrclient.h"

#include <conio.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <string>
#include <vector>

using namespace rs;

namespace {

const double kPi = 3.14159265358979323846;
const uint32_t kMax = vr::k_unMaxTrackedDeviceCount;

// ---------------------------------------------------------------------------------------------
// UI, input, tracking.

struct Ui
{
	TextOverlay ov;
	bool overlay = false;
	std::string last;
	void Show(const std::string &text)
	{
		if (text == last) return;
		last = text;
		printf("\n------------------------------------------------------------\n%s\n", text.c_str());
		fflush(stdout);
		if (overlay) ov.SetText(std::wstring(text.begin(), text.end())); // ASCII only
	}
};

struct Input
{
	bool trig[kMax] = {}, grip[kMax] = {};
	bool trigHeld[kMax] = {};
	int trigDown = -1, gripDown = -1;
	int key = 0;

	void Poll(vr::IVRSystem *sys, const std::vector<uint32_t> &ctrls)
	{
		trigDown = gripDown = -1;
		key = 0;
		for (uint32_t i : ctrls)
		{
			vr::VRControllerState_t st = {};
			if (!sys->GetControllerState(i, &st, sizeof st)) continue;
			bool t = (st.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) != 0;
			bool g = (st.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Grip)) != 0;
			if (t && !trig[i]) trigDown = int(i);
			if (g && !grip[i]) gripDown = int(i);
			trig[i] = t;
			grip[i] = g;
			trigHeld[i] = t;
		}
		if (_kbhit())
		{
			int c = _getch();
			if (c == 0 || c == 224) _getch();
			else key = toupper(c);
		}
	}
	bool Confirm() const { return trigDown >= 0 || key == '\r'; }
	bool Next() const { return gripDown >= 0 || key == 'N'; }
	bool Abort() const { return key == 27; }
};

struct Tracking
{
	vr::TrackedDevicePose_t raw[kMax] = {};
	vr::TrackedDevicePose_t stnd[kMax] = {};
	std::vector<uint32_t> controllers, stations;
	std::deque<std::pair<double, V3>> hist[kMax];
	std::string serial[kMax];
	bool serialRead[kMax] = {};

	void Poll(vr::IVRSystem *sys, double t)
	{
		sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, raw, kMax);
		sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, stnd, kMax);
		controllers.clear();
		stations.clear();
		for (uint32_t i = 0; i < kMax; i++)
		{
			vr::ETrackedDeviceClass cls = sys->GetTrackedDeviceClass(i);
			if (cls == vr::TrackedDeviceClass_Controller && raw[i].bDeviceIsConnected) controllers.push_back(i);
			if (cls == vr::TrackedDeviceClass_TrackingReference && raw[i].bPoseIsValid) stations.push_back(i);
			if (!serialRead[i] && cls != vr::TrackedDeviceClass_Invalid)
			{
				char buf[128] = {};
				sys->GetStringTrackedDeviceProperty(i, vr::Prop_SerialNumber_String, buf, sizeof buf, nullptr);
				serial[i] = buf;
				serialRead[i] = true;
			}
			if (raw[i].bPoseIsValid)
			{
				hist[i].push_back({ t, Pos(raw[i].mDeviceToAbsoluteTracking) });
				while (!hist[i].empty() && t - hist[i].front().first > 0.6) hist[i].pop_front();
			}
			else
			{
				hist[i].clear();
			}
		}
	}
	bool Ok(uint32_t i) const { return raw[i].bPoseIsValid && raw[i].eTrackingResult == vr::TrackingResult_Running_OK; }
	V3 P(uint32_t i) const { return Pos(raw[i].mDeviceToAbsoluteTracking); }
	double Speed(uint32_t i) const
	{
		const float *v = raw[i].vVelocity.v;
		return std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]);
	}
	// Moved less than 3 mm over the last half second.
	bool Still(uint32_t i, double t) const
	{
		if (hist[i].size() < 10 || t - hist[i].front().first < 0.5) return false;
		V3 last = hist[i].back().second;
		for (const auto &h : hist[i])
			if (Len(Sub(h.second, last)) > 0.003) return false;
		return true;
	}
};

struct Ctx
{
	VrClient vr;
	Ui ui;
	Input in;
	Tracking tr;
	DriverStatus ds;
	double tStatus = 0;
	bool quit = false;

	bool matrixIsStdToRaw = true; // verified at start-up
	bool checkOnly = false;       // --check: verify frames, report, exit
	bool checkPassed = false;
	double floorOffset = NAN;     // controller resting height above the floor
	bool offsetFromArgs = false;
	double wallHeight = 2.43;

	std::vector<V3> floorPts;
	Plane plane;
	P2 c0;
	V3 fHmd{ 0, 0, -1 };
	std::vector<P2> trace;
	double traceYMin = 0, traceYMax = 0;
	bool setSeated = false;
	M34 seated;

	void Tick()
	{
		double t = Now();
		tr.Poll(vr.sys, t);
		in.Poll(vr.sys, tr.controllers);
		if (t - tStatus > 1.0)
		{
			ds = ReadDriverStatus();
			tStatus = t;
		}
		vr::VREvent_t ev;
		while (vr.sys->PollNextEvent(&ev, sizeof ev))
			if (ev.eventType == vr::VREvent_Quit) quit = true;
		if (in.Abort()) quit = true;
		Sleep(10);
	}

	// What the driver currently thinks of this device (1 s granularity).
	bool DriverOk(uint32_t i, bool strict) const
	{
		auto it = ds.dev.find(tr.serial[i]);
		if (it == ds.dev.end()) return true;
		const std::string &s = it->second.state;
		if (s == "tracking" || s == "passthrough") return true;
		return !strict && s == "easing_correction";
	}
	double DriverJitter(uint32_t i) const
	{
		auto it = ds.dev.find(tr.serial[i]);
		return it == ds.dev.end() ? 0 : it->second.jitterMm;
	}
	std::string Name(uint32_t i) const
	{
		vr::ETrackedControllerRole r = vr.sys->GetControllerRoleForTrackedDeviceIndex(i);
		return r == vr::TrackedControllerRole_LeftHand ? "left" : (r == vr::TrackedControllerRole_RightHand ? "right" : tr.serial[i]);
	}
};

// ---------------------------------------------------------------------------------------------
// Steps.

bool StepInit(Ctx &c)
{
	if (!c.checkOnly) c.vr.setup->RoomSetupStarting();
	c.vr.setup->RevertWorkingCopy();
	vr::ChaperoneCalibrationState cal = c.vr.chap->GetCalibrationState();

	// Which way does the standing-zero-pose matrix go? Test both against the HMD's own poses.
	vr::HmdMatrix34_t live;
	bool haveLive = c.vr.setup->GetWorkingStandingZeroPoseToRawTrackingPose(&live);
	std::string devs;
	double t0 = Now();
	bool decided = false;
	while (!c.quit && Now() - t0 < 20)
	{
		c.Tick();
		devs = F("HMD %s, %zu controller(s), %zu base station(s)", c.tr.raw[0].bPoseIsValid ? "tracking" : "not tracking", c.tr.controllers.size(),
			c.tr.stations.size());
		if (!haveLive || !c.tr.raw[0].bPoseIsValid || !c.tr.stnd[0].bPoseIsValid)
		{
			c.ui.Show("SteamVR Tracking Enhancement - Room Setup\n\nWaiting for the headset to track (put it where the base stations see it)...\n" + devs);
			continue;
		}
		double e1 = 0, e2 = 0;
		decided = DetectConvention(live, Pos(c.tr.raw[0].mDeviceToAbsoluteTracking), Pos(c.tr.stnd[0].mDeviceToAbsoluteTracking), c.matrixIsStdToRaw, e1, e2);
		if (decided)
		{
			printf("standing matrix convention: %s (residuals %.4f / %.4f m)\n", c.matrixIsStdToRaw ? "standing->raw" : "raw->standing", e1, e2);
			break;
		}
		c.ui.Show(F("SteamVR Tracking Enhancement - Room Setup\n\nChecking the tracking frames... (residuals %.3f / %.3f m)\n%s", e1, e2, devs.c_str()));
	}
	if (!decided)
	{
		c.ui.Show("Could not verify SteamVR's standing-frame convention against the headset pose. Not continuing.\n" + devs);
		return false;
	}
	// Self-check of the 2D helpers against the 3x4 math.
	for (int k = 0; k < 50; k++)
	{
		double yaw = k * 0.37, ox = std::sin(k * 1.3) * 3, oz = std::cos(k * 0.7) * 3;
		P2 p{ std::cos(k * 2.1) * 4, std::sin(k * 1.9) * 4 };
		V3 a = Apply(Inverse(YawTrans(yaw, { ox, 0, oz })), { p.x, 0, p.z });
		P2 b = RawToStd2(p, yaw, { ox, oz });
		P2 back = StdToRaw2(b, yaw, { ox, oz });
		if (std::fabs(a.x - b.x) > 1e-9 || std::fabs(a.z - b.z) > 1e-9 || Dist(back, p) > 1e-9)
		{
			c.ui.Show("Internal error: 2D/3D transform mismatch. Not continuing.");
			return false;
		}
	}

	std::string drv = !c.ds.present ? "Tracking Enhancement driver: not found (poses are unfiltered; sampling still works)"
		: c.ds.hookEffective == 1 ? "Tracking Enhancement driver: active" : "Tracking Enhancement driver: loaded but NOT filtering";
	std::string warn;
	for (const std::string &w : c.ds.warnings) warn += "\n! " + w;
	std::string calS = cal == vr::ChaperoneCalibrationState_OK ? "OK" : F("state %d", int(cal));
	if (c.checkOnly)
	{
		float sx = 0, sz = 0;
		bool haveSize = c.vr.chap->GetPlayAreaSize(&sx, &sz);
		M34 M = FromVr(live);
		V3 floorRaw = c.matrixIsStdToRaw ? V3{ M.m[0][3], M.m[1][3], M.m[2][3] } : Apply(Inverse(M), {});
		printf("%s\n%s\ncalibration: %s%s\ncurrent play area: %.2f x %.2f m%s\ncurrent standing origin in raw space: (%.3f, %.3f, %.3f)\n"
			   "floor offset for the resting controller would be measured against this setup.\ncheck passed\n",
			devs.c_str(), drv.c_str(), calS.c_str(), warn.c_str(), sx, sz, haveSize ? "" : " (not readable)", floorRaw.x, floorRaw.y, floorRaw.z);
		c.checkPassed = true;
		return false;
	}
	while (!c.quit)
	{
		c.Tick();
		c.ui.Show("SteamVR Tracking Enhancement - Room Setup\n\n" + devs + "\n" + drv + "\nCurrent chaperone calibration: " + calS + warn +
			"\n\nFour steps: floor, centre/forward, boundary, review. Nothing is saved until you confirm at the end."
			"\n\nTrigger or Enter = start.   Esc = quit.");
		if (c.in.Confirm()) return true;
	}
	return false;
}

bool StepFloor(Ctx &c)
{
	std::string note;
	while (!c.quit)
	{
		c.Tick();
		std::string offs = std::isnan(c.floorOffset) ? "" : F("\nController rests %.1f cm above the floor (%s).", c.floorOffset * 100,
			c.offsetFromArgs ? "from --floor-offset" : "measured against the current setup");
		c.ui.Show(F("STEP 1 of 4: FLOOR\n\nLay ONE controller flat on the floor. Pull the OTHER controller's trigger (or press Enter); the "
			"tool samples once the floor controller is still.\nDo this at 4 corners and the centre (at least 3 spots).\n\nSamples: %zu%s%s"
			"\n\nGrip or N = next step.   Esc = quit.", c.floorPts.size(), offs.c_str(), note.c_str()));
		if (c.in.Next())
		{
			if (c.floorPts.size() >= 3) return true;
			note = "\n! Need at least 3 samples.";
			continue;
		}
		if (!c.in.Confirm()) continue;

		// The floor controller: lowest one, never the one whose trigger was pulled (if there are two).
		int floorIdx = -1;
		for (uint32_t i : c.tr.controllers)
		{
			if (int(i) == c.in.trigDown && c.tr.controllers.size() > 1) continue;
			if (!c.tr.raw[i].bPoseIsValid) continue;
			if (floorIdx < 0 || c.tr.P(i).y < c.tr.P(uint32_t(floorIdx)).y) floorIdx = int(i);
		}
		if (floorIdx < 0) { note = "\n! No tracked controller to sample."; continue; }
		uint32_t fi = uint32_t(floorIdx);

		// Wait for it to be still, tracking, and quiet according to the driver; then average 50 frames.
		double t0 = Now();
		bool got = false;
		while (!c.quit && Now() - t0 < 15)
		{
			c.Tick();
			bool ok = c.tr.Ok(fi), still = c.tr.Still(fi, Now()), drv = c.DriverOk(fi, true), quiet = c.DriverJitter(fi) <= 1.0;
			c.ui.Show(F("STEP 1 of 4: FLOOR\n\nSampling the %s controller...\n  tracking %s   still %s   driver %s   jitter %.2f mm %s",
				c.Name(fi).c_str(), ok ? "yes" : "NO", still ? "yes" : "no", drv ? "ok" : "NOT OK", c.DriverJitter(fi), quiet ? "" : "(too high)"));
			if (!(ok && still && drv && quiet)) continue;
			V3 acc;
			V3 accStd;
			int n = 0;
			bool stdOk = true;
			for (int k = 0; k < 50 && !c.quit; k++)
			{
				c.Tick();
				if (!c.tr.Ok(fi)) break;
				V3 p = c.tr.P(fi);
				acc.x += p.x; acc.y += p.y; acc.z += p.z;
				if (c.tr.stnd[fi].bPoseIsValid) { V3 s = Pos(c.tr.stnd[fi].mDeviceToAbsoluteTracking); accStd.y += s.y; }
				else stdOk = false;
				n++;
			}
			if (n < 50) break;
			V3 p{ acc.x / n, acc.y / n, acc.z / n };
			c.floorPts.push_back(p);
			if (std::isnan(c.floorOffset))
			{
				if (stdOk) c.floorOffset = accStd.y / n;
				else { c.floorOffset = 0; note = "\n! Current setup has no valid floor; using 0 offset. Pass --floor-offset <m> if needed."; }
			}
			printf("floor sample %zu: raw (%.4f, %.4f, %.4f)\n", c.floorPts.size(), p.x, p.y, p.z);
			Beep(880, 120);
			got = true;
			break;
		}
		if (!got) { note = "\n! Timed out waiting for the controller to be still and tracking. Try again."; Beep(330, 300); }
		else note = "";
	}
	return false;
}

bool StepCentre(Ctx &c)
{
	while (!c.quit)
	{
		c.Tick();
		c.ui.Show("STEP 2 of 4: CENTRE AND FORWARD\n\nStand where the MIDDLE of your play space should be and look straight ahead in the "
			"direction you want as 'forward' (usually your monitor).\n\nTrigger or Enter = sample (hold still for 1 second).   Esc = quit.");
		if (!c.in.Confirm()) continue;
		V3 acc, facc;
		int n = 0;
		for (int k = 0; k < 100 && !c.quit; k++)
		{
			c.Tick();
			if (!c.tr.Ok(0)) continue;
			V3 p = Pos(c.tr.raw[0].mDeviceToAbsoluteTracking), f = Forward(c.tr.raw[0].mDeviceToAbsoluteTracking);
			acc.x += p.x; acc.z += p.z;
			facc.x += f.x; facc.z += f.z;
			n++;
		}
		if (n < 50) { Beep(330, 300); continue; }
		c.c0 = { acc.x / n, acc.z / n };
		double fl = std::hypot(facc.x, facc.z);
		if (fl < 1e-6) continue;
		c.fHmd = { facc.x / fl, 0, facc.z / fl };
		printf("centre raw (%.3f, %.3f), forward raw (%.3f, %.3f)\n", c.c0.x, c.c0.z, c.fHmd.x, c.fHmd.z);
		Beep(880, 120);
		return true;
	}
	return false;
}

bool StepTrace(Ctx &c)
{
	std::string note;
	for (;;)
	{
		c.trace.clear();
		int held = -1;
		double yMin = 1e9, yMax = -1e9, len = 0;
		P2 last;
		while (!c.quit)
		{
			c.Tick();
			if (held < 0)
			{
				c.ui.Show("STEP 3 of 4: BOUNDARY\n\nHOLD the trigger and walk the edge of your space with the controller against the walls "
					"(or at the limit you want). Keep it at about the same height. RELEASE when you are back where you started." + note +
					"\n\nEsc = quit.");
				if (c.in.trigDown >= 0) { held = c.in.trigDown; yMin = 1e9; yMax = -1e9; len = 0; }
				continue;
			}
			uint32_t h = uint32_t(held);
			if (!c.in.trigHeld[h]) break;
			if (c.tr.Ok(h) && c.tr.Speed(h) < 3.0 && c.DriverOk(h, false))
			{
				V3 p = c.tr.P(h);
				P2 q{ p.x, p.z };
				if (!c.trace.empty()) len += Dist(last, q);
				last = q;
				c.trace.push_back(q);
				yMin = std::fmin(yMin, p.y);
				yMax = std::fmax(yMax, p.y);
			}
			c.ui.Show(F("STEP 3 of 4: BOUNDARY\n\nTracing with the %s controller... release the trigger when you are back at the start."
				"\n\nPoints: %zu   path: %.1f m   height span: %.0f cm", c.Name(h).c_str(), c.trace.size(), len, (yMax - yMin) * 100));
		}
		if (c.quit) return false;
		if (c.trace.size() < 50 || len < 2.0) { note = F("\n! Too short (%zu points, %.1f m). Try again.", c.trace.size(), len); Beep(330, 300); continue; }
		if (Dist(c.trace.front(), c.trace.back()) > 0.6)
			note = F("\n(The trace ended %.0f cm from its start; it was closed with a straight line.)", Dist(c.trace.front(), c.trace.back()) * 100);
		c.traceYMin = yMin;
		c.traceYMax = yMax;
		Beep(880, 120);
		return true;
	}
}

struct Plan
{
	bool snapped = false;
	std::vector<P2> polyRaw, polyStd;
	double yaw = 0, yawDeltaDeg = 0;
	P2 origin;
	Rect rect; // in standing space, centred on the origin
	double area = 0;
};

double SignedArea(const std::vector<P2> &poly)
{
	double s = 0;
	size_t n = poly.size();
	for (size_t i = 0; i < n; i++) s += poly[i].x * poly[(i + 1) % n].z - poly[(i + 1) % n].x * poly[i].z;
	return 0.5 * s;
}

Plan MakePlan(const std::vector<P2> &simplified, bool snap, V3 fHmd, P2 c0)
{
	Plan p;
	p.snapped = snap;
	double theta = DominantAngle(simplified);
	p.polyRaw = snap ? Rectify(simplified, theta, 0.10) : simplified;
	if (snap && p.polyRaw.size() == simplified.size() && p.polyRaw.size() > 8) p.snapped = false; // Rectify gave up
	double yawHmd = YawFromForward(fHmd);
	if (p.snapped)
	{
		// Forward square to the walls: the wall-aligned direction closest to where the user faced.
		double bestDot = -2;
		V3 best = fHmd;
		for (int k = 0; k < 4; k++)
		{
			double a = theta + k * kPi / 2;
			V3 d{ std::cos(a), 0, std::sin(a) };
			double dot = d.x * fHmd.x + d.z * fHmd.z;
			if (dot > bestDot) { bestDot = dot; best = d; }
		}
		p.yaw = YawFromForward(best);
	}
	else
	{
		p.yaw = yawHmd;
	}
	double dd = (p.yaw - yawHmd) * 180 / kPi;
	while (dd > 180) dd -= 360;
	while (dd < -180) dd += 360;
	p.yawDeltaDeg = dd;

	std::vector<P2> tmp;
	for (const P2 &q : p.polyRaw) tmp.push_back(RawToStd2(q, p.yaw, c0));
	Rect r = LargestAxisRect(tmp, 0.025);
	P2 cStd = r.ok ? r.Center() : P2{};
	p.origin = StdToRaw2(cStd, p.yaw, c0);
	p.rect = r;
	p.rect.min = { r.min.x - cStd.x, r.min.z - cStd.z };
	p.rect.max = { r.max.x - cStd.x, r.max.z - cStd.z };
	for (const P2 &q : p.polyRaw) p.polyStd.push_back(RawToStd2(q, p.yaw, p.origin));
	// Same winding as SteamVR's own room setup writes (negative shoelace sign in x/z).
	if (SignedArea(p.polyStd) > 0) std::reverse(p.polyStd.begin(), p.polyStd.end());
	p.area = Area(p.polyRaw);
	return p;
}

std::string WritePreview(const Ctx &c, const Plan &p, double yFloorRaw, std::vector<std::string> &warnings)
{
	std::string since = IsoHoursAgo(12);
	auto toStd = [&](double x, double z) { return RawToStd2({ x, z }, p.yaw, p.origin); };
	std::vector<P2> hot, cov, stn;
	for (const auto &r : ReadCsv("events.csv", since))
		if (r.size() >= 7 && (r[3] == "glitch_rejected" || r[3] == "relocation_eased")) hot.push_back(toStd(atof(r[4].c_str()), atof(r[6].c_str())));
	for (const auto &r : ReadCsv("coverage.csv", since))
		if (r.size() >= 6) cov.push_back(toStd(atof(r[3].c_str()), atof(r[5].c_str())));
	for (uint32_t i : c.tr.stations) { V3 s = c.tr.P(i); stn.push_back(toStd(s.x, s.z)); }

	// Glitch clusters near the boundary.
	std::map<std::pair<int, int>, int> cells;
	for (const P2 &h : hot) cells[{ int(std::floor(h.x / 0.5)), int(std::floor(h.z / 0.5)) }]++;
	for (const auto &kv : cells)
	{
		if (kv.second < 3) continue;
		P2 centre{ (kv.first.first + 0.5) * 0.5, (kv.first.second + 0.5) * 0.5 };
		double d = DistToBoundary(p.polyStd, centre);
		if (d < 0.4)
			warnings.push_back(F("%d pose glitches were logged around (%.1f, %.1f) m, %.0f cm from the boundary: look for a reflective surface there.",
				kv.second, centre.x, centre.z, d * 100));
	}

	double x0 = 1e9, x1 = -1e9, z0 = 1e9, z1 = -1e9;
	auto grow = [&](P2 q) { x0 = std::fmin(x0, q.x); x1 = std::fmax(x1, q.x); z0 = std::fmin(z0, q.z); z1 = std::fmax(z1, q.z); };
	for (const P2 &q : p.polyStd) grow(q);
	for (const P2 &q : stn) grow(q);
	x0 -= 0.4; x1 += 0.4; z0 -= 0.4; z1 += 0.4;
	double sc = 640 / std::fmax(x1 - x0, z1 - z0);
	auto X = [&](double x) { return 20 + (x - x0) * sc; };
	auto Y = [&](double z) { return 20 + (z - z0) * sc; }; // forward (-z) is up on the page

	std::string svg = "<svg viewBox=\"0 0 680 680\" xmlns=\"http://www.w3.org/2000/svg\" role=\"img\" aria-label=\"Room setup preview\">";
	svg += "<rect width=\"680\" height=\"680\" fill=\"var(--panel)\"/>";
	for (const P2 &q : cov) svg += F("<circle cx=\"%.1f\" cy=\"%.1f\" r=\"1.5\" fill=\"var(--muted)\" opacity=\"0.35\"/>", X(q.x), Y(q.z));
	std::string pts;
	for (const P2 &q : p.polyStd) pts += F("%.1f,%.1f ", X(q.x), Y(q.z));
	svg += "<polygon points=\"" + pts + "\" fill=\"none\" stroke=\"#3b7dd8\" stroke-width=\"3\"/>";
	if (p.rect.ok)
		svg += F("<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"#2f9a58\" fill-opacity=\"0.15\" stroke=\"#2f9a58\" stroke-width=\"2\"/>",
			X(p.rect.min.x), Y(p.rect.min.z), p.rect.W() * sc, p.rect.H() * sc);
	for (const P2 &q : hot) svg += F("<circle cx=\"%.1f\" cy=\"%.1f\" r=\"4\" fill=\"#d6453d\" opacity=\"0.8\"/>", X(q.x), Y(q.z));
	for (const V3 &f : c.floorPts) { P2 q = toStd(f.x, f.z); svg += F("<rect x=\"%.1f\" y=\"%.1f\" width=\"8\" height=\"8\" fill=\"#e8912d\"/>", X(q.x) - 4, Y(q.z) - 4); }
	for (const P2 &q : stn) svg += F("<path d=\"M%.1f,%.1f l7,13 h-14z\" fill=\"var(--fg)\"/>", X(q.x), Y(q.z) - 8);
	svg += F("<circle cx=\"%.1f\" cy=\"%.1f\" r=\"5\" fill=\"var(--fg)\"/>", X(0), Y(0));
	svg += F("<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"var(--fg)\" stroke-width=\"2\"/>", X(0), Y(0), X(0), Y(-0.5));
	svg += "</svg>";

	std::string warnHtml;
	for (const std::string &w : warnings) warnHtml += "<li>" + w + "</li>";
	std::string html = "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
		"<title>Room Setup Preview</title><style>:root{--bg:#f7f7f5;--panel:#fff;--fg:#1d1d1b;--muted:#6b6b66}"
		"@media (prefers-color-scheme:dark){:root{--bg:#161615;--panel:#1f1f1d;--fg:#ececea;--muted:#9a9a94}}"
		"body{background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,sans-serif;margin:0;padding:24px 16px}main{max-width:760px;margin:0 auto}"
		"svg{width:100%;max-width:680px;border:1px solid var(--muted);border-radius:8px}li{margin:6px 0}</style></head><body><main>"
		"<h1>Room setup preview</h1><p>SteamVR Tracking Enhancement</p><p>Standing frame, forward is up. Blue: boundary (" + F("%zu walls, %.2f m2", p.polyStd.size(), p.area) +
		"). Green: play area " + F("%.2f x %.2f m", p.rect.W(), p.rect.H()) + ". Orange: floor samples. Red: pose glitches (last 12 h). "
		"Grey: where devices have tracked. Triangles: base stations.</p>" +
		F("<p>Floor: %zu samples, tilt %.2f deg, rms %.1f mm, controller offset %.1f cm. Floor height in raw space: %.3f m. Forward %s (%+.1f deg from where you faced).</p>",
			c.floorPts.size(), c.plane.TiltRad() * 180 / kPi, c.plane.rms * 1000, c.floorOffset * 100, yFloorRaw,
			p.snapped ? "square to the walls" : "as you faced", p.yawDeltaDeg) +
		(warnHtml.empty() ? "" : "<h2>Warnings</h2><ul>" + warnHtml + "</ul>") + svg + "</main></body></html>";
	std::string path = TelemetryDir() + "\\roomsetup_preview.html";
	std::ofstream f(path, std::ios::binary);
	f << html;
	return path;
}

void ApplyWorking(Ctx &c, const Plan &p, double yFloorRaw)
{
	c.vr.setup->RevertWorkingCopy();
	c.vr.setup->SetWorkingPlayAreaSize(float(p.rect.W()), float(p.rect.H()));
	std::vector<vr::HmdQuad_t> quads;
	size_t n = p.polyStd.size();
	for (size_t i = 0; i < n; i++)
	{
		const P2 &a = p.polyStd[i], &b = p.polyStd[(i + 1) % n];
		vr::HmdQuad_t q;
		q.vCorners[0] = { { float(a.x), 0.0f, float(a.z) } };
		q.vCorners[1] = { { float(a.x), float(c.wallHeight), float(a.z) } };
		q.vCorners[2] = { { float(b.x), float(c.wallHeight), float(b.z) } };
		q.vCorners[3] = { { float(b.x), 0.0f, float(b.z) } };
		quads.push_back(q);
	}
	c.vr.setup->SetWorkingCollisionBoundsInfo(quads.data(), uint32_t(quads.size()));
	M34 M = YawTrans(p.yaw, { p.origin.x, yFloorRaw, p.origin.z });
	vr::HmdMatrix34_t vm = ToVr(c.matrixIsStdToRaw ? M : Inverse(M));
	c.vr.setup->SetWorkingStandingZeroPoseToRawTrackingPose(&vm);
	if (c.setSeated)
	{
		vr::HmdMatrix34_t vs = ToVr(c.matrixIsStdToRaw ? c.seated : Inverse(c.seated));
		c.vr.setup->SetWorkingSeatedZeroPoseToRawTrackingPose(&vs);
	}
}

// Returns 1 = commit, 0 = redo the trace, -1 = quit.
int StepReview(Ctx &c, Plan &chosen, double &yFloorRaw)
{
	c.plane = FitPlane(c.floorPts);
	std::vector<P2> simplified = SimplifyLoop(Decimate(c.trace, 0.02), 0.03);
	Plan a = MakePlan(simplified, true, c.fHmd, c.c0), b = MakePlan(simplified, false, c.fHmd, c.c0);
	double tiltDeg = c.plane.TiltRad() * 180 / kPi;
	std::string floorS = F("Floor: %zu samples, rms %.1f mm, universe tilt %.2f deg%s", c.floorPts.size(), c.plane.rms * 1000, tiltDeg,
		tiltDeg > 0.3 ? " (! floor is not level in tracking space: a base station's tilt calibration is off)" : "");
	std::string optA = a.rect.ok ? F("Trigger = SQUARE TO THE WALLS: %zu walls, %.2f m2, play area %.2f x %.2f m, forward turned %+.1f deg from where you faced",
		a.polyStd.size(), a.area, a.rect.W(), a.rect.H(), a.yawDeltaDeg) : "Trigger = (no rectangle fits the snapped boundary)";
	std::string optB = b.rect.ok ? F("Grip = AS TRACED: %zu points, %.2f m2, play area %.2f x %.2f m, forward as you faced", b.polyStd.size(), b.area,
		b.rect.W(), b.rect.H()) : "Grip = (no rectangle fits the traced boundary)";
	if (!a.snapped) optA = "Trigger = (boundary could not be snapped to right angles)";
	while (!c.quit)
	{
		c.Tick();
		c.ui.Show("STEP 4 of 4: REVIEW\n\n" + floorS + "\n\n" + optA + "\n\n" + optB + "\n\nR = redo the boundary.   Esc = quit.");
		if (c.in.key == 'R') return 0;
		if (c.in.trigDown >= 0 || c.in.key == '\r') { if (a.snapped && a.rect.ok) { chosen = a; break; } }
		if (c.in.gripDown >= 0 || c.in.key == 'N') { if (b.rect.ok) { chosen = b; break; } }
	}
	if (c.quit) return -1;

	yFloorRaw = c.plane.YAt(chosen.origin.x, chosen.origin.z) - c.floorOffset;
	double halfDiag = 0.5 * std::hypot(chosen.rect.W(), chosen.rect.H());
	double cornerErr = std::tan(c.plane.TiltRad()) * halfDiag;
	std::vector<std::string> warnings;
	if (cornerErr > 0.01) warnings.push_back(F("Because of the %.2f deg tilt the floor height is off by up to %.1f cm at the play-area corners.", tiltDeg, cornerErr * 100));
	if (c.traceYMax - c.traceYMin > 0.5) warnings.push_back(F("The controller height varied %.0f cm during the trace.", (c.traceYMax - c.traceYMin) * 100));
	std::string path = WritePreview(c, chosen, yFloorRaw, warnings);
	printf("preview: %s\n", path.c_str());
	ApplyWorking(c, chosen, yFloorRaw);
	c.vr.setup->ShowWorkingSetPreview();
	std::string warnS;
	for (const std::string &w : warnings) warnS += "\n! " + w;
	while (!c.quit)
	{
		c.Tick();
		c.ui.Show(F("PREVIEW\n\nThe new bounds are shown in the headset. Map: %s\n\nPlay area %.2f x %.2f m, %zu walls, floor at raw y %.3f m.%s"
			"\n\nTrigger or Enter = SAVE this setup.   R = redo the boundary.   Esc = quit without saving.", path.c_str(), chosen.rect.W(), chosen.rect.H(),
			chosen.polyStd.size(), yFloorRaw, warnS.c_str()));
		if (c.in.key == 'R') { c.vr.setup->HideWorkingSetPreview(); return 0; }
		if (c.in.Confirm()) { c.vr.setup->HideWorkingSetPreview(); return 1; }
	}
	c.vr.setup->HideWorkingSetPreview();
	return -1;
}

bool StepSeated(Ctx &c)
{
	while (!c.quit)
	{
		c.Tick();
		c.ui.Show("OPTIONAL: SEATED POSITION\n\nSit where you play seated games, face forward, and pull the trigger (or Enter) to set the seated "
			"origin there.\n\nGrip or N = keep the current seated position.");
		if (c.in.Next()) return true;
		if (!c.in.Confirm()) continue;
		V3 acc, facc;
		int n = 0;
		for (int k = 0; k < 100 && !c.quit; k++)
		{
			c.Tick();
			if (!c.tr.Ok(0)) continue;
			V3 p = Pos(c.tr.raw[0].mDeviceToAbsoluteTracking), f = Forward(c.tr.raw[0].mDeviceToAbsoluteTracking);
			acc.x += p.x; acc.y += p.y; acc.z += p.z;
			facc.x += f.x; facc.z += f.z;
			n++;
		}
		if (n < 50) { Beep(330, 300); continue; }
		double fl = std::hypot(facc.x, facc.z);
		if (fl < 1e-6) continue;
		c.seated = YawTrans(YawFromForward({ facc.x / fl, 0, facc.z / fl }), { acc.x / n, acc.y / n, acc.z / n });
		c.setSeated = true;
		Beep(880, 120);
		return true;
	}
	return false;
}

bool Commit(Ctx &c, const Plan &p, double yFloorRaw)
{
	std::string where;
	bool backed = BackupChaperone(c.vr.setup, where);
	ApplyWorking(c, p, yFloorRaw);
	bool ok = c.vr.setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
	float sx = 0, sz = 0;
	bool haveSize = c.vr.chap->GetPlayAreaSize(&sx, &sz);
	uint32_t nq = 0;
	c.vr.setup->GetLiveCollisionBoundsInfo(nullptr, &nq);
	std::string msg = ok ? "SAVED.\n\n" : "!!! SteamVR refused to save the working copy. Nothing changed.\n\n";
	msg += F("Live play area now %.2f x %.2f m (%s), %u wall quads.\nBackup of the previous setup: %s%s\n\nTrigger, Enter or Esc = exit.",
		sx, sz, haveSize ? "read back" : "not readable", nq, backed ? where.c_str() : "FAILED", backed ? "" : " (no backup could be written)");
	printf("%s\n", msg.c_str());
	while (!c.quit)
	{
		c.Tick();
		c.ui.Show(msg);
		if (c.in.Confirm()) break;
	}
	return ok;
}

} // namespace

int main(int argc, char **argv)
{
	setvbuf(stdout, nullptr, _IONBF, 0); // progress must reach a redirected console immediately
	Ctx c;
	bool noOverlay = false;
	for (int i = 1; i < argc; i++)
	{
		std::string a = argv[i];
		if (a == "--no-overlay") noOverlay = true;
		else if (a == "--check") { c.checkOnly = true; noOverlay = true; }
		else if (a == "--floor-offset" && i + 1 < argc) { c.floorOffset = atof(argv[++i]); c.offsetFromArgs = true; }
		else if (a == "--wall-height" && i + 1 < argc) c.wallHeight = atof(argv[++i]);
		else
		{
			printf("usage: svrenhance_roomsetup [--check] [--no-overlay] [--floor-offset <m>] [--wall-height <m>]\n"
				   "  --check          verify the tracking frames and report devices/driver/current setup, change nothing, exit\n"
				   "  --no-overlay     instructions on this console only (no panel in the headset)\n"
				   "  --floor-offset   height of a controller's tracked origin above the floor when it lies flat, in metres\n"
				   "                   (default: measured against the current SteamVR floor at the first floor sample)\n"
				   "  --wall-height    height of the boundary walls written to the chaperone, in metres (default 2.43)\n");
			return 2;
		}
	}
	std::string err;
	if (!c.vr.Init(err))
	{
		printf("%s\n", err.c_str());
		return 1;
	}
	printf("OpenVR runtime: %s\n", c.vr.dllPath.c_str());
	if (!noOverlay)
	{
		if (c.ui.ov.Create(c.vr.overlay, "svrenhance.roomsetup.hud", "SteamVR Tracking Enhancement Room Setup", err)) c.ui.overlay = true;
		else printf("overlay unavailable (%s); instructions on this console only\n", err.c_str());
	}
	c.ds = ReadDriverStatus();

	int rc = 1;
	if (c.checkOnly)
	{
		StepInit(c);
		c.vr.Shutdown();
		return c.checkPassed ? 0 : 1;
	}
	if (StepInit(c) && StepFloor(c) && StepCentre(c))
	{
		for (;;)
		{
			if (!StepTrace(c)) break;
			Plan plan;
			double yFloor = 0;
			int r = StepReview(c, plan, yFloor);
			if (r < 0) break;
			if (r == 0) continue;
			if (!StepSeated(c)) break;
			rc = Commit(c, plan, yFloor) ? 0 : 1;
			break;
		}
	}
	if (rc != 0) printf("exiting without saving\n");
	c.ui.ov.Destroy();
	c.vr.Shutdown();
	return rc;
}
