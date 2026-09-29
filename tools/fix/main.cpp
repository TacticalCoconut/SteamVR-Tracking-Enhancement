// SVREnhance Quick Fix: a panel in the SteamVR dashboard with which the user reports what is wrong
// with the world ("I'm floating", "I'm clipping into the floor", "the world is tilted") and gets
// the matching fix.
//
//   floating / clipping   the floor height is wrong. A controller lying on the real floor shows by
//                         how much; the standing origin is moved through IVRChaperoneSetup.
//   tilted                the tracking universe is not level. A controller laid on the floor at three
//                         or more spots gives the floor's plane; the SVREnhance driver rotates the
//                         world to level it (SteamVR's chaperone can only store a yaw).
//
// Every change can be undone, and the first change of a session backs up the chaperone.
#include "chaperone.h"
#include "fixmath.h"
#include "geom.h"
#include "overlay.h"
#include "panel.h"
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

const uint32_t kMax = vr::k_unMaxTrackedDeviceCount;
const char *const kSection = "driver_svrenhance";
const char *const kAppKey = "svrenhance.quickfix";
const double kNudgeM = 0.01;

enum Id { kFloating = 1, kClipping, kTilted, kUp, kDown, kUndo, kReset, kLearn, kCancel };

// ---------------------------------------------------------------------------------------------
// What the tool remembers between runs: key=value lines in %LOCALAPPDATA%\SVREnhance\quickfix.cfg.

struct Store
{
	std::map<std::string, double> v;
	std::string Path() const { return TelemetryDir() + "\\quickfix.cfg"; }
	void Load()
	{
		std::ifstream f(Path());
		std::string line;
		while (std::getline(f, line))
		{
			size_t eq = line.rfind('=');
			if (eq == std::string::npos || eq == 0) continue;
			v[line.substr(0, eq)] = atof(line.c_str() + eq + 1);
		}
	}
	void Save() const
	{
		CreateDirectoryA(TelemetryDir().c_str(), nullptr);
		std::ofstream f(Path(), std::ios::trunc);
		for (const auto &kv : v) f << kv.first << "=" << F("%.6f", kv.second) << "\n";
	}
	bool Has(const std::string &k) const { return v.count(k) != 0; }
	double Get(const std::string &k, double def) const
	{
		auto it = v.find(k);
		return it == v.end() ? def : it->second;
	}
};

// ---------------------------------------------------------------------------------------------

struct Tracking
{
	vr::TrackedDevicePose_t raw[kMax] = {};
	std::deque<std::pair<double, V3>> hist[kMax];
	std::string model[kMax], serial[kMax];
	int cls[kMax] = {};
	bool read[kMax] = {};

	void Poll(vr::IVRSystem *sys, double t)
	{
		sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, raw, kMax);
		for (uint32_t i = 0; i < kMax; i++)
		{
			cls[i] = int(sys->GetTrackedDeviceClass(i));
			if (cls[i] == vr::TrackedDeviceClass_Invalid) { read[i] = false; hist[i].clear(); continue; }
			if (!read[i])
			{
				char buf[128] = {};
				sys->GetStringTrackedDeviceProperty(i, vr::Prop_ModelNumber_String, buf, sizeof buf, nullptr);
				model[i] = buf;
				sys->GetStringTrackedDeviceProperty(i, vr::Prop_SerialNumber_String, buf, sizeof buf, nullptr);
				serial[i] = buf;
				read[i] = true;
			}
			if (Ok(i))
			{
				hist[i].push_back({ t, Pos(raw[i].mDeviceToAbsoluteTracking) });
				while (!hist[i].empty() && t - hist[i].front().first > 1.2) hist[i].pop_front();
			}
			else
			{
				hist[i].clear();
			}
		}
	}
	bool Ok(uint32_t i) const { return raw[i].bPoseIsValid && raw[i].bDeviceIsConnected && raw[i].eTrackingResult == vr::TrackingResult_Running_OK; }
	bool Hand(uint32_t i) const { return cls[i] == vr::TrackedDeviceClass_Controller || cls[i] == vr::TrackedDeviceClass_GenericTracker; }

	// Has not moved more than `tol` for a full second; `mean` is its average position over that second.
	bool Resting(uint32_t i, double t, double tol, V3 &mean) const
	{
		if (hist[i].size() < 20 || t - hist[i].front().first < 1.0) return false;
		V3 last = hist[i].back().second, acc;
		int n = 0;
		for (const auto &h : hist[i])
		{
			if (t - h.first > 1.0) continue;
			if (Len(Sub(h.second, last)) > tol) return false;
			acc.x += h.second.x; acc.y += h.second.y; acc.z += h.second.z;
			n++;
		}
		if (n < 15) return false;
		mean = { acc.x / n, acc.y / n, acc.z / n };
		return true;
	}
};

struct Change
{
	bool tilt = false;
	double floorDy = 0;
	bool prevEnable = false;
	float prevNx = 0, prevNz = 0, prevPivot[3] = { 0, 0, 0 };
};

struct App
{
	VrClient vr;
	Panel panel;
	TextOverlay hud;
	bool haveHud = false;
	vr::VROverlayHandle_t main = vr::k_ulOverlayHandleInvalid, thumb = vr::k_ulOverlayHandleInvalid;
	Tracking tr;
	DriverStatus ds;
	Store store;
	std::vector<Change> undo;
	std::vector<Button> buttons;

	bool stdToRaw = true;
	M34 standingToRaw;
	bool backedUp = false;
	bool quit = false;

	enum State { Idle, Floor, Confirm, Tilt, Learn } state = Idle;
	fix::Report report = fix::Report::Floating;
	fix::FloorDecision pending;
	std::vector<V3> tiltPts;
	int tiltDev = -1;
	double tState = 0, tResetArmed = 0, tVerifyUntil = 0, verifyDeg = 0, tOwnCommit = 0;

	// Only a device that was moved after the button was pressed, and then put down, is measured:
	// a controller that has been lying on the sofa all along is not on the floor.
	V3 p0[kMax];
	bool haveP0[kMax] = {}, moved[kMax] = {};
	std::string message = "Pick what is wrong. Nothing changes until a measurement is taken or a floor button is pressed.";
	std::string shown;
};

// ---------------------------------------------------------------------------------------------
// Driver settings

float GetF(App &a, const char *key, float def)
{
	if (!a.vr.settings) return def;
	vr::EVRSettingsError e = vr::VRSettingsError_None;
	float v = a.vr.settings->GetFloat(kSection, key, &e);
	return e == vr::VRSettingsError_None && std::isfinite(v) ? v : def;
}

bool GetB(App &a, const char *key, bool def)
{
	if (!a.vr.settings) return def;
	vr::EVRSettingsError e = vr::VRSettingsError_None;
	bool v = a.vr.settings->GetBool(kSection, key, &e);
	return e == vr::VRSettingsError_None ? v : def;
}

// The floor normal the driver corrects for now; (0,1,0) when it corrects nothing.
V3 CurrentNormal(App &a)
{
	if (!GetB(a, "world_fix_enable", false)) return { 0, 1, 0 };
	double nx = GetF(a, "world_fix_nx", 0), nz = GetF(a, "world_fix_nz", 0), h2 = nx * nx + nz * nz;
	if (h2 >= 1.0) return { 0, 1, 0 };
	return { nx, std::sqrt(1.0 - h2), nz };
}

// The driver polls the settings while they are written one by one. The revision is made odd first
// and even (and higher) last; the driver applies nothing while it is odd or changing.
bool WriteTilt(App &a, bool enable, float nx, float nz, const float pivot[3])
{
	if (!a.vr.settings) return false;
	vr::EVRSettingsError e = vr::VRSettingsError_None, any = vr::VRSettingsError_None;
	auto note = [&] { if (e != vr::VRSettingsError_None) any = e; };
	int32_t rev = a.vr.settings->GetInt32(kSection, "world_fix_rev", &e);
	if (e != vr::VRSettingsError_None) rev = 0;
	if (rev & 1) rev++; // an earlier writer did not finish
	a.vr.settings->SetInt32(kSection, "world_fix_rev", rev + 1, &e); note();
	a.vr.settings->SetFloat(kSection, "world_fix_nx", nx, &e); note();
	a.vr.settings->SetFloat(kSection, "world_fix_nz", nz, &e); note();
	a.vr.settings->SetFloat(kSection, "world_fix_pivot_x", pivot[0], &e); note();
	a.vr.settings->SetFloat(kSection, "world_fix_pivot_y", pivot[1], &e); note();
	a.vr.settings->SetFloat(kSection, "world_fix_pivot_z", pivot[2], &e); note();
	a.vr.settings->SetBool(kSection, "world_fix_enable", enable, &e); note();
	a.vr.settings->SetInt32(kSection, "world_fix_rev", rev + 2, &e); note();
	return any == vr::VRSettingsError_None;
}

// ---------------------------------------------------------------------------------------------

bool RefreshStanding(App &a)
{
	a.vr.setup->RevertWorkingCopy();
	vr::HmdMatrix34_t m;
	if (!a.vr.setup->GetWorkingStandingZeroPoseToRawTrackingPose(&m)) return false;
	a.standingToRaw = StandingToRaw(m, a.stdToRaw);
	return true;
}

double StandingY(const App &a, V3 rawPos) { return Apply(Inverse(a.standingToRaw), rawPos).y; }

void Backup(App &a)
{
	if (a.backedUp) return;
	std::string where;
	a.backedUp = BackupChaperone(a.vr.setup, where);
	printf(a.backedUp ? "chaperone backed up to %s\n" : "WARNING: could not back up the chaperone (%s)\n", where.c_str());
}

bool ApplyFloor(App &a, double dy, bool record)
{
	Backup(a);
	a.tOwnCommit = Now(); // the commit below comes back as an event; it is ours
	if (!ShiftFloor(a.vr.setup, a.stdToRaw, dy)) return false;
	RefreshStanding(a);
	a.store.v["floor_total_m"] = a.store.Get("floor_total_m", 0) + dy;
	a.store.Save();
	if (record)
	{
		Change c;
		c.floorDy = dy;
		a.undo.push_back(c);
	}
	printf("floor %s by %.1f mm (total since reset %.1f mm)\n", dy >= 0 ? "raised" : "lowered", std::fabs(dy) * 1000, a.store.Get("floor_total_m", 0) * 1000);
	return true;
}

Change SnapshotTilt(App &a)
{
	Change c;
	c.tilt = true;
	c.prevEnable = GetB(a, "world_fix_enable", false);
	c.prevNx = GetF(a, "world_fix_nx", 0);
	c.prevNz = GetF(a, "world_fix_nz", 0);
	c.prevPivot[0] = GetF(a, "world_fix_pivot_x", 0);
	c.prevPivot[1] = GetF(a, "world_fix_pivot_y", 0);
	c.prevPivot[2] = GetF(a, "world_fix_pivot_z", 0);
	return c;
}

bool ApplyTilt(App &a, V3 normal)
{
	Change before = SnapshotTilt(a);
	float pivot[3] = { before.prevPivot[0], before.prevPivot[1], before.prevPivot[2] };
	if (!before.prevEnable)
	{
		// First correction: turn about the standing origin, which then stays where it is.
		pivot[0] = float(a.standingToRaw.m[0][3]);
		pivot[1] = float(a.standingToRaw.m[1][3]);
		pivot[2] = float(a.standingToRaw.m[2][3]);
	}
	if (!WriteTilt(a, true, float(normal.x), float(normal.z), pivot)) return false;
	a.undo.push_back(before);
	printf("tilt correction written: normal (%.5f, %.5f, %.5f), pivot (%.3f, %.3f, %.3f)\n", normal.x, normal.y, normal.z, pivot[0], pivot[1], pivot[2]);
	return true;
}

void TrackMovement(App &a)
{
	if (a.state == App::Idle) return;
	for (uint32_t i = 0; i < kMax; i++)
	{
		if (!a.tr.Ok(i) || a.tr.hist[i].empty()) continue;
		V3 cur = a.tr.hist[i].back().second;
		if (!a.haveP0[i]) { a.p0[i] = cur; a.haveP0[i] = true; }
		else if (Len(Sub(cur, a.p0[i])) > 0.10) a.moved[i] = true;
	}
}

// The lowest controller that was moved since the button press and now lies still near the floor.
// Trackers are not used: one strapped to a foot rests near the floor all the time.
bool FindResting(App &a, double now, int only, uint32_t &idx, V3 &mean)
{
	bool found = false;
	double lowest = 1e9;
	for (uint32_t i = 0; i < kMax; i++)
	{
		if (only >= 0 && int(i) != only) continue;
		if (a.tr.cls[i] != vr::TrackedDeviceClass_Controller || !a.tr.Ok(i) || !a.moved[i]) continue;
		V3 m;
		if (!a.tr.Resting(i, now, 0.002, m)) continue;
		double y = StandingY(a, m);
		if (std::fabs(y) > 0.6) continue; // in a hand or on a table, not on the floor
		auto it = a.ds.dev.find(a.tr.serial[i]);
		if (it != a.ds.dev.end() && it->second.state != "tracking" && it->second.state != "passthrough") continue;
		if (y < lowest) { lowest = y; idx = i; mean = m; found = true; }
	}
	return found;
}

std::string RestKey(const App &a, uint32_t i) { return "rest." + a.tr.model[i]; }

bool AnyLearned(const App &a)
{
	for (uint32_t i = 0; i < kMax; i++)
		if (a.tr.cls[i] == vr::TrackedDeviceClass_Controller && a.tr.raw[i].bDeviceIsConnected && a.store.Has(RestKey(a, i))) return true;
	return false;
}

void Enter(App &a, App::State s, const std::string &msg)
{
	a.state = s;
	a.tState = Now();
	a.message = msg;
	for (uint32_t i = 0; i < kMax; i++) a.haveP0[i] = a.moved[i] = false;
	if (a.haveHud) a.hud.Visible(s != App::Idle);
}

void Finish(App &a, const std::string &msg, bool good)
{
	Enter(a, App::Idle, msg);
	Beep(good ? 880 : 330, good ? 120 : 300);
}

// ---------------------------------------------------------------------------------------------

void OnButton(App &a, int id)
{
	double now = Now();
	if (id != kReset) a.tResetArmed = 0; // Reset only fires on two presses in a row
	a.tVerifyUntil = 0;                  // a pending "did the driver apply it?" must not overwrite what comes next
	RefreshStanding(a);                  // the room setup may have been changed by someone else since
	if (id == kCancel)
	{
		if (a.state != App::Idle) Enter(a, App::Idle, "Cancelled. Nothing was changed.");
		return;
	}
	if (a.state == App::Confirm)
	{
		if (id == kFloating || id == kClipping)
		{
			bool ok = ApplyFloor(a, a.pending.raiseM, true);
			Finish(a, ok ? F("Applied what was measured: floor %s %.1f cm. Press Undo to revert.", a.pending.raiseM > 0 ? "raised" : "lowered",
				std::fabs(a.pending.raiseM) * 100) : "SteamVR refused the change. Nothing was changed.", ok);
		}
		return;
	}
	if (a.state != App::Idle) return;

	switch (id)
	{
	case kFloating:
	case kClipping:
		a.report = id == kFloating ? fix::Report::Floating : fix::Report::Clipping;
		if (!AnyLearned(a))
		{
			a.message = "I do not know yet how high your controllers sit when they lie flat, so I cannot measure. Use 'Floor up 1 cm' / "
				"'Floor down 1 cm' now; once the floor is right, press 'Learn controller height' and measuring works from then on.";
			return;
		}
		Enter(a, App::Floor, F("You reported %s.\n\nNOW lay a controller flat on the floor and take your hand away. I measure once it has been still for a second.\n"
			"(One that is already lying there does not count: lift it and put it down again.)", id == kFloating ? "FLOATING" : "CLIPPING"));
		break;
	case kTilted:
		if (a.ds.hookEffective != 1)
		{
			a.message = "Levelling the world is done by the SVREnhance driver, and it is not active. Install the driver and restart SteamVR.";
			return;
		}
		if (!a.vr.settings)
		{
			a.message = "This SteamVR does not offer the settings interface I need to reach the driver.";
			return;
		}
		a.tiltPts.clear();
		a.tiltDev = -1;
		Enter(a, App::Tilt, "You reported a TILTED world.\n\nNOW lay ONE controller flat on the floor and let go. I take a sample when it is still, then you move "
			"it to another spot at least 70 cm away. Three spots in a triangle are needed.\n(One that is already lying there does not count: lift it first.)");
		break;
	case kUp:
	case kDown:
	{
		double dy = id == kUp ? kNudgeM : -kNudgeM;
		bool ok = ApplyFloor(a, dy, true);
		a.message = ok ? F("Floor %s 1 cm. Total since the last reset: %+.1f cm.", dy > 0 ? "raised" : "lowered", a.store.Get("floor_total_m", 0) * 100)
			: "SteamVR refused the change. Nothing was changed.";
		break;
	}
	case kUndo:
	{
		if (a.undo.empty()) { a.message = "Nothing to undo in this session."; return; }
		Change c = a.undo.back();
		a.undo.pop_back();
		bool ok = c.tilt ? WriteTilt(a, c.prevEnable, c.prevNx, c.prevNz, c.prevPivot) : ApplyFloor(a, -c.floorDy, false);
		a.message = !ok ? "Undo failed; nothing was changed." : c.tilt ? "Tilt correction put back to what it was." : F("Floor change of %+.1f cm undone.", c.floorDy * 100);
		break;
	}
	case kReset:
	{
		double total = a.store.Get("floor_total_m", 0);
		bool tilt = GetB(a, "world_fix_enable", false);
		if (now - a.tResetArmed > 10)
		{
			a.tResetArmed = now;
			a.message = F("Reset removes everything this tool has done: floor shift %+.1f cm, tilt correction %s.\n\nPress Reset again within 10 seconds to do it.",
				total * 100, tilt ? "on" : "off");
			return;
		}
		a.tResetArmed = 0;
		// ApplyFloor keeps floor_total_m itself: it reaches zero only if the shift really happened.
		bool floorOk = std::fabs(total) <= 1e-6 || ApplyFloor(a, -total, false);
		bool tiltOk = true;
		if (tilt)
		{
			const float zero[3] = { 0, 0, 0 };
			tiltOk = WriteTilt(a, false, 0, 0, zero);
		}
		if (floorOk && tiltOk) a.undo.clear();
		a.message = floorOk && tiltOk ? "Reset done: the floor is where Room Setup put it and the world is not being levelled."
			: F("Reset was only partly applied (floor %s, tilt %s). Nothing else was changed; try again.", floorOk ? "done" : "FAILED", tiltOk ? "done" : "FAILED");
		break;
	}
	case kLearn:
		Enter(a, App::Learn, "Only do this while your floor is RIGHT.\n\nNOW lay a controller flat on the floor and take your hand away. I remember how high this "
			"model sits, so that later I can measure a wrong floor.\n(One that is already lying there does not count: lift it and put it down again.)");
		break;
	default:
		break;
	}
}

void Step(App &a)
{
	double now = Now();
	if (a.tVerifyUntil > 0)
	{
		// The driver polls its settings, eases the change in and writes its status once a second:
		// keep looking until it reports the correction, and give up only after a generous wait.
		bool applied = a.ds.worldFixActive && std::fabs(a.ds.worldFixTiltDeg - a.verifyDeg) < 0.05;
		if (applied)
		{
			a.tVerifyUntil = 0;
			a.message = F("Levelled. The driver is correcting a tilt of %.2f degrees.\n\nIf the floor height now feels off, report floating or clipping.", a.ds.worldFixTiltDeg);
		}
		else if (now > a.tVerifyUntil)
		{
			a.tVerifyUntil = 0;
			a.message = F("The correction was stored, but after 15 seconds the driver reports %.2f degrees instead of %.2f. It needs SVREnhance driver 0.2 or newer, "
				"active in SteamVR.", a.ds.worldFixTiltDeg, a.verifyDeg);
		}
	}
	if (a.state == App::Idle) return;
	double waited = now - a.tState;
	if (waited < 2.0) return; // the press itself, and the hand moving away

	uint32_t idx = 0;
	V3 mean;
	switch (a.state)
	{
	case App::Floor:
	{
		if (waited > 45) { Finish(a, "No controller came to rest on the floor within 45 seconds. Nothing was changed.", false); return; }
		if (!FindResting(a, now, -1, idx, mean)) return;
		if (!a.store.Has(RestKey(a, idx)))
		{
			Finish(a, F("The device on the floor is a '%s', and I have not learned how high that model sits. Use one I know, or the 1 cm buttons.", a.tr.model[idx].c_str()), false);
			return;
		}
		RefreshStanding(a);
		fix::FloorDecision d = fix::DecideFloor(a.report, StandingY(a, mean), a.store.Get(RestKey(a, idx), 0));
		if (d.kind == fix::FloorDecision::OutOfRange) { Finish(a, "The measurement is more than 50 cm off, which a floor error cannot be. Run Room Setup instead. Nothing was changed.", false); return; }
		if (d.kind == fix::FloorDecision::AlreadyRight) { Finish(a, F("Your floor is right (measured error %.1f mm). If the world still feels off, try 'The world is tilted'.", d.raiseM * 1000), true); return; }
		if (d.kind == fix::FloorDecision::Contradicts)
		{
			a.pending = d;
			Enter(a, App::Confirm, F("You reported %s, but the controller says the opposite: the floor is %.1f cm too %s, so you would be %s.\n\nPress either report button "
				"to apply what was MEASURED, or Cancel.", a.report == fix::Report::Floating ? "FLOATING" : "CLIPPING", std::fabs(d.raiseM) * 100, d.raiseM > 0 ? "low" : "high",
				d.raiseM > 0 ? "floating" : "clipping"));
			Beep(660, 200);
			return;
		}
		if (d.kind == fix::FloorDecision::Large)
		{
			a.pending = d;
			Enter(a, App::Confirm, F("The controller reads %.0f cm off. That is a lot for a floor error: is it lying on the bare FLOOR, not on furniture or a step?\n\n"
				"Press either report button to move the floor by %.0f cm anyway, or Cancel.", std::fabs(d.raiseM) * 100, std::fabs(d.raiseM) * 100));
			Beep(660, 200);
			return;
		}
		bool ok = ApplyFloor(a, d.raiseM, true);
		Finish(a, ok ? F("You were %s by %.1f cm. The floor has been %s. Press Undo to revert.", d.raiseM > 0 ? "floating" : "clipping", std::fabs(d.raiseM) * 100,
			d.raiseM > 0 ? "raised" : "lowered") : "SteamVR refused the change. Nothing was changed.", ok);
		break;
	}
	case App::Confirm:
		if (waited > 30) Finish(a, "Not confirmed. Nothing was changed.", false);
		break;
	case App::Learn:
	{
		if (waited > 45) { Finish(a, "No controller came to rest on the floor within 45 seconds. Nothing was learned.", false); return; }
		if (!FindResting(a, now, -1, idx, mean)) return;
		RefreshStanding(a);
		double y = StandingY(a, mean);
		if (y < -0.03 || y > 0.2) { Finish(a, F("That reads %.1f cm, which is not a controller lying on a correct floor. Nothing was learned.", y * 100), false); return; }
		a.store.v[RestKey(a, idx)] = y;
		a.store.Save();
		Finish(a, F("Learned: a '%s' sits %.1f cm above the floor when it lies flat.", a.tr.model[idx].c_str(), y * 100), true);
		break;
	}
	case App::Tilt:
	{
		if (waited > 150) { Finish(a, "Not enough floor samples within the time. Nothing was changed.", false); return; }
		if (!FindResting(a, now, a.tiltDev, idx, mean)) return;
		for (const V3 &p : a.tiltPts)
			if (Dist({ p.x, p.z }, { mean.x, mean.z }) < 0.7) return; // still at a spot already sampled
		a.tiltDev = int(idx);
		a.tiltPts.push_back(mean);
		Beep(880, 120);
		if (a.tiltPts.size() < 3 || (!fix::SpreadOk(a.tiltPts) && a.tiltPts.size() < 6))
		{
			a.message = F("Sample %zu taken.\n\nMove the SAME controller to another spot at least 70 cm away%s.", a.tiltPts.size(),
				a.tiltPts.size() >= 3 ? ", off the line of the spots so far" : "");
			return;
		}
		fix::TiltDecision d = fix::DecideTilt(a.tiltPts, CurrentNormal(a));
		if (d.kind == fix::TiltDecision::BadSamples) { Finish(a, F("The samples do not describe a floor (spread too small, or not all on the floor; scatter %.1f mm). Nothing was changed.", d.rmsMm), false); return; }
		if (d.kind == fix::TiltDecision::Level) { Finish(a, F("The world is level (measured tilt %.2f degrees). Nothing to fix.", d.residualDeg), true); return; }
		if (d.kind == fix::TiltDecision::TooMuch) { Finish(a, F("Measured %.1f degrees. That is too much to be a calibration error: check the base stations and run Room Setup. Nothing was changed.", d.totalDeg), false); return; }
		if (!ApplyTilt(a, d.normal)) { Finish(a, "Could not store the correction in SteamVR's settings. Nothing was changed.", false); return; }
		a.verifyDeg = d.totalDeg;
		a.tVerifyUntil = now + 15.0;
		Finish(a, F("The world was tilted %.2f degrees. Levelling now; it eases in over about three seconds.", d.residualDeg), true);
		break;
	}
	default:
		break;
	}
}

// ---------------------------------------------------------------------------------------------

std::wstring W(const std::string &s) { return std::wstring(s.begin(), s.end()); } // ASCII only

void Layout(App &a)
{
	a.buttons.clear();
	auto add = [&](int id, int x, int y, int w, int h, const wchar_t *label, COLORREF c) {
		Button b;
		b.id = id;
		b.r = { x, y, x + w, y + h };
		b.label = label;
		b.color = c;
		a.buttons.push_back(b);
	};
	const int m = 36, gap = 16, w3 = (Panel::kW - 2 * m - 2 * gap) / 3, w4 = (Panel::kW - 2 * m - 3 * gap) / 4;
	add(kFloating, m, 170, w3, 120, L"I'm FLOATING\nabove the floor", RGB(58, 96, 150));
	add(kClipping, m + w3 + gap, 170, w3, 120, L"I'm CLIPPING\ninto the floor", RGB(58, 96, 150));
	add(kTilted, m + 2 * (w3 + gap), 170, w3, 120, L"The world\nis TILTED", RGB(58, 96, 150));
	add(kUp, m, 306, w4, 70, L"Floor up 1 cm", RGB(70, 78, 92));
	add(kDown, m + w4 + gap, 306, w4, 70, L"Floor down 1 cm", RGB(70, 78, 92));
	add(kUndo, m + 2 * (w4 + gap), 306, w4, 70, L"Undo", RGB(70, 78, 92));
	add(kReset, m + 3 * (w4 + gap), 306, w4, 70, L"Reset", RGB(120, 62, 58));
	add(kLearn, m, 392, 2 * w4 + gap, 62, L"Learn controller height", RGB(60, 110, 84));
	add(kCancel, m + 2 * (w4 + gap), 392, 2 * w4 + gap, 62, L"Cancel", RGB(70, 78, 92));
}

void Draw(App &a)
{
	bool busy = a.state != App::Idle;
	for (Button &b : a.buttons)
	{
		if (b.id == kCancel) b.enabled = busy;
		else if (b.id == kFloating || b.id == kClipping) b.enabled = !busy || a.state == App::Confirm;
		else b.enabled = !busy;
	}
	std::string status = F("Floor shift %+.1f cm   |   Tilt correction %s   |   Driver %s", a.store.Get("floor_total_m", 0) * 100,
		a.ds.worldFixActive ? F("%.2f deg", a.ds.worldFixTiltDeg).c_str() : "off",
		!a.ds.present ? "not found" : a.ds.hookEffective == 1 ? "active" : "not filtering");
	std::string key = status + "\n" + a.message + F("|%d", int(a.state));
	if (key == a.shown) return;
	a.shown = key;
	printf("\n--- %s\n%s\n", status.c_str(), a.message.c_str());
	a.panel.Render(L"SVREnhance Quick Fix", W(status), W(a.message), a.buttons);
	if (a.main != vr::k_ulOverlayHandleInvalid) a.vr.overlay->SetOverlayRaw(a.main, a.panel.Pixels().data(), Panel::kW, Panel::kH, 4);
	if (a.haveHud && busy) a.hud.SetText(W(a.message));
}

void DrawThumbnail(App &a)
{
	const int n = 128;
	BITMAPINFO bi = {};
	bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bi.bmiHeader.biWidth = n;
	bi.bmiHeader.biHeight = -n;
	bi.bmiHeader.biPlanes = 1;
	bi.bmiHeader.biBitCount = 32;
	bi.bmiHeader.biCompression = BI_RGB;
	void *bits = nullptr;
	HDC dc = CreateCompatibleDC(nullptr);
	HBITMAP bmp = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
	if (dc && bmp && bits)
	{
		HGDIOBJ oldBmp = SelectObject(dc, bmp);
		RECT r = { 0, 0, n, n };
		HBRUSH b = CreateSolidBrush(RGB(58, 96, 150));
		FillRect(dc, &r, b);
		DeleteObject(b);
		HFONT f = CreateFontW(-52, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		HGDIOBJ oldFont = SelectObject(dc, f);
		SetBkMode(dc, TRANSPARENT);
		SetTextColor(dc, RGB(255, 255, 255));
		DrawTextW(dc, L"FIX", -1, &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
		GdiFlush();
		std::vector<uint8_t> px(size_t(n) * n * 4);
		const uint8_t *src = static_cast<const uint8_t *>(bits);
		for (size_t i = 0; i < size_t(n) * n; i++)
		{
			px[i * 4] = src[i * 4 + 2];
			px[i * 4 + 1] = src[i * 4 + 1];
			px[i * 4 + 2] = src[i * 4];
			px[i * 4 + 3] = 255;
		}
		a.vr.overlay->SetOverlayRaw(a.thumb, px.data(), n, n, 4);
		SelectObject(dc, oldFont);
		SelectObject(dc, oldBmp);
		DeleteObject(f);
	}
	if (bmp) DeleteObject(bmp);
	if (dc) DeleteDC(dc);
}

std::string ExeDir()
{
	char p[MAX_PATH] = {};
	GetModuleFileNameA(nullptr, p, MAX_PATH);
	std::string s = p;
	size_t k = s.find_last_of("\\/");
	return k == std::string::npos ? "." : s.substr(0, k);
}

// Registers (or removes) this tool with SteamVR so that it starts together with it.
int Register(App &a, bool on)
{
	if (!a.vr.applications) { printf("This SteamVR does not offer the applications interface.\n"); return 1; }
	std::string manifest = ExeDir() + "\\svrenhance_fix.vrmanifest";
	if (on)
	{
		vr::EVRApplicationError e = a.vr.applications->AddApplicationManifest(manifest.c_str(), false);
		if (e != vr::VRApplicationError_None) { printf("AddApplicationManifest(%s) failed: %d\n", manifest.c_str(), int(e)); return 1; }
		e = a.vr.applications->SetApplicationAutoLaunch(kAppKey, true);
		if (e != vr::VRApplicationError_None) { printf("SetApplicationAutoLaunch failed: %d\n", int(e)); return 1; }
		printf("Registered. SVREnhance Quick Fix now starts with SteamVR. Undo with --unregister.\n");
		return 0;
	}
	a.vr.applications->SetApplicationAutoLaunch(kAppKey, false);
	vr::EVRApplicationError e = a.vr.applications->RemoveApplicationManifest(manifest.c_str());
	printf(e == vr::VRApplicationError_None ? "Unregistered.\n" : "RemoveApplicationManifest returned %d\n", int(e));
	return e == vr::VRApplicationError_None ? 0 : 1;
}

} // namespace

int main(int argc, char **argv)
{
	setvbuf(stdout, nullptr, _IONBF, 0);
	App a;
	int reg = 0;
	bool noHud = false;
	for (int i = 1; i < argc; i++)
	{
		std::string s = argv[i];
		if (s == "--register") reg = 1;
		else if (s == "--unregister") reg = -1;
		else if (s == "--no-hud") noHud = true;
		else
		{
			printf("usage: svrenhance_fix [--register | --unregister] [--no-hud]\n"
				   "  (no option)    open the Quick Fix panel in the SteamVR dashboard; keys work too:\n"
				   "                 F floating, C clipping, T tilted, U floor up, D floor down, Z undo, R reset, L learn, X cancel, Esc quit\n"
				   "  --register     start this tool together with SteamVR from now on\n"
				   "  --unregister   stop doing that\n"
				   "  --no-hud       no instruction panel in front of the eyes during a measurement\n");
			return 2;
		}
	}
	std::string err;
	if (!a.vr.Init(err))
	{
		printf("%s\n", err.c_str());
		return 1;
	}
	if (reg != 0)
	{
		int rc = Register(a, reg > 0);
		a.vr.Shutdown();
		return rc;
	}

	// Which way SteamVR's standing matrix goes is measured against the headset, which must be tracking.
	a.vr.setup->RevertWorkingCopy();
	vr::HmdMatrix34_t live;
	bool haveLive = a.vr.setup->GetWorkingStandingZeroPoseToRawTrackingPose(&live);
	bool decided = false;
	double tSaid = 0;
	int mismatches = 0;
	vr::TrackedDevicePose_t rawP[1], stdP[1];
	// Started together with SteamVR, the headset may not be tracking for a while: wait for it for as
	// long as SteamVR runs. Only a frame that is there and does not fit ends the tool.
	while (!decided && !a.quit)
	{
		vr::VREvent_t ev;
		while (a.vr.sys->PollNextEvent(&ev, sizeof ev))
			if (ev.eventType == vr::VREvent_Quit) a.quit = true;
		if (_kbhit() && _getch() == 27) a.quit = true;
		if (!haveLive)
		{
			a.vr.setup->RevertWorkingCopy();
			haveLive = a.vr.setup->GetWorkingStandingZeroPoseToRawTrackingPose(&live);
		}
		a.vr.sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, rawP, 1);
		a.vr.sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, stdP, 1);
		if (haveLive && rawP[0].bPoseIsValid && stdP[0].bPoseIsValid)
		{
			double e1 = 0, e2 = 0;
			decided = DetectConvention(live, Pos(rawP[0].mDeviceToAbsoluteTracking), Pos(stdP[0].mDeviceToAbsoluteTracking), a.stdToRaw, e1, e2);
			if (!decided)
			{
				printf("standing frame check: residuals %.3f / %.3f m\n", e1, e2);
				haveLive = false; // read the room setup again: it may just have been changed
				if (++mismatches >= 20) break;
			}
		}
		else if (Now() - tSaid > 5)
		{
			printf(haveLive ? "waiting for the headset to track... (Esc to quit)\n" : "waiting for a room setup to exist... (Esc to quit)\n");
			tSaid = Now();
		}
		if (!decided) Sleep(500);
	}
	if (!decided)
	{
		if (!a.quit) printf("Could not verify SteamVR's standing frame against the headset pose. Not continuing; nothing was changed.\n");
		a.vr.Shutdown();
		return a.quit ? 0 : 1;
	}
	RefreshStanding(a);
	a.store.Load();

	if (!a.panel.Init()) { printf("GDI setup failed\n"); a.vr.Shutdown(); return 1; }
	vr::EVROverlayError oe = a.vr.overlay->CreateDashboardOverlay(kAppKey, "SVREnhance Fix", &a.main, &a.thumb);
	if (oe != vr::VROverlayError_None)
	{
		printf("dashboard panel unavailable (error %d); use the keys in this window instead\n", int(oe));
		a.main = a.thumb = vr::k_ulOverlayHandleInvalid;
	}
	else
	{
		a.vr.overlay->SetOverlayWidthInMeters(a.main, 2.0f);
		a.vr.overlay->SetOverlayInputMethod(a.main, vr::VROverlayInputMethod_Mouse);
		vr::HmdVector2_t scale = { { float(Panel::kW), float(Panel::kH) } };
		a.vr.overlay->SetOverlayMouseScale(a.main, &scale);
		DrawThumbnail(a);
	}
	if (!noHud && a.hud.Create(a.vr.overlay, "svrenhance.quickfix.hud", "SVREnhance Quick Fix", err))
	{
		a.haveHud = true;
		a.hud.Visible(false);
	}
	Layout(a);
	printf("SVREnhance Quick Fix is running. Open the SteamVR dashboard and pick the 'FIX' tab, or use the keys (run with --help for the list).\n");

	double tStatus = 0;
	while (!a.quit)
	{
		double now = Now();
		a.tr.Poll(a.vr.sys, now);
		if (now - tStatus > 1.0)
		{
			a.ds = ReadDriverStatus();
			tStatus = now;
		}
		vr::VREvent_t ev;
		while (a.vr.sys->PollNextEvent(&ev, sizeof ev))
		{
			if (ev.eventType == vr::VREvent_Quit) a.quit = true;
			// A room setup saved by someone else (SteamVR's Room Setup, another tool): what this tool
			// remembered about the floor no longer describes it.
			if ((ev.eventType == vr::VREvent_ChaperoneRoomSetupCommitted || ev.eventType == vr::VREvent_ChaperoneUniverseHasChanged) && now - a.tOwnCommit > 3.0)
			{
				RefreshStanding(a);
				if (std::fabs(a.store.Get("floor_total_m", 0)) > 1e-6 || !a.undo.empty())
				{
					a.store.v["floor_total_m"] = 0;
					a.store.Save();
					a.undo.clear();
					if (a.state == App::Idle) a.message = "The room setup was changed outside Quick Fix. Its floor is the new starting point; earlier floor changes can no longer be undone.";
				}
			}
		}
		TrackMovement(a);
		if (a.main != vr::k_ulOverlayHandleInvalid)
		{
			while (a.vr.overlay->PollNextOverlayEvent(a.main, &ev, sizeof ev))
			{
				if (ev.eventType == vr::VREvent_MouseButtonDown && (ev.data.mouse.button & vr::VRMouseButton_Left))
				{
					int id = Panel::Hit(a.buttons, ev.data.mouse.x, ev.data.mouse.y);
					if (id > 0) OnButton(a, id);
				}
			}
		}
		if (_kbhit())
		{
			int c = _getch();
			if (c == 0 || c == 224) _getch();
			else
			{
				switch (toupper(c))
				{
				case 'F': OnButton(a, kFloating); break;
				case 'C': OnButton(a, kClipping); break;
				case 'T': OnButton(a, kTilted); break;
				case 'U': OnButton(a, kUp); break;
				case 'D': OnButton(a, kDown); break;
				case 'Z': OnButton(a, kUndo); break;
				case 'R': OnButton(a, kReset); break;
				case 'L': OnButton(a, kLearn); break;
				case 'X': OnButton(a, kCancel); break;
				case 27: a.quit = true; break;
				default: break;
				}
			}
		}
		Step(a);
		Draw(a);
		Sleep(15);
	}

	if (a.haveHud) a.hud.Destroy();
	if (a.main != vr::k_ulOverlayHandleInvalid) a.vr.overlay->DestroyOverlay(a.main);
	a.panel.Destroy();
	a.vr.Shutdown();
	return 0;
}
