// Shared state for the SVREnhance driver: settings, per-device filter state, globals.
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include <openvr_driver.h>

#include "svr_math.h"

namespace svr {

static const char *const kSection = "driver_svrenhance";

// Per device-class tuning. Times are seconds, distances metres, angles radians.
struct ClassCfg
{
	bool gate = false;       // reject pose jumps that no real motion could produce (reflections)
	bool bridge = false;     // hold a stable pose through short optical dropouts
	bool smooth = false;     // One Euro jitter filter
	double jumpM = 0.03;     // unexplained position jump that makes a sample suspect
	double rotJumpRad = 0.35;
	double maxAccel = 150.0; // m/s^2 allowance added on top of jumpM
	double confirmS = 0.05;  // a new position must hold this long before it's accepted as real
	double suspectMaxS = 0.2;
	double bridgeMaxS = 0.3;
	double blendS = 0.15;    // how long corrections are eased in
	double blendMaxM = 0.5;  // larger corrections snap: the device was lost and re-acquired elsewhere
	double minCutoff = 3.0;  // One Euro, Hz
	double beta = 20.0;      // One Euro, Hz per m/s
	double rotBeta = 2.0;    // One Euro, Hz per rad/s
};

struct Settings
{
	// Not "enable": driver_<name>.enable is SteamVR's own switch for loading the whole driver.
	bool filterEnable = true;
	bool lighthouseOnly = true;
	ClassCfg hmd, controller, tracker;
	double stationMoveM = 0.01;
	double stationMoveRad = 0.0087; // 0.5 degree
	double stationBaselineS = 30.0; // let the universe alignment settle before taking station baselines

	// World correction (see WorldFix). Written by the quick-fix tool through IVRSettings.
	bool worldFixEnable = false;
	double worldFixNx = 0, worldFixNz = 0; // horizontal components of the real floor's unit normal
	double worldFixPivot[3] = { 0, 0, 0 };
	double worldFixEaseS = 0.8;
	bool worldFixAllSystems = false;
	int worldFixRev = 0; // odd while a writer is in the middle of a change
};

// The correction is several settings, written one by one by another process while the driver polls.
// The writer makes world_fix_rev odd before it starts and even (and higher) when it is done; the
// driver reads the revision before and after the values and applies them only when both readings
// agree, are even, and are new. Switching the correction off by hand is always honoured.
inline bool WorldFixReady(bool loaded, int revBefore, int revAfter, int lastRev, bool enable, bool lastEnable)
{
	if (revBefore != revAfter || (revAfter & 1)) return false;
	if (!loaded) return true;
	if (revAfter != lastRev) return true;
	return !enable && lastEnable;
}

// World correction: a rigid rotation of the whole tracking space about a pivot, which levels a tilted
// universe (SteamVR's chaperone stores only a yaw, so it cannot). Every lighthouse device gets the
// same rotation, so the devices keep their positions relative to each other. Changes are eased in,
// because this moves the user's view.
// The correction is p' = R p + T with T = pivot - R pivot (a rotation about the pivot); easing
// interpolates R and T together, so every intermediate state is rigid and a changed pivot is eased too.
struct WorldFix
{
	bool active = false; // false only while both ends are the identity
	Q fromR = QIdentity(), toR = QIdentity();
	V3 fromT, toT;
	double tChange = 0, easeS = 0.8;
};
static const double kWorldFixMaxTiltRad = 5.0 * 3.14159265358979323846 / 180.0;

extern WorldFix g_worldFix[2];
extern std::atomic<int> g_worldFixIdx;
// floorNormal: the real floor's normal as measured in uncorrected tracking space. A tilt beyond
// kWorldFixMaxTiltRad is not a calibration error; it is refused and the correction is switched off.
bool SetWorldFix(bool enable, V3 floorNormal, V3 pivot, double easeS, double now);
void WorldFixAt(const WorldFix &w, double now, Q &r, V3 &t);
// The rotation that takes the unit vector n to +Y by the shortest way.
Q LevelingRotation(V3 n);

// The settings block is double-buffered: RunFrame fills the idle slot and flips the index.
extern Settings g_settings[2];
extern std::atomic<int> g_settingsIdx;
inline const Settings &CurSettings() { return g_settings[g_settingsIdx.load(std::memory_order_acquire)]; }
void LoadSettings();

enum class Mode : uint8_t { Idle, Track, Suspect, Bridge, Blend, Lost };

struct OneEuro
{
	bool init = false;
	V3 x, dx;
	Q q = QIdentity();
	void Reset(V3 p, Q r) { init = true; x = p; dx = {}; q = r; }
};

struct DeviceStats
{
	uint64_t updates = 0;
	double maxGapS = 0;
	uint32_t gaps50ms = 0;       // update stream stalls >= 50 ms while connected (radio / USB)
	uint32_t glitches = 0;       // transient jumps rejected and discarded
	uint32_t relocations = 0;    // sustained jumps accepted, eased in
	uint32_t bridges = 0;        // dropouts bridged
	uint32_t bridgesExpired = 0; // dropouts longer than bridge_max
	double bridgedS = 0;
	double suspectS = 0;
	uint32_t disconnects = 0;
	uint32_t trackingLost = 0;   // Running_OK -> anything else
	double jitterMm = 0;         // high-frequency position noise while still
	double lastJumpM = 0;
};

// Base-station (TrackingReference) samples recorded on the hot path. The analysis (each station's
// geometry relative to the others, which a universe re-alignment leaves untouched but a bump
// changes) runs on the telemetry thread; see AnalyzeStations.
struct StationMon
{
	bool validNow = false;
	uint32_t invalidTransitions = 0;
	uint32_t samples = 0;
	uint32_t poseChanges = 0; // samples whose driver-space position differs > 2 mm from the previous one
	V3 posD;                  // driver space
	Q rotD = QIdentity();
	V3 posW;                  // world space, for display and event locations
};

struct Device
{
	std::mutex m;
	// 0 = never seen, 1 = waiting for RunFrame to read properties, 2 = ready, 3 = gave up.
	// The identity fields below are written once, before infoState is published as 2, and never
	// change afterwards, so readers that see 2 (acquire) may read them without taking `m`.
	std::atomic<int> infoState{ 0 };
	int infoTries = 0;
	int cls = 0;
	bool lighthouse = false;
	std::string serial, model, system, modeLabel;
	std::atomic<uint64_t> firmware{ 0 }; // stations: OOTX data arrives after activation, re-read until non-zero
	int fwTries = 0;

	Mode mode = Mode::Idle;
	double tLast = 0;     // pose time (clock + poseTimeOffset), kept monotonic
	double tLastWall = 0; // plain clock, for rate/gap statistics
	double tFirstSeen = 0;
	bool lastConnected = false;
	bool lastOk = false;

	bool haveGood = false;
	V3 goodPos; Q goodRot = QIdentity(); double tGood = 0; V3 vel;
	V3 outPos; Q outRot = QIdentity();

	double tSuspect = 0; V3 candPos; Q candRot = QIdentity(); double tCandStart = 0;
	double tHold = 0; V3 holdPos; V3 holdVel; Q holdRot = QIdentity();
	double tBlend = 0; V3 blendOffPos; Q blendOffRot = QIdentity(); double blendS = 0.15;

	OneEuro euro;
	V3 jitterFast, jitterSlow; bool jitterInit = false;

	// Last raw pose, for telemetry (world position of events / coverage map).
	vr::DriverPose_t lastRaw{};
	bool haveRaw = false;

	DeviceStats st;
	StationMon stn;
};

static const uint32_t kMaxDevices = vr::k_unMaxTrackedDeviceCount;
extern Device g_dev[kMaxDevices];

double Now();

// filter.cpp
void ProcessPose(uint32_t idx, vr::DriverPose_t &pose);
void ResolveDeviceInfo(); // RunFrame thread
void ResetDevices();      // Init, before the hook exists (the DLL is pinned, so statics survive a reload)

// hook.cpp
void ResetHookState(); // Init
void EnsurePoseHook(); // safe to call from any thread; installs once per Init
void RemovePoseHook();
uint64_t HookCalls();
bool HookInstalled();
int HookEffective(); // -1 unknown, 0 no pose traffic reached the hook, 1 yes
void CheckHookEffective(); // RunFrame thread

// telemetry.cpp
enum class Ev : uint8_t { Glitch, Relocation, BridgeStart, BridgeEnd, BridgeExpired, Disconnect, Reconnect, Gap, TrackingLost, StationMoved, StationLost, StationBack };
void PushEvent(Ev kind, uint32_t idx, V3 world, double magnitude, double durationS);
void StartTelemetry();
void StopTelemetry();
void Log(const char *fmt, ...);

struct StationReport
{
	bool baseline = false; // has at least one baselined pair
	bool moved = false;
	uint32_t moveEvents = 0;
	double driftM = 0, driftRad = 0, maxDriftM = 0;
	int pairs = 0;
};
void AnalyzeStations(double now); // snapshots all devices and runs the pairwise station check (telemetry thread; tests)
StationReport StationStatus(uint32_t idx);

} // namespace svr
