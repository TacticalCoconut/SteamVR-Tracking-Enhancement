// Offline tests for the pose pipeline: synthetic 1 kHz (or irregular) pose streams with injected
// reflection glitches, real corrections, dropouts and noise. No SteamVR needed.
#include "../driver/src/common.h"

#include <cstdarg>
#include <cstdio>
#include <random>
#include <vector>

namespace svr { extern double g_testNow; }
using namespace svr;

namespace {

struct FakeContext : vr::IVRDriverContext
{
	void *GetGenericInterface(const char *, vr::EVRInitError *e) override
	{
		if (e) *e = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}
	vr::DriverHandle_t GetDriverHandle() override { return 0; }
};

int g_failed = 0, g_passed = 0;

void Check(bool ok, const char *name, const char *fmt, ...)
{
	char buf[512];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof buf, fmt, ap);
	va_end(ap);
	printf("[%s] %-58s %s\n", ok ? "PASS" : "FAIL", name, buf);
	(ok ? g_passed : g_failed)++;
}

vr::DriverPose_t MakePose(V3 p, Q r = QIdentity(), vr::ETrackingResult res = vr::TrackingResult_Running_OK, bool valid = true)
{
	vr::DriverPose_t d{};
	d.qWorldFromDriverRotation = QIdentity();
	d.qDriverFromHeadRotation = QIdentity();
	ToArr(p, d.vecPosition);
	d.qRotation = r;
	d.result = res;
	d.poseIsValid = valid;
	d.deviceIsConnected = true;
	return d;
}

uint32_t g_nextIdx = 1;

uint32_t NewDevice(int cls, bool lighthouse = true)
{
	uint32_t i = g_nextIdx++;
	g_dev[i].infoState.store(2);
	g_dev[i].cls = cls;
	g_dev[i].lighthouse = lighthouse;
	g_dev[i].serial = "TEST";
	g_dev[i].lastConnected = true;
	return i;
}

vr::DriverPose_t Step(uint32_t idx, double t, vr::DriverPose_t p)
{
	g_testNow = t;
	ProcessPose(idx, p);
	return p;
}

V3 Out(const vr::DriverPose_t &p) { return FromArr(p.vecPosition); }

const double kPi = 3.14159265358979323846;

V3 Circle(double t, double r = 0.3, double f = 0.5)
{
	return { r * std::sin(2 * kPi * f * t), 1.2 + 0.1 * std::sin(2 * kPi * 0.3 * t), r * std::cos(2 * kPi * f * t) };
}

Q AxisAngle(V3 axis, double a)
{
	double s = std::sin(a / 2) / Len(axis);
	return { std::cos(a / 2), axis.x * s, axis.y * s, axis.z * s };
}

void TestCleanPassthrough()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	double maxDev = 0;
	for (int k = 0; k <= 2000; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t);
		auto o = Step(id, t, MakePose(truth));
		maxDev = std::fmax(maxDev, Len(Out(o) - truth));
	}
	auto &st = g_dev[id].st;
	Check(maxDev < 1e-9 && st.glitches == 0 && st.relocations == 0, "clean controller motion is untouched (zero added latency)",
		"max dev %.2e m, glitches %u, relocations %u", maxDev, st.glitches, st.relocations);
}

void TestReflectionGlitch()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	double maxErr = 0, maxRawErr = 0;
	for (int k = 0; k <= 3000; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t);
		V3 raw = truth;
		if (t >= 1.0 && t < 1.008) raw = raw + V3{ 0.15, 0, 0 }; // 8 ms reflection spike
		auto o = Step(id, t, MakePose(raw));
		maxErr = std::fmax(maxErr, Len(Out(o) - truth));
		maxRawErr = std::fmax(maxRawErr, Len(raw - truth));
	}
	auto &st = g_dev[id].st;
	Check(maxErr < 0.01 && st.glitches == 1 && st.relocations == 0, "15 cm / 8 ms reflection spike is rejected",
		"raw err %.1f cm -> output err %.2f cm, glitches %u, relocations %u", maxRawErr * 100, maxErr * 100, st.glitches, st.relocations);
}

void TestRealRelocation()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	const ClassCfg &c = CurSettings().controller;
	double maxStepExcess = 0, finalDev = 0;
	V3 prevOut, prevTruth;
	for (int k = 0; k <= 2000; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t);
		V3 raw = t >= 1.0 ? truth + V3{ 0, 0, 0.05 } : truth; // lighthouse re-solves 5 cm away and stays there
		auto o = Step(id, t, MakePose(raw));
		if (k > 0) maxStepExcess = std::fmax(maxStepExcess, Len((Out(o) - prevOut) - (truth - prevTruth)));
		if (t >= 1.0 + c.confirmS + c.blendS + 0.02) finalDev = std::fmax(finalDev, Len(Out(o) - raw));
		prevOut = Out(o);
		prevTruth = truth;
	}
	auto &st = g_dev[id].st;
	Check(st.relocations == 1 && st.glitches == 0 && finalDev < 1e-6 && maxStepExcess < 0.003, "persistent 5 cm correction is accepted and eased in",
		"relocations %u, glitches %u, residual %.1e m, worst per-ms snap %.2f mm", st.relocations, st.glitches, finalDev, maxStepExcess * 1000);
}

void TestLargeRelocationSnaps()
{
	// Re-acquired 1.2 m away: once confirmed, the output must be at the raw pose at once, not slide there.
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	const ClassCfg &c = CurSettings().controller;
	double residual = -1;
	for (int k = 0; k <= 2000; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t);
		V3 raw = t >= 1.0 ? truth + V3{ 1.2, 0, 0 } : truth;
		auto o = Step(id, t, MakePose(raw));
		if (t >= 1.0 + c.confirmS + 0.005 && residual < 0) residual = Len(Out(o) - raw);
	}
	auto &st = g_dev[id].st;
	Check(st.relocations == 1 && residual >= 0 && residual < 1e-9, "1.2 m re-acquisition snaps instead of sliding",
		"relocations %u, residual right after confirm %.1e m", st.relocations, residual);
}

void TestShortDropout()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	double holdY = 0, maxYDrop = 0, maxRawYDrop = 0, maxStepExcess = 0, finalDev = 0;
	bool allValidOk = true;
	V3 prevOut, prevTruth;
	for (int k = 0; k <= 2000; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t);
		bool lost = t >= 1.0 && t < 1.2;
		V3 raw = truth;
		if (lost) raw.y -= 0.5 * 9.8 * (t - 1.0) * (t - 1.0); // IMU-only dead reckoning falling away
		auto o = Step(id, t, MakePose(raw, QIdentity(), lost ? vr::TrackingResult_Running_OutOfRange : vr::TrackingResult_Running_OK));
		if (k == 999) holdY = Out(o).y;
		if (lost)
		{
			allValidOk &= o.poseIsValid && o.result == vr::TrackingResult_Running_OK;
			maxYDrop = std::fmax(maxYDrop, holdY - Out(o).y);
			maxRawYDrop = std::fmax(maxRawYDrop, holdY - raw.y);
		}
		if (t >= 1.2 && k > 0) maxStepExcess = std::fmax(maxStepExcess, Len((Out(o) - prevOut) - (truth - prevTruth)));
		if (t >= 1.2 + CurSettings().controller.blendS + 0.01) finalDev = std::fmax(finalDev, Len(Out(o) - truth));
		prevOut = Out(o);
		prevTruth = truth;
	}
	auto &st = g_dev[id].st;
	Check(allValidOk && maxYDrop < 0.03 && st.bridges == 1 && st.bridgesExpired == 0 && finalDev < 1e-6 && maxStepExcess < 0.005,
		"200 ms optical dropout is bridged without fly-away",
		"raw fell %.1f cm, output fell %.2f cm, bridges %u, recovery snap %.2f mm/ms, residual %.1e", maxRawYDrop * 100, maxYDrop * 100, st.bridges,
		maxStepExcess * 1000, finalDev);
}

void TestLongDropout()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	bool passthroughAfter = true;
	for (int k = 0; k <= 2000; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t);
		bool lost = t >= 1.0 && t < 1.6;
		auto in = MakePose(truth, QIdentity(), lost ? vr::TrackingResult_Running_OutOfRange : vr::TrackingResult_Running_OK);
		auto o = Step(id, t, in);
		if (t >= 1.0 + CurSettings().controller.bridgeMaxS + 0.01 && lost)
			passthroughAfter &= Len(Out(o) - truth) < 1e-12 && o.result == vr::TrackingResult_Running_OutOfRange;
	}
	auto &st = g_dev[id].st;
	Check(passthroughAfter && st.bridgesExpired == 1, "dropout longer than bridge_max is handed back to SteamVR",
		"bridges %u, expired %u, passthrough after limit %s", st.bridges, st.bridgesExpired, passthroughAfter ? "yes" : "no");
}

void TestFastSwingNoFalsePositives()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	std::mt19937 rng(7);
	std::uniform_real_distribution<double> u(0, 1);
	double t = 0, maxDev = 0;
	while (t < 4.0)
	{
		V3 truth = { 0.8 * std::sin(2 * kPi * 1.5 * t), 1.2 + 0.4 * std::sin(2 * kPi * 2.0 * t), 0.3 * std::cos(2 * kPi * 1.5 * t) };
		auto o = Step(id, t, MakePose(truth));
		maxDev = std::fmax(maxDev, Len(Out(o) - truth));
		double r = u(rng);
		t += r < 0.01 ? 0.03 : (r < 0.02 ? 0.02 : 0.002); // 500 Hz with occasional radio stalls
	}
	auto &st = g_dev[id].st;
	Check(st.glitches == 0 && st.relocations == 0 && maxDev < 1e-9, "9 m/s, ~100 m/s^2 swings with 20-30 ms stalls: no false rejects",
		"glitches %u, relocations %u, max dev %.1e", st.glitches, st.relocations, maxDev);

	// Same swing at a steady 1 kHz, with a 12 cm reflection spike mid-swing.
	uint32_t g = NewDevice(vr::TrackedDeviceClass_Controller);
	double maxErr = 0;
	for (int k = 0; k <= 3000; k++)
	{
		double tt = k * 0.001;
		V3 truth = { 0.8 * std::sin(2 * kPi * 1.5 * tt), 1.2 + 0.4 * std::sin(2 * kPi * 2.0 * tt), 0.3 * std::cos(2 * kPi * 1.5 * tt) };
		V3 raw = (tt >= 1.5 && tt < 1.506) ? truth + V3{ 0, 0, 0.12 } : truth;
		auto o = Step(g, tt, MakePose(raw));
		maxErr = std::fmax(maxErr, Len(Out(o) - truth));
	}
	auto &sg = g_dev[g].st;
	Check(sg.glitches == 1 && sg.relocations == 0 && maxErr < 0.02, "12 cm spike during a 9 m/s swing is still rejected",
		"glitches %u, relocations %u, output err %.2f cm", sg.glitches, sg.relocations, maxErr * 100);
}

void TestRotationGlitch()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	double maxAng = 0;
	for (int k = 0; k <= 2000; k++)
	{
		double t = k * 0.001;
		Q truth = AxisAngle({ 0, 1, 0 }, 0.5 * t);
		Q raw = (t >= 1.0 && t < 1.005) ? QMul(AxisAngle({ 1, 0, 0 }, kPi / 2), truth) : truth;
		auto p = MakePose({ 0, 1, 0 }, raw);
		p.vecAngularVelocity[1] = 0.5;
		auto o = Step(id, t, p);
		maxAng = std::fmax(maxAng, QAngle(o.qRotation, truth));
	}
	auto &st = g_dev[id].st;
	Check(st.glitches == 1 && maxAng < 2 * kPi / 180, "90 degree / 5 ms orientation flip is rejected", "glitches %u, max output error %.2f deg",
		st.glitches, maxAng * 180 / kPi);
}

void TestTrackerSmoothing()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_GenericTracker);
	std::mt19937 rng(11);
	std::normal_distribution<double> n(0, 0.001);
	double sumRaw = 0, sumOut = 0;
	int cnt = 0;
	for (int k = 0; k <= 3000; k++)
	{
		double t = k * 0.001;
		V3 truth{ 0, 1, 0 };
		V3 raw = truth + V3{ n(rng), n(rng), n(rng) };
		auto o = Step(id, t, MakePose(raw));
		if (t > 0.5)
		{
			sumRaw += Dot(raw - truth, raw - truth);
			sumOut += Dot(Out(o) - truth, Out(o) - truth);
			cnt++;
		}
	}
	double rawRms = std::sqrt(sumRaw / cnt), outRms = std::sqrt(sumOut / cnt);
	double jitter = g_dev[id].st.jitterMm;
	Check(outRms < rawRms * 0.35 && g_dev[id].st.glitches == 0, "tracker jitter filter cuts 1 mm noise", "raw rms %.3f mm -> output rms %.3f mm",
		rawRms * 1000, outRms * 1000);
	Check(jitter > 0.8 && jitter < 3.0, "jitter metric reports the noise level", "jitter_mm %.2f (1 mm per-axis sigma)", jitter);

	uint32_t mv = NewDevice(vr::TrackedDeviceClass_GenericTracker);
	double lag = 0;
	for (int k = 0; k <= 2000; k++)
	{
		double t = k * 0.001;
		V3 truth{ 1.0 * t, 1, 0 }; // 1 m/s
		auto o = Step(mv, t, MakePose(truth));
		if (t > 1.0) lag = std::fmax(lag, Len(Out(o) - truth));
	}
	Check(lag < 0.015, "tracker filter lag at 1 m/s", "%.1f mm", lag * 1000);
}

void TestHmd()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_HMD);
	double maxErr = 0;
	bool dropoutPassthrough = true;
	for (int k = 0; k <= 3000; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t, 0.2, 0.3);
		V3 raw = (t >= 1.0 && t < 1.01) ? truth + V3{ 0, 0.2, 0 } : truth;
		bool lost = t >= 2.0 && t < 2.1;
		auto o = Step(id, t, MakePose(raw, QIdentity(), lost ? vr::TrackingResult_Running_OutOfRange : vr::TrackingResult_Running_OK));
		if (t < 2.0) maxErr = std::fmax(maxErr, Len(Out(o) - truth));
		if (lost) dropoutPassthrough &= o.result == vr::TrackingResult_Running_OutOfRange && Len(Out(o) - raw) < 1e-12;
	}
	auto &st = g_dev[id].st;
	Check(st.glitches == 1 && maxErr < 0.01, "HMD 20 cm / 10 ms spike is rejected", "glitches %u, output err %.2f cm", st.glitches, maxErr * 100);
	Check(dropoutPassthrough && st.bridges == 0, "HMD tracking loss is never hidden", "bridges %u", st.bridges);
}

void TestNonLighthousePassthrough()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller, false);
	double maxDev = 0;
	for (int k = 0; k <= 1500; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t);
		V3 raw = (t >= 1.0 && t < 1.008) ? truth + V3{ 0.15, 0, 0 } : truth;
		auto o = Step(id, t, MakePose(raw));
		maxDev = std::fmax(maxDev, Len(Out(o) - raw));
	}
	Check(maxDev < 1e-12, "non-lighthouse devices pass through untouched", "max dev %.1e", maxDev);
}

void TestNaNAngularVelocity()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	double maxDev = 0;
	for (int k = 0; k <= 1500; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t);
		auto p = MakePose(truth);
		p.vecAngularVelocity[0] = std::nan("");
		auto o = Step(id, t, p);
		maxDev = std::fmax(maxDev, Len(Out(o) - truth));
	}
	auto &st = g_dev[id].st;
	Check(maxDev < 1e-9 && st.glitches == 0 && st.relocations == 0, "NaN angular velocity from a driver does not trip the gate",
		"max dev %.1e, glitches %u, relocations %u", maxDev, st.glitches, st.relocations);
}

void TestTimeStepsBackwardDuringHold()
{
	// A controller moving at 2 m/s loses optical tracking; while bridged, the driver's poseTimeOffset
	// jumps back by 300 ms. The held pose must keep decaying, never grow.
	uint32_t id = NewDevice(vr::TrackedDeviceClass_Controller);
	V3 holdPt;
	double maxFromHold = 0;
	for (int k = 0; k <= 1300; k++)
	{
		double t = k * 0.001;
		V3 truth{ 2.0 * t, 1.2, 0 };
		bool lost = t >= 1.0 && t < 1.25;
		auto p = MakePose(truth, QIdentity(), lost ? vr::TrackingResult_Running_OutOfRange : vr::TrackingResult_Running_OK);
		if (k == 1100) p.poseTimeOffset = -0.3;
		auto o = Step(id, t, p);
		if (k == 999) holdPt = Out(o);
		if (lost) maxFromHold = std::fmax(maxFromHold, Len(Out(o) - holdPt));
	}
	Check(maxFromHold < 0.12, "clock stepping back 300 ms during a bridge does not launch the pose",
		"max distance from hold point %.1f cm (coast bound at 2 m/s is 8 cm)", maxFromHold * 100);
}

void TestNaNPoseTimeOffset()
{
	uint32_t id = NewDevice(vr::TrackedDeviceClass_GenericTracker);
	int nanOut = 0;
	double maxDev = 0;
	for (int k = 0; k <= 1500; k++)
	{
		double t = k * 0.001;
		V3 truth = Circle(t);
		auto p = MakePose(truth);
		if (k == 700) p.poseTimeOffset = std::nan("");
		auto o = Step(id, t, p);
		if (!PoseFinite(o)) nanOut++;
		else if (k > 720) maxDev = std::fmax(maxDev, Len(Out(o) - truth));
	}
	Check(nanOut == 0 && maxDev < 0.02, "NaN poseTimeOffset on a smoothed tracker never reaches the output", "NaN poses out %d, max dev %.1f mm",
		nanOut, maxDev * 1000);
}

V3 CrossV(V3 a, V3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }

void TestWorldFix()
{
	// The universe is tilted 1.5 deg: the real floor is the plane through `pivot` with normal n.
	double tilt = 1.5 * kPi / 180;
	V3 n{ 0, std::cos(tilt), std::sin(tilt) }, pivot{ 0.3, -2.5, 1.0 };
	bool set = SetWorldFix(true, n, pivot, 0.0, 0.0);
	uint32_t a = NewDevice(vr::TrackedDeviceClass_Controller), b = NewDevice(vr::TrackedDeviceClass_GenericTracker);
	uint32_t h = NewDevice(vr::TrackedDeviceClass_HMD), other = NewDevice(vr::TrackedDeviceClass_Controller, false);
	uint32_t atPivot = NewDevice(vr::TrackedDeviceClass_GenericTracker);
	V3 e1{ 1, 0, 0 }, e2 = CrossV(n, e1);
	V3 pa = pivot + e1 * 1.4 + e2 * -0.9, pb = pivot + e1 * -1.1 + e2 * 1.7; // two spots on the real floor
	V3 ph = pa + n * 1.7;                                                   // a head 1.7 m above the first one
	auto oa = Step(a, 1.0, MakePose(pa)), ob = Step(b, 1.0, MakePose(pb)), oh = Step(h, 1.0, MakePose(ph));
	auto op = Step(atPivot, 1.0, MakePose(pivot)), oo = Step(other, 1.0, MakePose(pa));
	V3 wa = WorldPos(oa), wb = WorldPos(ob), wh = WorldPos(oh);
	double level = std::fmax(std::fabs(wa.y - pivot.y), std::fabs(wb.y - pivot.y));
	double rigid = std::fabs(Len(wa - wb) - Len(pa - pb));
	double upright = Len(wh - (wa + V3{ 0, 1.7, 0 }));
	Check(set && level < 1e-9 && rigid < 1e-9 && upright < 1e-9 && Len(WorldPos(op) - pivot) < 1e-9,
		"world fix levels a 1.5 deg tilt, rigidly, about the pivot", "floor height error %.1e m, distance error %.1e m, head-above-feet error %.1e m",
		level, rigid, upright);
	Check(Len(Out(oa) - pa) < 1e-12 && Len(WorldPos(oo) - pa) < 1e-12, "world fix leaves driver space and other tracking systems alone",
		"driver-space change %.1e, non-lighthouse change %.1e", Len(Out(oa) - pa), Len(WorldPos(oo) - pa));

	// Refused: a tilt that cannot be a calibration error, and a broken normal.
	bool big = SetWorldFix(true, { 0, std::cos(8 * kPi / 180), std::sin(8 * kPi / 180) }, pivot, 0.0, 2.0);
	V3 afterBig = WorldPos(Step(a, 2.0, MakePose(pa)));
	bool nan = SetWorldFix(true, { std::nan(""), 1, 0 }, pivot, 0.0, 3.0);
	V3 afterNan = WorldPos(Step(a, 3.0, MakePose(pa)));
	Check(!big && !nan && Len(afterBig - pa) < 1e-12 && Len(afterNan - pa) < 1e-12, "8 deg tilt and NaN normal are refused (correction off)",
		"accepted %d/%d, residual %.1e / %.1e", int(big), int(nan), Len(afterBig - pa), Len(afterNan - pa));
	bool farPivot = SetWorldFix(true, n, { 0, 0, 5000 }, 0.0, 3.5);
	V3 afterFar = WorldPos(Step(a, 3.5, MakePose(pa)));
	Check(!farPivot && Len(afterFar - pa) < 1e-12, "a pivot 5 km away is refused", "accepted %d, residual %.1e", int(farPivot), Len(afterFar - pa));
	SetWorldFix(false, { 0, 1, 0 }, {}, 0.0, 4.0);
}

void TestWorldFixSettings()
{
	// The driver polls while another process writes the correction setting by setting.
	bool midWrite = WorldFixReady(true, 5, 5, 4, true, true);      // writer made the revision odd and is still writing
	bool straddled = WorldFixReady(true, 4, 6, 4, true, true);     // the revision changed while the driver was reading
	bool complete = WorldFixReady(true, 6, 6, 4, true, true);      // a complete new set
	bool unchanged = WorldFixReady(true, 6, 6, 6, true, true);     // nothing new
	bool atStart = WorldFixReady(false, 6, 6, 0, true, false);     // what was stored when SteamVR started
	bool brokenStart = WorldFixReady(false, 7, 7, 0, true, false); // a writer died half-way before SteamVR started
	bool offByHand = WorldFixReady(true, 6, 6, 6, false, true);    // the user switched it off in the settings file
	Check(!midWrite && !straddled && complete && !unchanged && atStart && !brokenStart && offByHand,
		"correction settings are applied only as a complete set", "mid-write %d, straddled %d, complete %d, unchanged %d, start %d, broken %d, off %d",
		int(midWrite), int(straddled), int(complete), int(unchanged), int(atStart), int(brokenStart), int(offByHand));

	// A device the driver has not identified yet, while a correction is in force.
	SetWorldFix(true, { 0, std::cos(2 * kPi / 180), std::sin(2 * kPi / 180) }, { 0, 0, 0 }, 0.0, 10.0);
	uint32_t id = g_nextIdx++; // infoState 0: never seen
	auto first = Step(id, 10.0, MakePose({ 1, 1, 2 })), second = Step(id, 10.011, MakePose({ 1, 1, 2 }));
	bool held = !first.poseIsValid && !second.poseIsValid && first.result == vr::TrackingResult_Calibrating_InProgress;
	g_dev[id].cls = vr::TrackedDeviceClass_Controller; // what ResolveDeviceInfo does on the next RunFrame
	g_dev[id].lighthouse = true;
	g_dev[id].infoState.store(2);
	auto third = Step(id, 10.022, MakePose({ 1, 1, 2 }));
	bool corrected = third.poseIsValid && Len(WorldPos(third) - V3{ 1, 1, 2 }) > 0.01;
	SetWorldFix(false, { 0, 1, 0 }, {}, 0.0, 11.0);
	uint32_t plain = g_nextIdx++;
	auto untouched = Step(plain, 11.0, MakePose({ 1, 1, 2 }));
	Check(held && corrected && untouched.poseIsValid, "an unidentified device is held back only while a correction is active",
		"held %d, then corrected %d, without correction passes %d", int(held), int(corrected), int(untouched.poseIsValid));
}

void TestWorldFixEasing()
{
	// Switched on, changed and switched off while running: the world must never jump.
	SetWorldFix(false, { 0, 1, 0 }, {}, 0.0, 0.0);
	uint32_t id = NewDevice(vr::TrackedDeviceClass_HMD);
	V3 p{ 2.0, 1.6, 0.5 }, pivot{ 0, 0, 0 };
	V3 n1{ std::sin(2 * kPi / 180), std::cos(2 * kPi / 180), 0 }, n2{ 0, std::cos(1 * kPi / 180), std::sin(1 * kPi / 180) };
	double maxStep = 0, errOn = -1, errOff = -1;
	V3 prev = p;
	Q r1 = LevelingRotation(n1);
	V3 target1 = QRotate(r1, p - pivot) + pivot;
	for (int k = 0; k <= 5000; k++)
	{
		double t = k * 0.001;
		if (k == 1000) SetWorldFix(true, n1, pivot, 0.8, t);
		if (k == 2500) SetWorldFix(true, n2, V3{ 0.5, 0, -0.5 }, 0.8, t); // new tilt and new pivot, mid-session
		if (k == 4000) SetWorldFix(false, { 0, 1, 0 }, {}, 0.8, t);
		V3 w = WorldPos(Step(id, t, MakePose(p)));
		maxStep = std::fmax(maxStep, Len(w - prev));
		prev = w;
		if (k == 2400) errOn = Len(w - target1);
		if (k == 5000) errOff = Len(w - p);
	}
	Check(maxStep < 0.0003 && errOn < 1e-9 && errOff < 1e-9, "world fix is eased on, changed and off without a jump",
		"largest step %.3f mm per ms, residual on %.1e, off %.1e", maxStep * 1000, errOn, errOff);
	SetWorldFix(false, { 0, 1, 0 }, {}, 0.0, 6.0);
}

void TestStationMonitor()
{
	// Four stations. Once the baselines exist (30 s) the whole set is re-aligned, as a universe re-solve
	// does to every reported station pose at once; later one station is bumped 2 cm. Only the bump may be
	// reported, and only for that station.
	uint32_t s[4];
	V3 p[4] = { { -1.5, 2.2, -1.5 }, { 1.5, 2.2, -1.5 }, { 1.5, 2.2, 1.5 }, { -1.5, 2.2, 1.5 } };
	Q r[4] = { AxisAngle({ 0, 1, 0 }, 0.7), AxisAngle({ 0, 1, 0 }, -0.7), AxisAngle({ 0, 1, 0 }, 2.4), AxisAngle({ 0, 1, 0 }, -2.4) };
	for (int i = 0; i < 4; i++) s[i] = NewDevice(vr::TrackedDeviceClass_TrackingReference);
	bool falseFlag = false;
	for (int k = 0; k <= 90; k++)
	{
		double t = k * 1.0;
		Q realign = t >= 45 ? AxisAngle({ 0, 1, 0 }, 10 * kPi / 180) : QIdentity();
		V3 shift = t >= 45 ? V3{ 0.3, -0.1, 0.2 } : V3{};
		for (int i = 0; i < 4; i++)
		{
			V3 q = QRotate(realign, p[i]) + shift;
			if (t >= 70 && i == 2) q = q + V3{ 0.02, 0, 0 };
			Step(s[i], t, MakePose(q, QMul(realign, r[i])));
		}
		AnalyzeStations(t);
		if (t < 70)
			for (int i = 0; i < 4; i++) falseFlag |= StationStatus(s[i]).moved;
	}
	StationReport a = StationStatus(s[0]), b = StationStatus(s[1]), c = StationStatus(s[2]), d = StationStatus(s[3]);
	bool othersClear = !a.moved && !b.moved && !d.moved && a.moveEvents + b.moveEvents + d.moveEvents == 0;
	Check(a.baseline && !falseFlag && c.moved && c.moveEvents == 1 && othersClear && std::fabs(c.driftM - 0.02) < 0.002,
		"universe re-alignment ignored, the one bumped station is named",
		"flags before bump %d; bumped: moved %d, events %u, drift %.1f mm; others clear %d", int(falseFlag), int(c.moved), c.moveEvents,
		c.driftM * 1000, int(othersClear));
}

} // namespace

int main()
{
	static FakeContext ctx;
	vr::VRDriverContext() = &ctx;
	LoadSettings(); // no IVRSettings available -> built-in defaults

	TestCleanPassthrough();
	TestReflectionGlitch();
	TestRealRelocation();
	TestLargeRelocationSnaps();
	TestShortDropout();
	TestLongDropout();
	TestFastSwingNoFalsePositives();
	TestRotationGlitch();
	TestTrackerSmoothing();
	TestHmd();
	TestNonLighthousePassthrough();
	TestNaNAngularVelocity();
	TestTimeStepsBackwardDuringHold();
	TestNaNPoseTimeOffset();
	TestStationMonitor();
	TestWorldFix();
	TestWorldFixSettings();
	TestWorldFixEasing();

	printf("\n%d passed, %d failed\n", g_passed, g_failed);
	return g_failed ? 1 : 0;
}
