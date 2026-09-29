// Offline tests for the quick-fix decisions.
#include "fixmath.h"

#include <cstdarg>
#include <cstdio>
#include <random>

using namespace fix;

namespace {

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

std::mt19937 g_rng(5);
double Noise(double sigma) { return std::normal_distribution<double>(0, sigma)(g_rng); }

V3 Normal(double tiltDeg, double headingDeg)
{
	double t = tiltDeg * kPi / 180, h = headingDeg * kPi / 180;
	return { std::sin(t) * std::cos(h), std::cos(t), std::sin(t) * std::sin(h) };
}

// Controller positions on a floor with the given normal through `origin`, with sensor noise.
std::vector<V3> FloorSamples(V3 n, V3 origin, const std::vector<P2> &spots, double rest, double sigma)
{
	std::vector<V3> out;
	for (const P2 &s : spots)
	{
		double y = origin.y - (n.x * (s.x - origin.x) + n.z * (s.z - origin.z)) / n.y;
		out.push_back({ s.x + n.x * rest + Noise(sigma), y + n.y * rest + Noise(sigma), s.z + n.z * rest + Noise(sigma) });
	}
	return out;
}

void TestFloor()
{
	const double rest = 0.031;
	// Virtual floor 4.2 cm too low: the user floats, the resting controller reads 4.2 cm too high.
	FloorDecision a = DecideFloor(Report::Floating, rest + 0.042, rest);
	Check(a.kind == FloorDecision::Apply && std::fabs(a.raiseM - 0.042) < 1e-12, "floating 4.2 cm: floor is raised 4.2 cm", "raise %.1f mm", a.raiseM * 1000);
	FloorDecision b = DecideFloor(Report::Clipping, rest - 0.027, rest);
	Check(b.kind == FloorDecision::Apply && std::fabs(b.raiseM + 0.027) < 1e-12, "clipping 2.7 cm: floor is lowered 2.7 cm", "raise %.1f mm", b.raiseM * 1000);
	FloorDecision c = DecideFloor(Report::Floating, rest - 0.03, rest);
	Check(c.kind == FloorDecision::Contradicts && c.raiseM < 0, "report says floating, measurement says clipping: asks first", "measured %.1f mm", c.raiseM * 1000);
	FloorDecision d = DecideFloor(Report::Clipping, rest + 0.002, rest);
	Check(d.kind == FloorDecision::AlreadyRight, "2 mm error: floor is already right", "measured %.1f mm", d.raiseM * 1000);
	// A controller left on a sofa 45 cm up agrees with "floating" and must still not be believed unasked.
	FloorDecision sofa = DecideFloor(Report::Floating, 0.45, rest);
	Check(sofa.kind == FloorDecision::Large, "a 42 cm error is only applied after confirmation", "measured %.0f mm", sofa.raiseM * 1000);
	FloorDecision e = DecideFloor(Report::Floating, rest + 0.8, rest), f = DecideFloor(Report::Floating, std::nan(""), rest);
	Check(e.kind == FloorDecision::OutOfRange && f.kind == FloorDecision::OutOfRange, "80 cm and NaN are refused", "");
	Check(Nudge(Report::Floating, 0.01) == 0.01 && Nudge(Report::Clipping, 0.01) == -0.01, "nudges go the curing way", "");
}

void TestTilt()
{
	std::vector<P2> spots = { { -1.0, -0.8 }, { 1.1, -0.9 }, { 0.2, 1.2 }, { -0.9, 1.0 } };
	V3 origin{ 0.3, -2.5, 0.4 };

	// No correction yet, the universe is tilted 1.2 deg.
	V3 n = Normal(1.2, 40);
	TiltDecision a = DecideTilt(FloorSamples(n, origin, spots, 0.031, 0.0005), { 0, 1, 0 });
	double err = TiltDeg(Level(a.normal, n, false));
	Check(a.kind == TiltDecision::Apply && err < 0.06 && std::fabs(a.totalDeg - 1.2) < 0.06, "1.2 deg tilt is measured from four floor samples",
		"measured %.3f deg, direction error %.3f deg, rms %.2f mm", a.totalDeg, err, a.rmsMm);

	// The driver already corrects 1.0 deg and 0.5 deg remain in another direction: the stored normal
	// must become the total, not the remainder.
	V3 stored = Normal(1.0, 40), truth = Normal(1.4, 70);
	V3 seen = Level(stored, truth, false); // the real floor's normal as it appears through the active correction
	TiltDecision b = DecideTilt(FloorSamples(seen, origin, spots, 0.031, 0.0005), stored);
	double errB = TiltDeg(Level(b.normal, truth, false));
	Check(b.kind == TiltDecision::Apply && errB < 0.06, "a second fix composes with the correction already active",
		"residual seen %.3f deg, new total %.3f deg, error after fix %.3f deg", b.residualDeg, b.totalDeg, errB);

	TiltDecision c = DecideTilt(FloorSamples({ 0, 1, 0 }, origin, spots, 0.031, 0.0005), { 0, 1, 0 });
	Check(c.kind == TiltDecision::Level, "level floor: nothing to fix", "residual %.3f deg", c.residualDeg);

	TiltDecision d = DecideTilt(FloorSamples(Normal(7.5, 10), origin, spots, 0.031, 0.0005), { 0, 1, 0 });
	Check(d.kind == TiltDecision::TooMuch, "7.5 deg is not a calibration error: refused", "measured %.2f deg", d.totalDeg);

	// Samples that cannot define a plane, or do not lie on one.
	std::vector<P2> line = { { -1.0, 0 }, { 0, 0.05 }, { 1.0, 0 } }, close = { { 0, 0 }, { 0.3, 0 }, { 0, 0.3 } };
	TiltDecision e = DecideTilt(FloorSamples(n, origin, line, 0.031, 0.0005), { 0, 1, 0 });
	TiltDecision f = DecideTilt(FloorSamples(n, origin, close, 0.031, 0.0005), { 0, 1, 0 });
	std::vector<V3> lifted = FloorSamples(n, origin, spots, 0.031, 0.0005);
	lifted[2].y += 0.20; // one sample taken on a chair
	TiltDecision g = DecideTilt(lifted, { 0, 1, 0 });
	Check(e.kind == TiltDecision::BadSamples && f.kind == TiltDecision::BadSamples && g.kind == TiltDecision::BadSamples,
		"collinear, bunched or off-floor samples are refused", "off-floor rms %.1f mm", g.rmsMm);
}

void TestLevel()
{
	V3 n = Normal(3.0, 200);
	V3 y = Level(n, n, false), back = Level(n, y, true);
	double a = std::sqrt(y.x * y.x + (y.y - 1) * (y.y - 1) + y.z * y.z);
	double b = std::sqrt((back.x - n.x) * (back.x - n.x) + (back.y - n.y) * (back.y - n.y) + (back.z - n.z) * (back.z - n.z));
	Check(a < 1e-12 && b < 1e-12, "levelling rotation takes the normal to +Y and back", "%.1e / %.1e", a, b);
}

} // namespace

int main()
{
	TestFloor();
	TestTilt();
	TestLevel();
	printf("\n%d passed, %d failed\n", g_passed, g_failed);
	return g_failed ? 1 : 0;
}
