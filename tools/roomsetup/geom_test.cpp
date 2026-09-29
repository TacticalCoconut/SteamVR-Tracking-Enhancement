// Offline tests for the room-setup geometry.
#include "geom.h"

#include <cstdarg>
#include <cstdio>
#include <random>

using namespace rs;

namespace {

int g_failed = 0, g_passed = 0;
const double kPi = 3.14159265358979323846;

void Check(bool ok, const char *name, const char *fmt, ...)
{
	char buf[512];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof buf, fmt, ap);
	va_end(ap);
	printf("[%s] %-56s %s\n", ok ? "PASS" : "FAIL", name, buf);
	(ok ? g_passed : g_failed)++;
}

std::mt19937 g_rng(3);
double Noise(double sigma) { return std::normal_distribution<double>(0, sigma)(g_rng); }

// Walks the polygon's edges with the given step and per-sample noise, like a hand trace.
std::vector<P2> Trace(const std::vector<P2> &poly, double step, double sigma)
{
	std::vector<P2> out;
	size_t n = poly.size();
	for (size_t i = 0; i < n; i++)
	{
		P2 a = poly[i], b = poly[(i + 1) % n];
		double len = Dist(a, b);
		int k = std::max(1, int(len / step));
		for (int s = 0; s < k; s++)
		{
			double t = double(s) / k;
			out.push_back({ a.x + (b.x - a.x) * t + Noise(sigma), a.z + (b.z - a.z) * t + Noise(sigma) });
		}
	}
	return out;
}

void TestPlane()
{
	// Floor tilted 0.6 degrees, 2 mm noise, 5 points as a user would place them.
	double tilt = 0.6 * kPi / 180, ax = std::tan(tilt);
	std::vector<V3> pts;
	for (P2 p : { P2{ -1.2, -1.2 }, P2{ 1.2, -1.2 }, P2{ 1.2, 1.2 }, P2{ -1.2, 1.2 }, P2{ 0, 0 } })
		pts.push_back({ p.x, 1.0 + ax * p.x + Noise(0.002), p.z });
	Plane pl = FitPlane(pts);
	double tiltErr = std::fabs(pl.TiltRad() - tilt) * 180 / kPi;
	Check(pl.ok && tiltErr < 0.1 && pl.rms < 0.004 && std::fabs(pl.YAt(0, 0) - 1.0) < 0.004, "floor plane fit recovers a 0.6 deg tilt",
		"tilt err %.3f deg, rms %.1f mm, height err %.1f mm", tiltErr, pl.rms * 1000, (pl.YAt(0, 0) - 1.0) * 1000);

	std::vector<V3> flat = { { 0, 0.5, 0 }, { 1, 0.5, 0 }, { 0, 0.5, 1 }, { 1, 0.5, 1 } };
	Plane f = FitPlane(flat);
	Check(f.ok && f.TiltRad() < 1e-9 && std::fabs(f.n.y - 1) < 1e-9, "level floor gives tilt 0", "tilt %.2e", f.TiltRad());
}

void TestSquareTrace()
{
	std::vector<P2> room = { { -1.5, -1.2 }, { 1.5, -1.2 }, { 1.5, 1.2 }, { -1.5, 1.2 } };
	std::vector<P2> tr = Trace(room, 0.01, 0.004);
	std::vector<P2> simp = SimplifyLoop(Decimate(tr, 0.02), 0.03);
	double ang = DominantAngle(simp);
	std::vector<P2> rect = Rectify(simp, ang, 0.10);
	bool four = rect.size() == 4;
	double w = four ? Dist(rect[0], rect[1]) : 0, h = four ? Dist(rect[1], rect[2]) : 0;
	bool dims = four && ((std::fabs(w - 3.0) < 0.01 && std::fabs(h - 2.4) < 0.01) || (std::fabs(w - 2.4) < 0.01 && std::fabs(h - 3.0) < 0.01));
	Check(dims && std::fabs(ang) < 0.5 * kPi / 180, "noisy hand trace of a 3.0 x 2.4 room snaps to 4 walls",
		"%zu samples -> %zu after DP -> %zu walls, %.3f x %.3f m, angle %.2f deg", tr.size(), simp.size(), rect.size(), w, h, ang * 180 / kPi);
}

void TestRotatedRoom()
{
	double a = 20 * kPi / 180;
	std::vector<P2> room;
	for (P2 p : { P2{ -1.5, -1.2 }, P2{ 1.5, -1.2 }, P2{ 1.5, 1.2 }, P2{ -1.5, 1.2 } }) room.push_back(Rot(p, a));
	std::vector<P2> simp = SimplifyLoop(Decimate(Trace(room, 0.01, 0.004), 0.02), 0.03);
	double ang = DominantAngle(simp);
	Check(std::fabs(ang - a) * 180 / kPi < 0.5, "dominant wall angle of a room rotated 20 deg", "%.2f deg", ang * 180 / kPi);
	std::vector<P2> rect = Rectify(simp, ang, 0.10);
	std::vector<P2> back;
	for (const P2 &p : rect) back.push_back(Rot(p, -ang));
	Rect r = LargestAxisRect(back, 0.05);
	Check(rect.size() == 4 && r.ok && std::fabs(r.W() - 3.0) < 0.06 && std::fabs(r.H() - 2.4) < 0.06 && Dist(r.Center(), {}) < 0.05,
		"play rectangle inside the rotated room", "%zu walls, %.2f x %.2f m, centre off by %.1f cm", rect.size(), r.W(), r.H(),
		Dist(r.Center(), {}) * 100);
}

void TestAlcove()
{
	// L-shaped room: 4 x 3 with a 1.5 x 1 corner cut out.
	std::vector<P2> room = { { 0, 0 }, { 4, 0 }, { 4, 3 }, { 1.5, 3 }, { 1.5, 2 }, { 0, 2 } };
	std::vector<P2> simp = SimplifyLoop(Decimate(Trace(room, 0.01, 0.004), 0.02), 0.03);
	std::vector<P2> rect = Rectify(simp, DominantAngle(simp), 0.10);
	Check(rect.size() == 6 && std::fabs(Area(rect) - 10.5) < 0.05, "L-shaped room keeps its alcove", "%zu walls, area %.2f m2 (10.5)",
		rect.size(), Area(rect));
	Rect r = LargestAxisRect(rect, 0.05);
	Check(r.ok && std::fabs(r.W() * r.H() - 8.0) < 0.3, "largest rectangle in the L (4 x 2 = 8 m2)", "%.2f x %.2f = %.2f m2", r.W(), r.H(),
		r.W() * r.H());

	// A 5 cm bump in a wall (a skirting board or a hand wobble) is absorbed, not kept as a wall.
	std::vector<P2> bumpy = { { 0, 0 }, { 2, 0 }, { 2, 0.05 }, { 2.2, 0.05 }, { 2.2, 0 }, { 4, 0 }, { 4, 3 }, { 0, 3 } };
	std::vector<P2> rb = Rectify(bumpy, 0, 0.10);
	Check(rb.size() == 4 && std::fabs(Area(rb) - 12.0) < 0.05, "5 cm wall bump is absorbed", "%zu walls, area %.2f", rb.size(), Area(rb));
}

void TestQueries()
{
	std::vector<P2> sq = { { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } };
	Check(Inside(sq, { 0.5, 0.5 }) && !Inside(sq, { 1.5, 0 }) && std::fabs(Area(sq) - 4) < 1e-9 && std::fabs(Perimeter(sq) - 8) < 1e-9 &&
			std::fabs(DistToBoundary(sq, { 0.7, 0 }) - 0.3) < 1e-9,
		"inside / area / perimeter / boundary distance", "");
}

} // namespace

int main()
{
	TestPlane();
	TestSquareTrace();
	TestRotatedRoom();
	TestAlcove();
	TestQueries();
	printf("\n%d passed, %d failed\n", g_passed, g_failed);
	return g_failed ? 1 : 0;
}
