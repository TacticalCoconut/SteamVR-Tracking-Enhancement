// Decisions of the quick-fix tool, free of any SteamVR dependency so they can be tested offline:
// how far the floor is off, whether a measurement agrees with what the user reported, and which
// floor normal to hand to the driver to level the world.
#pragma once

#include "geom.h"

#include <cmath>
#include <vector>

namespace fix {

using rs::P2;
using rs::V3;

const double kPi = 3.14159265358979323846;

enum class Report { Floating, Clipping };

struct FloorDecision
{
	// Contradicts and Large are not applied without the user confirming them.
	enum Kind { Apply, AlreadyRight, Contradicts, Large, OutOfRange } kind = OutOfRange;
	double raiseM = 0; // raise the virtual floor by this much; negative lowers it
};

// yMeasured: height, in standing space, of a controller lying on the real floor.
// rest: the height that controller model has when it lies on a correct floor (learned).
// When the virtual floor is too low the user floats, and the controller on the real floor shows
// up higher than its resting height: the difference is how much the floor must be raised.
inline FloorDecision DecideFloor(Report report, double yMeasured, double rest)
{
	FloorDecision d;
	double c = yMeasured - rest;
	if (!std::isfinite(c) || std::fabs(c) > 0.5) return d;
	d.raiseM = c;
	if (std::fabs(c) < 0.003) { d.kind = FloorDecision::AlreadyRight; return d; }
	bool measuredFloating = c > 0;
	bool reportedFloating = report == Report::Floating;
	if (measuredFloating != reportedFloating && std::fabs(c) > 0.005) d.kind = FloorDecision::Contradicts;
	// A floor that is off by more than 15 cm is rare; a controller lying on something that is not
	// the floor is not. Ask.
	else if (std::fabs(c) > 0.15) d.kind = FloorDecision::Large;
	else d.kind = FloorDecision::Apply;
	return d;
}

// Without a measurement: one step in the direction that cures what was reported.
inline double Nudge(Report report, double stepM) { return report == Report::Floating ? stepM : -stepM; }

// ---------------------------------------------------------------------------------------------
// Tilt

// Rotates v by the shortest rotation that takes the unit vector n to +Y (inverse = false), or by
// its inverse. Rodrigues' formula about the axis n x Y.
inline V3 Level(V3 n, V3 v, bool inverse)
{
	double s = std::sqrt(n.x * n.x + n.z * n.z);
	if (s < 1e-12) return v;
	V3 k{ -n.z / s, 0, n.x / s };
	double a = std::atan2(s, n.y) * (inverse ? -1.0 : 1.0);
	double c = std::cos(a), sn = std::sin(a);
	V3 kxv{ k.y * v.z - k.z * v.y, k.z * v.x - k.x * v.z, k.x * v.y - k.y * v.x };
	double kv = k.x * v.x + k.y * v.y + k.z * v.z;
	return { v.x * c + kxv.x * sn + k.x * kv * (1 - c), v.y * c + kxv.y * sn + k.y * kv * (1 - c), v.z * c + kxv.z * sn + k.z * kv * (1 - c) };
}

inline double TiltDeg(V3 n)
{
	double len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
	if (len < 1e-12) return 0;
	return std::acos(std::fmax(-1.0, std::fmin(1.0, n.y / len))) * 180 / kPi;
}

// Three or more floor samples only define a plane when they are spread out: the two farthest apart
// at least 1 m, and another at least 40 cm off the line between them.
inline bool SpreadOk(const std::vector<V3> &pts)
{
	size_t n = pts.size();
	if (n < 3) return false;
	size_t a = 0, b = 1;
	double span = -1; // (not named "far": windows.h defines that as a macro)
	for (size_t i = 0; i < n; i++)
		for (size_t j = i + 1; j < n; j++)
		{
			double d = rs::Dist({ pts[i].x, pts[i].z }, { pts[j].x, pts[j].z });
			if (d > span) { span = d; a = i; b = j; }
		}
	if (span < 1.0) return false;
	double off = 0;
	for (size_t k = 0; k < n; k++)
	{
		if (k == a || k == b) continue;
		double dx = pts[b].x - pts[a].x, dz = pts[b].z - pts[a].z;
		double d = std::fabs(dx * (pts[k].z - pts[a].z) - dz * (pts[k].x - pts[a].x)) / span;
		off = std::fmax(off, d);
	}
	return off >= 0.4;
}

struct TiltDecision
{
	enum Kind { Apply, Level, TooMuch, BadSamples } kind = BadSamples;
	V3 normal{ 0, 1, 0 }; // the floor's normal in UNcorrected tracking space: what the driver stores
	double residualDeg = 0; // the tilt the user has now
	double totalDeg = 0;    // the tilt the driver would correct in total
	double rmsMm = 0;
};

// floorPts: positions of a controller lying on the floor, in tracking space as applications see it,
// i.e. with the driver's current correction already in it. current: the normal the driver holds now
// ((0,1,0) when it holds none).
inline TiltDecision DecideTilt(const std::vector<V3> &floorPts, V3 current, double maxTiltDeg = 5.0)
{
	TiltDecision d;
	if (!SpreadOk(floorPts)) return d;
	rs::Plane p = rs::FitPlane(floorPts);
	d.rmsMm = p.rms * 1000;
	if (!p.ok || p.rms > 0.01) return d; // the samples do not lie on one plane: not all on the floor
	d.residualDeg = TiltDeg(p.n);
	// The measured normal is expressed in corrected space; take the current correction out again.
	d.normal = Level(current, p.n, true);
	d.totalDeg = TiltDeg(d.normal);
	if (d.residualDeg < 0.15) { d.kind = TiltDecision::Level; return d; }
	d.kind = d.totalDeg > maxTiltDeg ? TiltDecision::TooMuch : TiltDecision::Apply;
	return d;
}

} // namespace fix
