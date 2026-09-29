// Geometry for the room-setup tool: floor plane fit, boundary trace clean-up, rectilinear snapping,
// and the largest axis-aligned play rectangle. No OpenVR dependency; tested by geom_test.cpp.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace rs {

struct V3 { double x = 0, y = 0, z = 0; };
struct P2 { double x = 0, z = 0; };

inline double Dist(P2 a, P2 b) { return std::hypot(a.x - b.x, a.z - b.z); }
inline P2 Rot(P2 p, double a) { return { p.x * std::cos(a) - p.z * std::sin(a), p.x * std::sin(a) + p.z * std::cos(a) }; }

// ---------------------------------------------------------------------------------------------
// Floor: least-squares plane through the sampled points.

struct Plane
{
	bool ok = false;
	V3 n{ 0, 1, 0 }; // unit normal, y >= 0
	V3 c;            // centroid (a point on the plane)
	double rms = 0;  // residual, metres
	double TiltRad() const { return std::acos(std::fmax(-1.0, std::fmin(1.0, n.y))); }
	// Height of the plane at (x, z).
	double YAt(double x, double z) const { return n.y > 1e-9 ? c.y - (n.x * (x - c.x) + n.z * (z - c.z)) / n.y : c.y; }
};

// Smallest-eigenvalue direction of the covariance matrix, by Jacobi rotations (3x3 symmetric).
inline Plane FitPlane(const std::vector<V3> &pts)
{
	Plane p;
	size_t n = pts.size();
	if (n < 3) return p;
	V3 c;
	for (const V3 &q : pts) { c.x += q.x; c.y += q.y; c.z += q.z; }
	c.x /= double(n); c.y /= double(n); c.z /= double(n);
	double a[3][3] = {};
	for (const V3 &q : pts)
	{
		double d[3] = { q.x - c.x, q.y - c.y, q.z - c.z };
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) a[i][j] += d[i] * d[j];
	}
	double v[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
	for (int sweep = 0; sweep < 60; sweep++)
	{
		if (std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]) < 1e-18) break;
		for (int pi = 0; pi < 3; pi++)
			for (int qi = pi + 1; qi < 3; qi++)
			{
				if (std::fabs(a[pi][qi]) < 1e-20) continue;
				double theta = (a[qi][qi] - a[pi][pi]) / (2 * a[pi][qi]);
				double t = (theta >= 0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
				double cs = 1 / std::sqrt(t * t + 1), sn = t * cs;
				for (int k = 0; k < 3; k++) { double kp = a[k][pi], kq = a[k][qi]; a[k][pi] = cs * kp - sn * kq; a[k][qi] = sn * kp + cs * kq; }
				for (int k = 0; k < 3; k++) { double pk = a[pi][k], qk = a[qi][k]; a[pi][k] = cs * pk - sn * qk; a[qi][k] = sn * pk + cs * qk; }
				for (int k = 0; k < 3; k++) { double kp = v[k][pi], kq = v[k][qi]; v[k][pi] = cs * kp - sn * kq; v[k][qi] = sn * kp + cs * kq; }
			}
	}
	int m = 0;
	for (int i = 1; i < 3; i++)
		if (a[i][i] < a[m][m]) m = i;
	V3 nrm{ v[0][m], v[1][m], v[2][m] };
	double len = std::sqrt(nrm.x * nrm.x + nrm.y * nrm.y + nrm.z * nrm.z);
	if (len < 1e-12) return p;
	nrm.x /= len; nrm.y /= len; nrm.z /= len;
	if (nrm.y < 0) { nrm.x = -nrm.x; nrm.y = -nrm.y; nrm.z = -nrm.z; }
	double ss = 0;
	for (const V3 &q : pts)
	{
		double d = (q.x - c.x) * nrm.x + (q.y - c.y) * nrm.y + (q.z - c.z) * nrm.z;
		ss += d * d;
	}
	p.n = nrm; p.c = c; p.rms = std::sqrt(ss / double(n)); p.ok = true;
	return p;
}

inline double Median(std::vector<double> v)
{
	if (v.empty()) return 0;
	std::sort(v.begin(), v.end());
	size_t n = v.size();
	return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// ---------------------------------------------------------------------------------------------
// Boundary trace clean-up.

// Drops points closer than minStep to the last kept point.
inline std::vector<P2> Decimate(const std::vector<P2> &in, double minStep)
{
	std::vector<P2> out;
	for (const P2 &p : in)
		if (out.empty() || Dist(out.back(), p) >= minStep) out.push_back(p);
	return out;
}

inline double PointSegDist(P2 p, P2 a, P2 b)
{
	double dx = b.x - a.x, dz = b.z - a.z, l2 = dx * dx + dz * dz;
	double t = l2 > 0 ? ((p.x - a.x) * dx + (p.z - a.z) * dz) / l2 : 0;
	t = std::fmax(0.0, std::fmin(1.0, t));
	return Dist(p, { a.x + t * dx, a.z + t * dz });
}

inline void DpOpen(const std::vector<P2> &pts, size_t i0, size_t i1, double tol, std::vector<bool> &keep)
{
	if (i1 <= i0 + 1) return;
	double best = -1;
	size_t bi = i0;
	for (size_t i = i0 + 1; i < i1; i++)
	{
		double d = PointSegDist(pts[i], pts[i0], pts[i1]);
		if (d > best) { best = d; bi = i; }
	}
	if (best > tol)
	{
		keep[bi] = true;
		DpOpen(pts, i0, bi, tol, keep);
		DpOpen(pts, bi, i1, tol, keep);
	}
}

// Douglas-Peucker for a closed loop: anchored at point 0 and the point farthest from it, so the
// closing edge is simplified like any other.
inline std::vector<P2> SimplifyLoop(const std::vector<P2> &pts, double tol)
{
	size_t n = pts.size();
	if (n < 4) return pts;
	// (not named "far": windows.h defines that as a macro)
	size_t split = 0;
	double fd = -1;
	for (size_t i = 1; i < n; i++)
	{
		double d = Dist(pts[0], pts[i]);
		if (d > fd) { fd = d; split = i; }
	}
	std::vector<P2> ring(pts.begin(), pts.end());
	ring.push_back(pts[0]); // close
	std::vector<bool> keep(ring.size(), false);
	keep[0] = keep[split] = true;
	DpOpen(ring, 0, split, tol, keep);
	DpOpen(ring, split, ring.size() - 1, tol, keep);
	std::vector<P2> out;
	for (size_t i = 0; i + 1 < ring.size(); i++)
		if (keep[i]) out.push_back(ring[i]);
	return out;
}

// Dominant wall direction in [-45, 45) degrees: length-weighted mean of the edge angles modulo 90.
inline double DominantAngle(const std::vector<P2> &poly)
{
	double sx = 0, sy = 0;
	size_t n = poly.size();
	for (size_t i = 0; i < n; i++)
	{
		P2 a = poly[i], b = poly[(i + 1) % n];
		double len = Dist(a, b);
		double ang = std::atan2(b.z - a.z, b.x - a.x);
		sx += len * std::cos(4 * ang);
		sy += len * std::sin(4 * ang);
	}
	return (sx == 0 && sy == 0) ? 0 : std::atan2(sy, sx) / 4;
}

// Snaps a simplified loop to a rectilinear polygon whose walls are parallel/perpendicular to
// `angle`. Runs of same-direction edges merge into one wall at their length-weighted coordinate;
// walls shorter than minEdge are absorbed into their neighbours. Returns the input unchanged when
// it cannot produce at least four walls.
inline std::vector<P2> Rectify(const std::vector<P2> &poly, double angle, double minEdge)
{
	size_t n = poly.size();
	if (n < 4) return poly;
	std::vector<P2> r;
	for (const P2 &p : poly) r.push_back(Rot(p, -angle));

	struct Wall { bool horiz; double coord; double len; };
	std::vector<Wall> walls;
	for (size_t i = 0; i < n; i++)
	{
		P2 a = r[i], b = r[(i + 1) % n];
		double dx = b.x - a.x, dz = b.z - a.z;
		bool horiz = std::fabs(dz) <= std::fabs(dx); // constant z
		double len = std::fabs(horiz ? dx : dz);
		double coord = horiz ? 0.5 * (a.z + b.z) : 0.5 * (a.x + b.x);
		if (!walls.empty() && walls.back().horiz == horiz)
		{
			Wall &w = walls.back();
			double tot = w.len + len;
			w.coord = tot > 0 ? (w.coord * w.len + coord * len) / tot : coord;
			w.len = tot;
		}
		else
		{
			walls.push_back({ horiz, coord, len });
		}
	}
	// The ring may start mid-run: merge the last wall into the first if they share a direction.
	if (walls.size() > 1 && walls.front().horiz == walls.back().horiz)
	{
		Wall &f = walls.front(), &l = walls.back();
		double tot = f.len + l.len;
		f.coord = tot > 0 ? (f.coord * f.len + l.coord * l.len) / tot : f.coord;
		f.len = tot;
		walls.pop_back();
	}
	// Absorb short walls: removing one merges its two (same-direction) neighbours.
	for (;;)
	{
		if (walls.size() <= 4) break;
		size_t si = 0;
		for (size_t i = 1; i < walls.size(); i++)
			if (walls[i].len < walls[si].len) si = i;
		if (walls[si].len >= minEdge) break;
		size_t m = walls.size();
		size_t prev = (si + m - 1) % m, next = (si + 1) % m;
		if (prev == next) break;
		Wall &a = walls[prev], &b = walls[next];
		double tot = a.len + b.len;
		a.coord = tot > 0 ? (a.coord * a.len + b.coord * b.len) / tot : a.coord;
		a.len = tot;
		if (next > si) { walls.erase(walls.begin() + next); walls.erase(walls.begin() + si); }
		else { walls.erase(walls.begin() + si); walls.erase(walls.begin() + next); }
	}
	if (walls.size() < 4 || walls.size() % 2) return poly;
	std::vector<P2> out;
	for (size_t i = 0; i < walls.size(); i++)
	{
		const Wall &a = walls[i], &b = walls[(i + 1) % walls.size()];
		if (a.horiz == b.horiz) return poly;
		P2 corner = a.horiz ? P2{ b.coord, a.coord } : P2{ a.coord, b.coord };
		out.push_back(Rot(corner, angle));
	}
	return out;
}

// ---------------------------------------------------------------------------------------------
// Polygon queries.

inline bool Inside(const std::vector<P2> &poly, P2 p)
{
	bool in = false;
	size_t n = poly.size();
	for (size_t i = 0, j = n - 1; i < n; j = i++)
	{
		const P2 &a = poly[i], &b = poly[j];
		if ((a.z > p.z) != (b.z > p.z) && p.x < (b.x - a.x) * (p.z - a.z) / (b.z - a.z) + a.x) in = !in;
	}
	return in;
}

inline double Area(const std::vector<P2> &poly)
{
	double s = 0;
	size_t n = poly.size();
	for (size_t i = 0; i < n; i++) s += poly[i].x * poly[(i + 1) % n].z - poly[(i + 1) % n].x * poly[i].z;
	return std::fabs(s) * 0.5;
}

inline double Perimeter(const std::vector<P2> &poly)
{
	double s = 0;
	size_t n = poly.size();
	for (size_t i = 0; i < n; i++) s += Dist(poly[i], poly[(i + 1) % n]);
	return s;
}

inline double DistToBoundary(const std::vector<P2> &poly, P2 p)
{
	double best = 1e9;
	size_t n = poly.size();
	for (size_t i = 0; i < n; i++) best = std::fmin(best, PointSegDist(p, poly[i], poly[(i + 1) % n]));
	return best;
}

struct Rect
{
	bool ok = false;
	P2 min, max;
	double W() const { return max.x - min.x; }
	double H() const { return max.z - min.z; }
	P2 Center() const { return { 0.5 * (min.x + max.x), 0.5 * (min.z + max.z) }; }
};

// Largest axis-aligned rectangle inside the polygon, on a grid of `cell` metres (every cell the
// rectangle covers must be inside). The polygon is expected in the frame the rectangle should be
// aligned to; rotate it first.
inline Rect LargestAxisRect(const std::vector<P2> &poly, double cell)
{
	Rect best;
	if (poly.size() < 3 || cell <= 0) return best;
	double x0 = 1e9, x1 = -1e9, z0 = 1e9, z1 = -1e9;
	for (const P2 &p : poly) { x0 = std::fmin(x0, p.x); x1 = std::fmax(x1, p.x); z0 = std::fmin(z0, p.z); z1 = std::fmax(z1, p.z); }
	int cols = int((x1 - x0) / cell) + 1, rows = int((z1 - z0) / cell) + 1;
	if (cols <= 0 || rows <= 0 || cols > 4000 || rows > 4000) return best;
	std::vector<int> h(cols, 0);
	double bestArea = 0;
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			// A cell counts only if its four corners are inside, so the rectangle never leaks out.
			double cx0 = x0 + c * cell, cz0 = z0 + r * cell;
			bool in = Inside(poly, { cx0, cz0 }) && Inside(poly, { cx0 + cell, cz0 }) && Inside(poly, { cx0, cz0 + cell }) &&
				Inside(poly, { cx0 + cell, cz0 + cell });
			h[c] = in ? h[c] + 1 : 0;
		}
		// Largest rectangle in this row's histogram.
		std::vector<int> st;
		for (int c = 0; c <= cols; c++)
		{
			int cur = c < cols ? h[c] : 0;
			while (!st.empty() && h[st.back()] >= cur)
			{
				int top = st.back();
				st.pop_back();
				int left = st.empty() ? 0 : st.back() + 1;
				int width = c - left, height = h[top];
				double area = double(width) * height;
				if (area > bestArea && width > 0 && height > 0)
				{
					bestArea = area;
					best.ok = true;
					best.min = { x0 + left * cell, z0 + (r - height + 1) * cell };
					best.max = { x0 + c * cell, z0 + (r + 1) * cell };
				}
			}
			st.push_back(c);
		}
	}
	return best;
}

} // namespace rs
