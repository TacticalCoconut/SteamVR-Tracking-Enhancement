// Small vector / quaternion helpers for the pose pipeline.
// Quaternions use OpenVR's HmdQuaternion_t layout {w, x, y, z}.
#pragma once

#include <cmath>
#include <openvr_driver.h>

namespace svr {

struct V3 { double x = 0, y = 0, z = 0; };

inline V3 operator+(V3 a, V3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline V3 operator-(V3 a, V3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline V3 operator*(V3 a, double s) { return { a.x * s, a.y * s, a.z * s }; }
inline double Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 Cross(V3 a, V3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
inline double Len(V3 a) { return std::sqrt(Dot(a, a)); }
inline V3 FromArr(const double a[3]) { return { a[0], a[1], a[2] }; }
inline void ToArr(V3 v, double a[3]) { a[0] = v.x; a[1] = v.y; a[2] = v.z; }

using Q = vr::HmdQuaternion_t;

inline Q QIdentity() { return { 1, 0, 0, 0 }; }
inline Q QConj(Q q) { return { q.w, -q.x, -q.y, -q.z }; }

inline Q QMul(Q a, Q b)
{
	return {
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

inline Q QNorm(Q q)
{
	double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n < 1e-12) return QIdentity();
	return { q.w / n, q.x / n, q.y / n, q.z / n };
}

inline double QDot(Q a, Q b) { return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; }

// Angle in radians of the rotation that takes a to b.
inline double QAngle(Q a, Q b)
{
	double d = std::fabs(QDot(a, b));
	if (d > 1.0) d = 1.0;
	return 2.0 * std::acos(d);
}

inline Q QSlerp(Q a, Q b, double t)
{
	double d = QDot(a, b);
	if (d < 0) { b = { -b.w, -b.x, -b.y, -b.z }; d = -d; }
	if (d > 0.9995)
	{
		return QNorm({ a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t });
	}
	double th = std::acos(d);
	double s = std::sin(th);
	double wa = std::sin((1 - t) * th) / s, wb = std::sin(t * th) / s;
	return { a.w * wa + b.w * wb, a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb };
}

inline V3 QRotate(Q q, V3 v)
{
	V3 u{ q.x, q.y, q.z };
	double s = q.w;
	return u * (2.0 * Dot(u, v)) + v * (s * s - Dot(u, u)) + Cross(u, v) * (2.0 * s);
}

inline bool Finite(double v) { return std::isfinite(v); }

inline bool PoseFinite(const vr::DriverPose_t &p)
{
	for (int i = 0; i < 3; i++)
		if (!Finite(p.vecPosition[i]) || !Finite(p.vecVelocity[i]) || !Finite(p.vecWorldFromDriverTranslation[i])) return false;
	const Q &r = p.qRotation;
	return Finite(r.w) && Finite(r.x) && Finite(r.y) && Finite(r.z);
}

// Driver-space position -> tracking-universe ("world") position, using the transform the
// driver itself supplies in the pose.
inline V3 WorldPos(const vr::DriverPose_t &p)
{
	return QRotate(p.qWorldFromDriverRotation, FromArr(p.vecPosition)) + FromArr(p.vecWorldFromDriverTranslation);
}

inline Q WorldRot(const vr::DriverPose_t &p) { return QMul(p.qWorldFromDriverRotation, p.qRotation); }

// Low-pass blend factor for a first-order filter with the given cutoff, sampled every dt seconds.
inline double Alpha(double cutoffHz, double dt)
{
	if (cutoffHz <= 0 || dt <= 0) return 1.0;
	double tau = 1.0 / (2.0 * 3.14159265358979323846 * cutoffHz);
	return 1.0 / (1.0 + tau / dt);
}

inline double Clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double SmoothStep(double s) { s = Clamp(s, 0, 1); return s * s * (3 - 2 * s); }

} // namespace svr
