// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-08-24. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <openvr_driver.h>
#include <cmath>

constexpr double POSE_PI = 3.14159265358979323846;

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const vr::HmdVector3d_t& vector) {
	return quaternionRotateVector(quat, vector.v);
}

inline vr::HmdQuaternion_t quaternionNormalize(vr::HmdQuaternion_t q) {
	double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n > 0.0) {
		q.w /= n; q.x /= n; q.y /= n; q.z /= n;
	}
	return q;
}

inline vr::HmdQuaternion_t quaternionConjugate(const vr::HmdQuaternion_t& q) {
	return { q.w, -q.x, -q.y, -q.z };
}

inline vr::HmdQuaternion_t quaternionProjectYaw(const vr::HmdQuaternion_t& q) {
	double n = std::sqrt(q.w * q.w + q.y * q.y);
	if (n < 1e-9)
		return { 1, 0, 0, 0 };
	return { q.w / n, 0.0, q.y / n, 0.0 };
}

inline vr::HmdVector3d_t quaternionAngularVelocity(const vr::HmdQuaternion_t& cur, const vr::HmdQuaternion_t& prev, double dt) {
	if (dt <= 0.0)
		return { 0, 0, 0 };

	vr::HmdQuaternion_t d = quaternionNormalize(cur * quaternionConjugate(prev));
	if (d.w < 0.0) { d.w = -d.w; d.x = -d.x; d.y = -d.y; d.z = -d.z; }

	double s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
	if (s < 1e-9)
		return { 0, 0, 0 };

	double scale = (2.0 * std::atan2(s, d.w)) / (s * dt);
	return { d.x * scale, d.y * scale, d.z * scale };
}

inline double quaternionYawRad(const vr::HmdQuaternion_t& q) {
	double fwd[3] = { 0.0, 0.0, -1.0 };
	vr::HmdVector3d_t f = quaternionRotateVector(q, fwd);
	return std::atan2(-f.v[0], -f.v[2]);
}

inline double quaternionYawDeg(const vr::HmdQuaternion_t& q) {
	return quaternionYawRad(q) * 180.0 / POSE_PI;
}

inline vr::HmdQuaternion_t quaternionFromYaw(double yaw) {
	return { std::cos(yaw * 0.5), 0.0, std::sin(yaw * 0.5), 0.0 };
}

inline double wrapDeg(double d) {
	while (d > 180.0) d -= 360.0;
	while (d < -180.0) d += 360.0;
	return d;
}

inline double wrapRad(double r) {
	while (r > POSE_PI) r -= 2.0 * POSE_PI;
	while (r < -POSE_PI) r += 2.0 * POSE_PI;
	return r;
}

inline vr::HmdQuaternion_t quaternionFromAngularVelocity(const double(&omega)[3], double dt) {
	double wx = omega[0] * dt, wy = omega[1] * dt, wz = omega[2] * dt;
	double angle = std::sqrt(wx * wx + wy * wy + wz * wz);
	if (angle < 1e-12)
		return { 1, 0, 0, 0 };
	double s = std::sin(angle * 0.5) / angle;
	return { std::cos(angle * 0.5), wx * s, wy * s, wz * s };
}

inline vr::HmdQuaternion_t quaternionFromRotationVector(const vr::HmdVector3d_t& v) {
	return quaternionFromAngularVelocity(v.v, 1.0);
}

inline vr::HmdVector3d_t quaternionToRotationVector(vr::HmdQuaternion_t q) {
	q = quaternionNormalize(q);
	if (q.w < 0.0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
	double s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
	if (s < 1e-12)
		return { 0, 0, 0 };
	double scale = 2.0 * std::atan2(s, q.w) / s;
	return { q.x * scale, q.y * scale, q.z * scale };
}

inline double quaternionAngleRad(const vr::HmdQuaternion_t& q) {
	vr::HmdVector3d_t v = quaternionToRotationVector(q);
	return std::sqrt(v.v[0] * v.v[0] + v.v[1] * v.v[1] + v.v[2] * v.v[2]);
}

template < class T >
inline vr::HmdQuaternion_t HmdQuaternion_FromMatrix(const T& matrix)
{
	double m[3][3];
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			m[i][j] = matrix.m[i][j];
			if (!std::isfinite(m[i][j])) return { 1, 0, 0, 0 };
		}

	// Recover the largest quaternion component first. At a half-turn the
	// antisymmetric differences vanish, so they cannot determine axis signs;
	// the symmetric sums below retain the relative signs of the other axes.
	double diagonal[4] = {
		1 + m[0][0] + m[1][1] + m[2][2],
		1 + m[0][0] - m[1][1] - m[2][2],
		1 - m[0][0] + m[1][1] - m[2][2],
		1 - m[0][0] - m[1][1] + m[2][2]
	};
	int largest = 0;
	for (int i = 1; i < 4; ++i)
		if (diagonal[i] > diagonal[largest]) largest = i;
	const double s = 2.0 * std::sqrt(diagonal[largest]);
	vr::HmdQuaternion_t q;
	switch (largest)
	{
	case 0: q = { 0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s }; break;
	case 1: q = { (m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s }; break;
	case 2: q = { (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s }; break;
	default: q = { (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s }; break;
	}
	return quaternionNormalize(q);
}

inline vr::HmdVector3d_t vecAdd(const vr::HmdVector3d_t& a, const vr::HmdVector3d_t& b) {
	return { a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2] };
}

inline vr::HmdVector3d_t vecSub(const vr::HmdVector3d_t& a, const vr::HmdVector3d_t& b) {
	return { a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2] };
}

inline vr::HmdVector3d_t vecScale(const vr::HmdVector3d_t& a, double s) {
	return { a.v[0] * s, a.v[1] * s, a.v[2] * s };
}

inline double vecDot(const vr::HmdVector3d_t& a, const vr::HmdVector3d_t& b) {
	return a.v[0] * b.v[0] + a.v[1] * b.v[1] + a.v[2] * b.v[2];
}

inline vr::HmdVector3d_t vecCross(const vr::HmdVector3d_t& a, const vr::HmdVector3d_t& b) {
	return {
		a.v[1] * b.v[2] - a.v[2] * b.v[1],
		a.v[2] * b.v[0] - a.v[0] * b.v[2],
		a.v[0] * b.v[1] - a.v[1] * b.v[0]
	};
}

inline double vecNorm(const vr::HmdVector3d_t& a) {
	return std::sqrt(vecDot(a, a));
}

inline vr::HmdVector3d_t vecFromArray(const double(&a)[3]) {
	return { a[0], a[1], a[2] };
}

struct Mat3
{
	double m[3][3];

	static Mat3 zero() {
		Mat3 r;
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) r.m[i][j] = 0.0;
		return r;
	}

	static Mat3 identity() {
		Mat3 r = zero();
		r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
		return r;
	}

	static Mat3 fromQuaternion(const vr::HmdQuaternion_t& q) {
		Mat3 r;
		double w = q.w, x = q.x, y = q.y, z = q.z;
		r.m[0][0] = 1 - 2 * (y * y + z * z); r.m[0][1] = 2 * (x * y - z * w);     r.m[0][2] = 2 * (x * z + y * w);
		r.m[1][0] = 2 * (x * y + z * w);     r.m[1][1] = 1 - 2 * (x * x + z * z); r.m[1][2] = 2 * (y * z - x * w);
		r.m[2][0] = 2 * (x * z - y * w);     r.m[2][1] = 2 * (y * z + x * w);     r.m[2][2] = 1 - 2 * (x * x + y * y);
		return r;
	}

	vr::HmdVector3d_t mul(const vr::HmdVector3d_t& v) const {
		return {
			m[0][0] * v.v[0] + m[0][1] * v.v[1] + m[0][2] * v.v[2],
			m[1][0] * v.v[0] + m[1][1] * v.v[1] + m[1][2] * v.v[2],
			m[2][0] * v.v[0] + m[2][1] * v.v[1] + m[2][2] * v.v[2]
		};
	}

	vr::HmdVector3d_t mulTransposed(const vr::HmdVector3d_t& v) const {
		return {
			m[0][0] * v.v[0] + m[1][0] * v.v[1] + m[2][0] * v.v[2],
			m[0][1] * v.v[0] + m[1][1] * v.v[1] + m[2][1] * v.v[2],
			m[0][2] * v.v[0] + m[1][2] * v.v[1] + m[2][2] * v.v[2]
		};
	}

	void addScaled(const Mat3& o, double s) {
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) m[i][j] += o.m[i][j] * s;
	}

	void scale(double s) {
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) m[i][j] *= s;
	}
};

inline bool solveLinearSystem(int n, double* a, double* b, double* x)
{
	for (int col = 0; col < n; col++)
	{
		int pivot = col;
		for (int r = col + 1; r < n; r++)
			if (std::fabs(a[r * n + col]) > std::fabs(a[pivot * n + col]))
				pivot = r;
		if (std::fabs(a[pivot * n + col]) < 1e-12)
			return false;
		if (pivot != col)
		{
			for (int c = 0; c < n; c++) { double t = a[col * n + c]; a[col * n + c] = a[pivot * n + c]; a[pivot * n + c] = t; }
			double t = b[col]; b[col] = b[pivot]; b[pivot] = t;
		}
		for (int r = col + 1; r < n; r++)
		{
			double f = a[r * n + col] / a[col * n + col];
			for (int c = col; c < n; c++) a[r * n + c] -= f * a[col * n + c];
			b[r] -= f * b[col];
		}
	}
	for (int r = n - 1; r >= 0; r--)
	{
		double s = b[r];
		for (int c = r + 1; c < n; c++) s -= a[r * n + c] * x[c];
		x[r] = s / a[r * n + r];
	}
	return true;
}
