#pragma once
#include "core/vec3.hxx"
#include <cmath>

class Mat4;

class Quat {
public:
	double w, x, y, z;

	Quat() : w(1.0), x(0.0), y(0.0), z(0.0) {}

	Quat(double w_, double x_, double y_, double z_)
		: w(w_), x(x_), y(y_), z(z_) {
	}

	static Quat identity() {
		return Quat(1, 0, 0, 0);
	}

	// From axis-angle (axis must be unit vector, angle in radians)
	static Quat fromAxisAngle(const Vec3& axis, double radians) {
		double half = radians * 0.5;
		double s = std::sin(half);
		return Quat(std::cos(half), axis.x * s, axis.y * s, axis.z * s);
	}

	// From Euler angles (XYZ intrinsic order, radians)
	// pitch = X, yaw = Y, roll = Z
	static Quat fromEuler(double pitch, double yaw, double roll) {
		double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
		double cy = std::cos(yaw * 0.5),   sy = std::sin(yaw * 0.5);
		double cr = std::cos(roll * 0.5),  sr = std::sin(roll * 0.5);
		return Quat(
			cr * cp * cy + sr * sp * sy,
			sr * cp * cy - cr * sp * sy,
			cr * sp * cy + sr * cp * sy,
			cr * cp * sy - sr * sp * cy
		);
	}

	static Quat fromMat4(const Mat4& m);

	// Hamilton product
	Quat operator*(const Quat& rhs) const {
		return Quat(
			w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
			w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
			w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
			w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
		);
	}

	Quat& operator*=(const Quat& rhs) {
		*this = *this * rhs;
		return *this;
	}

	Quat conjugate() const {
		return Quat(w, -x, -y, -z);
	}

	double length() const {
		return std::sqrt(w * w + x * x + y * y + z * z);
	}

	double dot(const Quat& rhs) const {
		return w * rhs.w + x * rhs.x + y * rhs.y + z * rhs.z;
	}

	Quat normalize() const {
		double len = length();
		if (len < 1e-15) return identity();
		return Quat(w / len, x / len, y / len, z / len);
	}

	Quat inverse() const {
		double len2 = w * w + x * x + y * y + z * z;
		if (len2 < 1e-15) return identity();
		return Quat(w / len2, -x / len2, -y / len2, -z / len2);
	}

	// Rotate a point: q * p * q^{-1}
	Vec3 rotatePoint(const Vec3& p) const {
		Vec3 qv(x, y, z);
		Vec3 uv = qv.cross(p);
		Vec3 uuv = qv.cross(uv);
		return p + (uv * w + uuv) * 2.0;
	}

	Mat4 toMat4() const;

	void toAxisAngle(Vec3& axis, double& radians) const {
		Quat q = normalize();
		if (q.w < 0) q = Quat(-q.w, -q.x, -q.y, -q.z);
		double halfAngle = std::acos((std::min)(q.w, 1.0));
		double s = std::sin(halfAngle);
		radians = halfAngle * 2.0;
		if (std::abs(s) < 1e-10) {
			axis = Vec3(1, 0, 0);
		} else {
			axis = Vec3(q.x / s, q.y / s, q.z / s);
		}
	}

	static Quat slerp(const Quat& a, const Quat& b, double t) {
		double dotVal = a.dot(b);
		Quat end = b;
		if (dotVal < 0) {
			end = Quat(-b.w, -b.x, -b.y, -b.z);
			dotVal = -dotVal;
		}
		if (dotVal > 0.9995) {
			return Quat(
				a.w + t * (end.w - a.w),
				a.x + t * (end.x - a.x),
				a.y + t * (end.y - a.y),
				a.z + t * (end.z - a.z)
			).normalize();
		}
		double theta0 = std::acos(dotVal);
		double theta = theta0 * t;
		double sinTheta = std::sin(theta);
		double sinTheta0 = std::sin(theta0);
		double s0 = std::cos(theta) - dotVal * sinTheta / sinTheta0;
		double s1 = sinTheta / sinTheta0;
		return Quat(
			s0 * a.w + s1 * end.w,
			s0 * a.x + s1 * end.x,
			s0 * a.y + s1 * end.y,
			s0 * a.z + s1 * end.z
		);
	}
};

#include "math/mat4.hxx"

inline Mat4 Quat::toMat4() const {
	double xx = x * x, yy = y * y, zz = z * z;
	double xy = x * y, xz = x * z, yz = y * z;
	double wx = w * x, wy = w * y, wz = w * z;

	Mat4 mat;
	mat.at(0, 0) = 1.0 - 2.0 * (yy + zz);
	mat.at(0, 1) = 2.0 * (xy - wz);
	mat.at(0, 2) = 2.0 * (xz + wy);
	mat.at(1, 0) = 2.0 * (xy + wz);
	mat.at(1, 1) = 1.0 - 2.0 * (xx + zz);
	mat.at(1, 2) = 2.0 * (yz - wx);
	mat.at(2, 0) = 2.0 * (xz - wy);
	mat.at(2, 1) = 2.0 * (yz + wx);
	mat.at(2, 2) = 1.0 - 2.0 * (xx + yy);
	return mat;
}

inline Quat Quat::fromMat4(const Mat4& m) {
	double trace = m.at(0, 0) + m.at(1, 1) + m.at(2, 2);
	if (trace > 0) {
		double s = 0.5 / std::sqrt(trace + 1.0);
		return Quat(0.25 / s,
			(m.at(2, 1) - m.at(1, 2)) * s,
			(m.at(0, 2) - m.at(2, 0)) * s,
			(m.at(1, 0) - m.at(0, 1)) * s);
	}
	if (m.at(0, 0) > m.at(1, 1) && m.at(0, 0) > m.at(2, 2)) {
		double s = 2.0 * std::sqrt(1.0 + m.at(0, 0) - m.at(1, 1) - m.at(2, 2));
		return Quat((m.at(2, 1) - m.at(1, 2)) / s,
			0.25 * s,
			(m.at(0, 1) + m.at(1, 0)) / s,
			(m.at(0, 2) + m.at(2, 0)) / s);
	}
	if (m.at(1, 1) > m.at(2, 2)) {
		double s = 2.0 * std::sqrt(1.0 + m.at(1, 1) - m.at(0, 0) - m.at(2, 2));
		return Quat((m.at(0, 2) - m.at(2, 0)) / s,
			(m.at(0, 1) + m.at(1, 0)) / s,
			0.25 * s,
			(m.at(1, 2) + m.at(2, 1)) / s);
	}
	double s = 2.0 * std::sqrt(1.0 + m.at(2, 2) - m.at(0, 0) - m.at(1, 1));
	return Quat((m.at(1, 0) - m.at(0, 1)) / s,
		(m.at(0, 2) + m.at(2, 0)) / s,
		(m.at(1, 2) + m.at(2, 1)) / s,
		0.25 * s);
}
