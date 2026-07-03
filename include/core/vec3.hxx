#pragma once
#include "core/point.hxx"

class Vec3 : public Point {
private:
	double len_ = 0.0;

	double computeLen() const {
		return std::sqrt(x * x + y * y + z * z);
	}

public:
	Vec3() : Point(), len_(0.0) {}

	Vec3(double x_, double y_, double z_)
		: Point(x_, y_, z_) {
		len_ = computeLen();
	}

	Vec3(const Point& start, const Point& end)
		: Point(end.x - start.x, end.y - start.y, end.z - start.z) {
		len_ = computeLen();
	}

	Vec3(Point* start, Point* end)
		: Vec3(*start, *end) {
	}

	Vec3(const Point& p)
		: Point(p) {
		len_ = computeLen();
	}

	double cachedLength() const noexcept {
		return len_;
	}

	double dot(const Vec3& rhs) const {
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	Vec3 cross(const Vec3& rhs) const {
		return Vec3(
			y * rhs.z - z * rhs.y,
			z * rhs.x - x * rhs.z,
			x * rhs.y - y * rhs.x
		);
	}

	double distanceTo(const Vec3& other) const {
		return (*this - other).length();
	}

	Vec3 normalize() const {
		if (len_ < 1e-12) return Vec3(0, 0, 0);
		return Vec3(x / len_, y / len_, z / len_);
	}

	Vec3& normalized() {
		if (len_ > 1e-12) {
			x /= len_;  y /= len_;  z /= len_;
			len_ = 1.0;
		}
		return *this;
	}

	Vec3 operator+(const Vec3& rhs) const {
		return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	Vec3 operator-(const Vec3& rhs) const {
		return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	Vec3 operator*(double scalar) const {
		return Vec3(x * scalar, y * scalar, z * scalar);
	}

	Vec3 operator/(double scalar) const {
		return Vec3(x / scalar, y / scalar, z / scalar);
	}

	Vec3 operator-() const {
		return Vec3(-x, -y, -z);
	}

	Vec3 operator+() const {
		return *this;
	}

	Vec3& operator+=(const Vec3& rhs) {
		x += rhs.x;  y += rhs.y;  z += rhs.z;
		len_ = computeLen();
		return *this;
	}

	Vec3& operator-=(const Vec3& rhs) {
		x -= rhs.x;  y -= rhs.y;  z -= rhs.z;
		len_ = computeLen();
		return *this;
	}

	Vec3& operator*=(double scalar) {
		x *= scalar;  y *= scalar;  z *= scalar;
		len_ = std::abs(scalar) * len_;
		return *this;
	}

	Vec3& operator/=(double scalar) {
		x /= scalar;  y /= scalar;  z /= scalar;
		len_ = len_ / std::abs(scalar);
		return *this;
	}
};

inline Vec3 operator*(double scalar, const Vec3& v) {
	return Vec3(scalar * v.x, scalar * v.y, scalar * v.z);
}
