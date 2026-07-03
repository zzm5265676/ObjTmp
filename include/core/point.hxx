#pragma once 
#include <cmath>
#include <iostream>
#include <stdexcept>
class Point {
public:
	double x, y, z;

	// =====================  =====================

	// 
	Point() : x(0), y(0), z(0) {}

	// 
	Point(double x, double y, double z) : x(x), y(y), z(z) {}

	// 
	Point(const Point& other) : x(other.x), y(other.y), z(other.z) {}

	// 
	Point& operator=(const Point& other) {
		if (this != &other) {
			x = other.x;
			y = other.y;
			z = other.z;
		}
		return *this;
	}

	// =====================  =====================

	// Point + Point
	Point operator+(const Point& rhs) const {
		return Point(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	// Point - Point
	Point operator-(const Point& rhs) const {
		return Point(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	// Point * 
	Point operator*(double scalar) const {
		return Point(x * scalar, y * scalar, z * scalar);
	}

	//  * Point Point
	friend Point operator*(double scalar, const Point& p) {
		return Point(scalar * p.x, scalar * p.y, scalar * p.z);
	}

	// Point / 
	Point operator/(double scalar) const {
		return Point(x / scalar, y / scalar, z / scalar);
	}

	//  -p
	Point operator-() const {
		return Point(-x, -y, -z);
	}

	//  +p
	Point operator+() const {
		return *this;
	}

	// =====================  =====================

	Point& operator+=(const Point& rhs) {
		x += rhs.x;  y += rhs.y;  z += rhs.z;
		return *this;
	}

	Point& operator-=(const Point& rhs) {
		x -= rhs.x;  y -= rhs.y;  z -= rhs.z;
		return *this;
	}

	Point& operator*=(double scalar) {
		x *= scalar;  y *= scalar;  z *= scalar;
		return *this;
	}

	Point& operator/=(double scalar) {
		x /= scalar;  y /= scalar;  z /= scalar;
		return *this;
	}

	// =====================  =====================

	bool operator==(const Point& rhs) const {
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}

	bool operator!=(const Point& rhs) const {
		return !(*this == rhs);
	}

	// =====================  =====================

	double& operator[](int index) {
		switch (index) {
		case 0:  return x;
		case 1:  return y;
		case 2:  return z;
		default: throw std::out_of_range("Point index out of range [0,2]");
		}
	}

	const double& operator[](int index) const {
		switch (index) {
		case 0:  return x;
		case 1:  return y;
		case 2:  return z;
		default: throw std::out_of_range("Point index out of range [0,2]");
		}
	}

	// =====================  =====================

	friend std::ostream& operator<<(std::ostream& os, const Point& p) {
		os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
		return os;
	}

	friend std::istream& operator>>(std::istream& is, Point& p) {
		is >> p.x >> p.y >> p.z;
		return is;
	}

	// =====================  =====================

	// 
	double length() const {
		return std::sqrt(x * x + y * y + z * z);
	}

	// 
	double lengthSquared() const {
		return x * x + y * y + z * z;
	}

	// 
	double distanceTo(const Point& other) const {
		return (*this - other).length();
	}

	// 
	double dot(const Point& rhs) const {
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	// 
	Point cross(const Point& rhs) const {
		return Point(
			y * rhs.z - z * rhs.y,
			z * rhs.x - x * rhs.z,
			x * rhs.y - y * rhs.x
		);
	}

	// 
	Point normalize() const {
		double len = length();
		if (len < 1e-12) return Point(0, 0, 0);
		return *this / len;
	}

	// 
	Point& normalized() {
		double len = length();
		if (len > 1e-12) {
			x /= len;  y /= len;  z /= len;
		}
		return *this;
	}
};
