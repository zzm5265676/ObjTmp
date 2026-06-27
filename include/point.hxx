#pragma once 
#include <cmath>
#include <iostream>
#ifndef POINT_H
#define POINT_H

#include <iostream>
#include <cmath>

class Point {
public:
	double x, y, z;

	// ===================== 构造 =====================

	// 默认构造
	Point() : x(0), y(0), z(0) {}

	// 带参构造
	Point(double x, double y, double z) : x(x), y(y), z(z) {}

	// 拷贝构造（编译器会自动生成，显式写出更清晰）
	Point(const Point& other) : x(other.x), y(other.y), z(other.z) {}

	// 赋值运算符（编译器会自动生成，显式写出更清晰）
	Point& operator=(const Point& other) {
		if (this != &other) {
			x = other.x;
			y = other.y;
			z = other.z;
		}
		return *this;
	}

	// ===================== 算术运算符 =====================

	// Point + Point
	Point operator+(const Point& rhs) const {
		return Point(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	// Point - Point
	Point operator-(const Point& rhs) const {
		return Point(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	// Point * 标量
	Point operator*(double scalar) const {
		return Point(x * scalar, y * scalar, z * scalar);
	}

	// 标量 * Point（友元，因为左操作数不是 Point）
	friend Point operator*(double scalar, const Point& p) {
		return Point(scalar * p.x, scalar * p.y, scalar * p.z);
	}

	// Point / 标量
	Point operator/(double scalar) const {
		return Point(x / scalar, y / scalar, z / scalar);
	}

	// 取负 -p
	Point operator-() const {
		return Point(-x, -y, -z);
	}

	// 取正 +p
	Point operator+() const {
		return *this;
	}

	// ===================== 复合赋值运算符 =====================

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

	// ===================== 比较运算符 =====================

	bool operator==(const Point& rhs) const {
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}

	bool operator!=(const Point& rhs) const {
		return !(*this == rhs);
	}

	// ===================== 下标运算符 =====================

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

	// ===================== 流运算符 =====================

	friend std::ostream& operator<<(std::ostream& os, const Point& p) {
		os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
		return os;
	}

	friend std::istream& operator>>(std::istream& is, Point& p) {
		is >> p.x >> p.y >> p.z;
		return is;
	}

	// ===================== 常用方法 =====================

	// 模长（到原点的距离）
	double length() const {
		return std::sqrt(x * x + y * y + z * z);
	}

	// 模长的平方（避免开方，性能更好）
	double lengthSquared() const {
		return x * x + y * y + z * z;
	}

	// 两点间距离
	double distanceTo(const Point& other) const {
		return (*this - other).length();
	}

	// 点乘
	double dot(const Point& rhs) const {
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	// 叉乘
	Point cross(const Point& rhs) const {
		return Point(
			y * rhs.z - z * rhs.y,
			z * rhs.x - x * rhs.z,
			x * rhs.y - y * rhs.x
		);
	}

	// 归一化（返回单位向量，不修改自身）
	Point normalize() const {
		double len = length();
		if (len == 0) return Point(0, 0, 0);
		return *this / len;
	}

	// 就地归一化（修改自身）
	Point& normalized() {
		double len = length();
		if (len != 0) {
			x /= len;  y /= len;  z /= len;
		}
		return *this;
	}
};

#endif // POINT_H