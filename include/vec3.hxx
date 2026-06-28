#pragma once
#include "point.hxx"

class Vec3 :public Point {
private:
public:
	double len = 0.0;

	Vec3() : Point(), len(0.0) {}

	Vec3(double x_, double y_, double z_)
		: Point(x_, y_, z_), len(length()) {
	}

	Vec3(const Point& start, const Point& end)
		: Point(end.x - start.x, end.y - start.y, end.z - start.z) {
		len = length();
	}

	Vec3(Point* start, Point* end)
		: Vec3(*start, *end) {
	}

	Vec3& operator=(const Point& p) {
		x = p.x;
		y = p.y;
		z = p.z;
		len = p.length();
		return *this;
	}
};
