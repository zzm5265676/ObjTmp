#pragma once
#include "point.hxx"

class Vec3 :public Point {
private:
public:
	double len;
	Vec3() {}
	Vec3(double x_, double y_, double z_) {
		x = x_;
		y = y_;
		z = z_;
		len = length();
	}
	Vec3(const Point& start, const Point& end) {
		x = end.x - start.x;
		y = end.y - start.y;
		z = end.z - start.z;
		len = start.distanceTo(end);
	}
	Vec3(Point* start, Point* end) :Vec3((*end).x - (*start).x, (*end).y - (*start).y, (*end).z - (*start).z) {
	}

};
