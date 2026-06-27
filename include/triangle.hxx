#pragma once
#include "point.hxx"
#include "vec3.hxx"
#include <vector>
class Triangle {
	Point* p1, * p2, * p3;
	Vec3 faceNormal;
	double area;
	Point center;
public:
	Triangle(Point* a, Point* b, Point* c) :p1(a), p2(b), p3(c) {
		Vec3 v1(p1, p2);
		Vec3 v2(p1, p3);
		area = v1.cross(v2).length() / 2.0;
		center = (*p1 + *p2 + *p3) / 3.0;
		faceNormal = dynamic_cast<Vec3>(v1.cross(v2).normalize());
	}
	Point getCenter() const {
		return center;
	}
	Vec3 getNormal() const {
		return faceNormal;
	}
	std::vector<Point*> getPoints() const {
		return { p1, p2, p3 };
	}

	friend std::ostream& operator<<(std::ostream& os, const Triangle& t) {
		os << "Triangle: " << t.p1 << " " << t.p2 << " " << t.p3 << "\n";
		os << "Center: " << t.center << "\n";
		os << "Area: " << t.area << "\n";
		os << "Normal: " << t.faceNormal;
		return os;
	}
};