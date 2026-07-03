#pragma once
#include "core/point.hxx"
#include "core/vec3.hxx"
#include "core/triangle.hxx"
#include <vector>
#include <cmath>
#include <algorithm>

namespace geo {

// ============================================================
// AABB
// ============================================================

struct AABB {
	double minX, minY, minZ;
	double maxX, maxY, maxZ;

	AABB()
		: minX(1e30), minY(1e30), minZ(1e30),
		  maxX(-1e30), maxY(-1e30), maxZ(-1e30) {}

	AABB(double minX_, double minY_, double minZ_,
	     double maxX_, double maxY_, double maxZ_)
		: minX(minX_), minY(minY_), minZ(minZ_),
		  maxX(maxX_), maxY(maxY_), maxZ(maxZ_) {}

	void expand(double x, double y, double z) {
		if (x < minX) minX = x; if (x > maxX) maxX = x;
		if (y < minY) minY = y; if (y > maxY) maxY = y;
		if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
	}

	void expand(const Point& p) { expand(p.x, p.y, p.z); }

	bool overlaps(const AABB& o, double eps = 0.0) const {
		return minX <= o.maxX + eps && maxX >= o.minX - eps
			&& minY <= o.maxY + eps && maxY >= o.minY - eps
			&& minZ <= o.maxZ + eps && maxZ >= o.minZ - eps;
	}

	Point center() const {
		return Point((minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5);
	}

	Point extents() const {
		return Point(maxX - minX, maxY - minY, maxZ - minZ);
	}
};

inline AABB computeTriangleAABB(const Triangle* tri) {
	AABB box;
	for (int i = 0; i < 3; ++i) {
		Vertex* v = tri->vertex(i);
		if (v) box.expand(v->x, v->y, v->z);
	}
	return box;
}

inline bool overlap(const AABB& a, const AABB& b, double eps = 0.0) {
	return a.overlaps(b, eps);
}

// ============================================================
// Geometric Predicates
// ============================================================

// orient3D: signed volume of tetrahedron (a,b,c,d)
// >0: d on positive side of abc plane (right-hand rule)
// <0: d on negative side
// =0: coplanar
inline double orient3D(const Point& a, const Point& b, const Point& c, const Point& d) {
	Vec3 ab(b.x - a.x, b.y - a.y, b.z - a.z);
	Vec3 ac(c.x - a.x, c.y - a.y, c.z - a.z);
	Vec3 ad(d.x - a.x, d.y - a.y, d.z - a.z);
	return ab.cross(ac).dot(ad);
}

// Point on segment test
inline bool pointOnSegment(const Point& p, const Point& a, const Point& b, double eps = 1e-8) {
	Vec3 ab(b.x - a.x, b.y - a.y, b.z - a.z);
	Vec3 ap(p.x - a.x, p.y - a.y, p.z - a.z);
	double len2 = ab.dot(ab);
	if (len2 < eps * eps) {
		return ap.cachedLength() < eps;
	}
	double t = ap.dot(ab) / len2;
	if (t < -eps || t > 1.0 + eps) return false;
	Vec3 proj = ab * t;
	double dist = Vec3(ap.x - proj.x, ap.y - proj.y, ap.z - proj.z).cachedLength();
	return dist < eps;
}

// Signed distance from point to triangle plane
inline double signedDistanceToPlane(const Point& p, const Triangle* tri) {
	if (!tri) return 0.0;
	Vertex* v0 = tri->vertex(0);
	if (!v0) return 0.0;
	const Vec3& n = tri->normal();
	return n.x * (p.x - v0->x) + n.y * (p.y - v0->y) + n.z * (p.z - v0->z);
}

// Project point onto triangle plane
inline Point projectToPlane(const Point& p, const Triangle* tri) {
	double d = signedDistanceToPlane(p, tri);
	const Vec3& n = tri->normal();
	return Point(p.x - d * n.x, p.y - d * n.y, p.z - d * n.z);
}

// Point in triangle (barycentric coordinates)
// First checks if point is on the triangle's plane
inline bool pointInTriangle(const Point& p, const Triangle* tri, double eps = 1e-8) {
	if (!tri) return false;
	Vertex* v0 = tri->vertex(0);
	Vertex* v1 = tri->vertex(1);
	Vertex* v2 = tri->vertex(2);
	if (!v0 || !v1 || !v2) return false;

	// Check distance to plane first
	double dist = signedDistanceToPlane(p, tri);
	if (std::abs(dist) > eps) return false;  // not on plane

	Vec3 e0(v1->x - v0->x, v1->y - v0->y, v1->z - v0->z);
	Vec3 e1(v2->x - v0->x, v2->y - v0->y, v2->z - v0->z);
	Vec3 ep(p.x - v0->x, p.y - v0->y, p.z - v0->z);

	double d00 = e0.dot(e0);
	double d01 = e0.dot(e1);
	double d11 = e1.dot(e1);
	double dp0 = ep.dot(e0);
	double dp1 = ep.dot(e1);

	double denom = d00 * d11 - d01 * d01;
	if (std::abs(denom) < eps * eps) return false;

	double invDenom = 1.0 / denom;
	double u = (d11 * dp0 - d01 * dp1) * invDenom;
	double v = (d00 * dp1 - d01 * dp0) * invDenom;

	return u >= -eps && v >= -eps && (u + v) <= 1.0 + eps;
}

// ============================================================
// Segment-Triangle Intersection (Moller-Trumbore)
// ============================================================

inline bool segmentTriangleIntersection(
	const Point& p0, const Point& p1,
	const Triangle* tri,
	Point& hit, double eps = 1e-8)
{
	if (!tri) return false;
	Vertex* v0 = tri->vertex(0);
	Vertex* v1 = tri->vertex(1);
	Vertex* v2 = tri->vertex(2);
	if (!v0 || !v1 || !v2) return false;

	Vec3 dir(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
	double segLen = dir.cachedLength();
	if (segLen < eps) return false;
	dir = Vec3(dir.x / segLen, dir.y / segLen, dir.z / segLen);

	Vec3 e1(v1->x - v0->x, v1->y - v0->y, v1->z - v0->z);
	Vec3 e2(v2->x - v0->x, v2->y - v0->y, v2->z - v0->z);
	Vec3 pvec = dir.cross(e2);
	double det = e1.dot(pvec);

	if (std::abs(det) < 1e-12) return false;

	double invDet = 1.0 / det;
	Vec3 tvec(p0.x - v0->x, p0.y - v0->y, p0.z - v0->z);
	double u = tvec.dot(pvec) * invDet;
	if (u < -eps || u > 1.0 + eps) return false;

	Vec3 qvec = tvec.cross(e1);
	double v = dir.dot(qvec) * invDet;
	if (v < -eps || u + v > 1.0 + eps) return false;

	double t = e2.dot(qvec) * invDet;
	if (t < -eps || t > segLen + eps) return false;

	hit = Point(p0.x + dir.x * t, p0.y + dir.y * t, p0.z + dir.z * t);
	return true;
}

// ============================================================
// Triangle-Triangle Intersection
// ============================================================

enum class TriTriIntersectionType {
	None,
	Point,
	Segment,
	CoplanarOverlap
};

struct TriTriIntersectionResult {
	TriTriIntersectionType type;
	std::vector<Point> points;
};

namespace detail {

	inline bool edgeTriIntersect(
		const Point& p0, const Point& p1,
		const Point& v0, const Point& v1, const Point& v2,
		Point& hit, double eps)
	{
		Vec3 dir(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
		double segLen = dir.cachedLength();
		if (segLen < eps) return false;
		dir = Vec3(dir.x / segLen, dir.y / segLen, dir.z / segLen);

		Vec3 e1(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
		Vec3 e2(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
		Vec3 pvec = dir.cross(e2);
		double det = e1.dot(pvec);
		if (std::abs(det) < 1e-12) return false;

		double invDet = 1.0 / det;
		Vec3 tvec(p0.x - v0.x, p0.y - v0.y, p0.z - v0.z);
		double u = tvec.dot(pvec) * invDet;
		if (u < -eps || u > 1.0 + eps) return false;

		Vec3 qvec = tvec.cross(e1);
		double v = dir.dot(qvec) * invDet;
		if (v < -eps || u + v > 1.0 + eps) return false;

		double t = e2.dot(qvec) * invDet;
		if (t < eps || t > segLen - eps) return false;

		hit = Point(p0.x + dir.x * t, p0.y + dir.y * t, p0.z + dir.z * t);
		return true;
	}

	inline bool pointInTriStrict(const Point& p,
		const Point& v0, const Point& v1, const Point& v2, double eps)
	{
		Vec3 n = Vec3(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z)
		       .cross(Vec3(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z));
		double nLen = n.cachedLength();
		if (nLen < eps) return false;

		Vec3 a = Vec3(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z)
		       .cross(Vec3(p.x - v0.x, p.y - v0.y, p.z - v0.z));
		Vec3 b = Vec3(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z)
		       .cross(Vec3(p.x - v1.x, p.y - v1.y, p.z - v1.z));
		Vec3 c = Vec3(v0.x - v2.x, v0.y - v2.y, v0.z - v2.z)
		       .cross(Vec3(p.x - v2.x, p.y - v2.y, p.z - v2.z));

		double margin = nLen * nLen * eps;
		return n.dot(a) > margin && n.dot(b) > margin && n.dot(c) > margin;
	}

} // namespace detail

inline TriTriIntersectionResult intersectTriangles(
	const Triangle* a, const Triangle* b, double eps = 1e-8)
{
	TriTriIntersectionResult result;
	result.type = TriTriIntersectionType::None;

	if (!a || !b) return result;

	Point a0(a->vertex(0)->x, a->vertex(0)->y, a->vertex(0)->z);
	Point a1(a->vertex(1)->x, a->vertex(1)->y, a->vertex(1)->z);
	Point a2(a->vertex(2)->x, a->vertex(2)->y, a->vertex(2)->z);
	Point b0(b->vertex(0)->x, b->vertex(0)->y, b->vertex(0)->z);
	Point b1(b->vertex(1)->x, b->vertex(1)->y, b->vertex(1)->z);
	Point b2(b->vertex(2)->x, b->vertex(2)->y, b->vertex(2)->z);

	Point hit;

	// Edges of A against triangle B
	Point edgesA[3][2] = {{a0,a1},{a1,a2},{a2,a0}};
	for (int i = 0; i < 3; ++i) {
		if (detail::edgeTriIntersect(edgesA[i][0], edgesA[i][1], b0, b1, b2, hit, eps)) {
			result.points.push_back(hit);
		}
	}

	// Edges of B against triangle A
	Point edgesB[3][2] = {{b0,b1},{b1,b2},{b2,b0}};
	for (int i = 0; i < 3; ++i) {
		if (detail::edgeTriIntersect(edgesB[i][0], edgesB[i][1], a0, a1, a2, hit, eps)) {
			result.points.push_back(hit);
		}
	}

	// Coplanar containment
	if (detail::pointInTriStrict(a0, b0, b1, b2, eps)) result.points.push_back(a0);
	if (detail::pointInTriStrict(a1, b0, b1, b2, eps)) result.points.push_back(a1);
	if (detail::pointInTriStrict(a2, b0, b1, b2, eps)) result.points.push_back(a2);
	if (detail::pointInTriStrict(b0, a0, a1, a2, eps)) result.points.push_back(b0);
	if (detail::pointInTriStrict(b1, a0, a1, a2, eps)) result.points.push_back(b1);
	if (detail::pointInTriStrict(b2, a0, a1, a2, eps)) result.points.push_back(b2);

	// Classify
	if (result.points.empty()) {
		result.type = TriTriIntersectionType::None;
	} else if (result.points.size() == 1) {
		result.type = TriTriIntersectionType::Point;
	} else {
		double d = orient3D(a0, a1, a2, b0);
		if (std::abs(d) < eps) {
			result.type = TriTriIntersectionType::CoplanarOverlap;
		} else {
			result.type = TriTriIntersectionType::Segment;
		}
	}

	return result;
}

// ============================================================
// Point-in-Mesh Classification
// ============================================================

enum class PointClass {
	Outside,
	Inside,
	OnBoundary
};

// classifyPointInMesh is declared in mesh.hxx (needs Mesh fully defined)

} // namespace geo
