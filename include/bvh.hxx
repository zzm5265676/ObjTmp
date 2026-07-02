#pragma once
#include "geometry_utils.hxx"
#include "mesh.hxx"
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

namespace geo {

struct BVHNode {
	AABB bounds;
	int left = -1;
	int right = -1;
	int triStart = -1;
	int triCount = 0;

	bool isLeaf() const { return left < 0; }
};

class BVH {
public:
	BVH(const Mesh& mesh) : mesh_(mesh) {
		int n = static_cast<int>(mesh.triangleCount());
		triIndices_.resize(n);
		for (int i = 0; i < n; ++i) triIndices_[i] = i;

		if (n > 0) {
			nodes_.reserve(2 * n);
			buildNode(0, n);
		}
	}

	// Ray query: returns closest hit
	bool raycast(const Point& origin, const Vec3& direction,
	             double maxDist, Point& hit, int& triIdx) const {
		triIdx = -1;
		double closestT = maxDist;
		bool found = false;

		if (!nodes_.empty()) {
			raycastNode(0, origin, direction, maxDist, closestT, hit, triIdx, found);
		}
		return found;
	}

	// Closest point query
	Point closestPoint(const Point& p, int& triIdx) const {
		triIdx = -1;
		double bestDist2 = (std::numeric_limits<double>::max)();
		Point bestPoint;

		if (!nodes_.empty()) {
			closestPointNode(0, p, bestDist2, bestPoint, triIdx);
		}
		return bestPoint;
	}

	// AABB overlap query
	std::vector<int> queryAABB(const AABB& box) const {
		std::vector<int> result;
		if (!nodes_.empty()) {
			queryAABBNode(0, box, result);
		}
		return result;
	}

	// Sphere overlap query
	std::vector<int> querySphere(const Point& center, double radius) const {
		AABB box(center.x - radius, center.y - radius, center.z - radius,
		         center.x + radius, center.y + radius, center.z + radius);
		auto candidates = queryAABB(box);

		// Filter by actual sphere intersection
		std::vector<int> result;
		for (int idx : candidates) {
			const Triangle* tri = mesh_.triangle(idx);
			if (!tri) continue;
			// Check if sphere intersects triangle AABB
			AABB triBox = computeTriangleAABB(tri);
			if (boxOverlapsSphere(triBox, center, radius)) {
				result.push_back(idx);
			}
		}
		return result;
	}

private:
	std::vector<BVHNode> nodes_;
	std::vector<int> triIndices_;
	const Mesh& mesh_;

	int buildNode(int start, int count) {
		int nodeIdx = static_cast<int>(nodes_.size());
		nodes_.push_back(BVHNode());

		// Compute bounds
		AABB bounds;
		for (int i = start; i < start + count; ++i) {
			const Triangle* tri = mesh_.triangle(triIndices_[i]);
			if (tri) {
				for (int j = 0; j < 3; ++j) {
					Vertex* v = tri->vertex(j);
					if (v) bounds.expand(v->x, v->y, v->z);
				}
			}
		}
		nodes_[nodeIdx].bounds = bounds;

		// Leaf node
		if (count <= 4) {
			nodes_[nodeIdx].triStart = start;
			nodes_[nodeIdx].triCount = count;
			return nodeIdx;
		}

		// Choose split axis (longest extent)
		double dx = bounds.maxX - bounds.minX;
		double dy = bounds.maxY - bounds.minY;
		double dz = bounds.maxZ - bounds.minZ;
		int axis = (dx > dy && dx > dz) ? 0 : (dy > dz ? 1 : 2);

		// Sort by centroid
		std::sort(triIndices_.begin() + start, triIndices_.begin() + start + count,
			[&](int a, int b) {
				const Triangle* ta = mesh_.triangle(a);
				const Triangle* tb = mesh_.triangle(b);
				if (!ta || !tb) return false;
				double ca = centroid(ta, axis);
				double cb = centroid(tb, axis);
				return ca < cb;
			});

		// Split at midpoint
		int mid = count / 2;
		nodes_[nodeIdx].left = buildNode(start, mid);
		nodes_[nodeIdx].right = buildNode(start + mid, count - mid);

		return nodeIdx;
	}

	static double centroid(const Triangle* tri, int axis) {
		double sum = 0;
		for (int i = 0; i < 3; ++i) {
			Vertex* v = tri->vertex(i);
			if (v) {
				sum += (axis == 0) ? v->x : (axis == 1) ? v->y : v->z;
			}
		}
		return sum / 3.0;
	}

	void raycastNode(int nodeIdx, const Point& origin, const Vec3& dir,
	                 double maxDist, double& closestT, Point& hit, int& triIdx, bool& found) const {
		const BVHNode& node = nodes_[nodeIdx];

		// Check ray-AABB intersection
		if (!rayAABBIntersect(origin, dir, node.bounds, maxDist)) return;

		if (node.isLeaf()) {
			for (int i = node.triStart; i < node.triStart + node.triCount; ++i) {
				const Triangle* tri = mesh_.triangle(triIndices_[i]);
				if (!tri) continue;

				Point p0(origin.x, origin.y, origin.z);
				Point p1(origin.x + dir.x * maxDist, origin.y + dir.y * maxDist, origin.z + dir.z * maxDist);
				Point triHit;
				if (segmentTriangleIntersection(p0, p1, tri, triHit, 1e-8)) {
					double t = Vec3(triHit.x - origin.x, triHit.y - origin.y, triHit.z - origin.z).cachedLength();
					if (t < closestT) {
						closestT = t;
						hit = triHit;
						triIdx = triIndices_[i];
						found = true;
					}
				}
			}
		} else {
			raycastNode(node.left, origin, dir, maxDist, closestT, hit, triIdx, found);
			raycastNode(node.right, origin, dir, maxDist, closestT, hit, triIdx, found);
		}
	}

	void closestPointNode(int nodeIdx, const Point& p, double& bestDist2, Point& bestPoint, int& triIdx) const {
		const BVHNode& node = nodes_[nodeIdx];

		// Check distance to AABB
		double dist2 = pointAABBDist2(p, node.bounds);
		if (dist2 > bestDist2) return;

		if (node.isLeaf()) {
			for (int i = node.triStart; i < node.triStart + node.triCount; ++i) {
				const Triangle* tri = mesh_.triangle(triIndices_[i]);
				if (!tri) continue;

				// Project point onto triangle plane
				Point proj = projectToPlane(p, tri);
				if (pointInTriangle(proj, tri, 1e-6)) {
					double d2 = Vec3(proj.x - p.x, proj.y - p.y, proj.z - p.z).dot(Vec3(proj.x - p.x, proj.y - p.y, proj.z - p.z));
					if (d2 < bestDist2) {
						bestDist2 = d2;
						bestPoint = proj;
						triIdx = triIndices_[i];
					}
				} else {
					// Check distance to edges
					for (int j = 0; j < 3; ++j) {
						Vertex* v0 = tri->vertex(j);
						Vertex* v1 = tri->vertex((j + 1) % 3);
						if (!v0 || !v1) continue;
						Point a(v0->x, v0->y, v0->z);
						Point b(v1->x, v1->y, v1->z);
						// Closest point on segment
						Vec3 ab(b.x - a.x, b.y - a.y, b.z - a.z);
						Vec3 ap(p.x - a.x, p.y - a.y, p.z - a.z);
						double t = ap.dot(ab) / ab.dot(ab);
						t = (std::max)(0.0, (std::min)(1.0, t));
						Point cand(a.x + t * ab.x, a.y + t * ab.y, a.z + t * ab.z);
						double d2 = Vec3(cand.x - p.x, cand.y - p.y, cand.z - p.z).dot(Vec3(cand.x - p.x, cand.y - p.y, cand.z - p.z));
						if (d2 < bestDist2) {
							bestDist2 = d2;
							bestPoint = cand;
							triIdx = triIndices_[i];
						}
					}
				}
			}
		} else {
			// Visit closer child first
			double dl = pointAABBDist2(p, nodes_[node.left].bounds);
			double dr = pointAABBDist2(p, nodes_[node.right].bounds);
			if (dl < dr) {
				closestPointNode(node.left, p, bestDist2, bestPoint, triIdx);
				closestPointNode(node.right, p, bestDist2, bestPoint, triIdx);
			} else {
				closestPointNode(node.right, p, bestDist2, bestPoint, triIdx);
				closestPointNode(node.left, p, bestDist2, bestPoint, triIdx);
			}
		}
	}

	void queryAABBNode(int nodeIdx, const AABB& box, std::vector<int>& result) const {
		const BVHNode& node = nodes_[nodeIdx];
		if (!node.bounds.overlaps(box)) return;

		if (node.isLeaf()) {
			for (int i = node.triStart; i < node.triStart + node.triCount; ++i) {
				result.push_back(triIndices_[i]);
			}
		} else {
			queryAABBNode(node.left, box, result);
			queryAABBNode(node.right, box, result);
		}
	}

	static bool rayAABBIntersect(const Point& origin, const Vec3& dir, const AABB& box, double maxDist) {
		double tmin = 0, tmax = maxDist;
		for (int i = 0; i < 3; ++i) {
			double o = (i == 0) ? origin.x : (i == 1) ? origin.y : origin.z;
			double d = (i == 0) ? dir.x : (i == 1) ? dir.y : dir.z;
			double bmin = (i == 0) ? box.minX : (i == 1) ? box.minY : box.minZ;
			double bmax = (i == 0) ? box.maxX : (i == 1) ? box.maxY : box.maxZ;

			if ((std::abs)(d) < 1e-15) {
				if (o < bmin || o > bmax) return false;
			} else {
				double invD = 1.0 / d;
				double t0 = (bmin - o) * invD;
				double t1 = (bmax - o) * invD;
				if (t0 > t1) (std::swap)(t0, t1);
				tmin = (std::max)(tmin, t0);
				tmax = (std::min)(tmax, t1);
				if (tmin > tmax) return false;
			}
		}
		return true;
	}

	static double pointAABBDist2(const Point& p, const AABB& box) {
		double dx = (std::max)(0.0, (std::max)(box.minX - p.x, p.x - box.maxX));
		double dy = (std::max)(0.0, (std::max)(box.minY - p.y, p.y - box.maxY));
		double dz = (std::max)(0.0, (std::max)(box.minZ - p.z, p.z - box.maxZ));
		return dx * dx + dy * dy + dz * dz;
	}

	static bool boxOverlapsSphere(const AABB& box, const Point& center, double radius) {
		return pointAABBDist2(center, box) <= radius * radius;
	}
};

} // namespace geo
