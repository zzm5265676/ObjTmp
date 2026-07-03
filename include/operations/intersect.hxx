#pragma once
#include "mesh/mesh.hxx"
#include "geometry/bvh.hxx"
#include "geometry/geometry_utils.hxx"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <functional>

namespace mesh_intersect {

struct IntersectionSegment {
	Point start, end;
	int triA, triB;
};

struct IntersectionCurve {
	std::vector<IntersectionSegment> segments;
	bool isClosed = false;
};

struct IntersectionResult {
	std::vector<IntersectionCurve> curves;
	bool hasIntersection = false;
};

// Find which edge of triangle (v0,v1,v2) a point lies on
// Returns edge index (0,1,2) or -1
// edge 0: v0->v1, edge 1: v1->v2, edge 2: v2->v0
inline int findEdge(const Point& p, const Point& v0, const Point& v1, const Point& v2, double eps) {
	if (geo::pointOnSegment(p, v0, v1, eps)) return 0;
	if (geo::pointOnSegment(p, v1, v2, eps)) return 1;
	if (geo::pointOnSegment(p, v2, v0, eps)) return 2;
	return -1;
}

// Split a triangle at two points on its edges
// Returns 3 new triangles
// The two points must be on DIFFERENT edges
// triVerts: [v0, v1, v2]  original vertex indices
// edge0, p0Idx: first point is on this edge, p0Idx is its vertex index
// edge1, p1Idx: second point is on this edge, p1Idx is its vertex index
//
// Edge definitions: 0 = v0->v1, 1 = v1->v2, 2 = v2->v0
//
// When two edges share a vertex, the split creates:
// - A small triangle at the shared vertex
// - A quad that's split into 2 triangles
inline std::vector<std::array<int, 3>> splitTriangle(
	const std::array<int, 3>& triVerts,
	int edge0, int p0Idx,
	int edge1, int p1Idx)
{
	std::vector<std::array<int, 3>> result;
	int v0 = triVerts[0], v1 = triVerts[1], v2 = triVerts[2];

	// Normalize: ensure edge0 < edge1 for case matching
	if (edge0 > edge1) {
		std::swap(edge0, edge1);
		std::swap(p0Idx, p1Idx);
	}

	// Case 1: edges 0 and 1 (share v1)
	// Triangle is split by segment p0-p1 into:
	// (v0, p0, p1), (p0, v1, p1), (p0, p1, v2)
	if (edge0 == 0 && edge1 == 1) {
		result.push_back({v0, p0Idx, p1Idx});
		result.push_back({p0Idx, v1, p1Idx});
		result.push_back({p0Idx, p1Idx, v2});
		return result;
	}

	// Case 2: edges 1 and 2 (share v2)
	// (v0, p0, p1), (p0, v1, p1), (p0, p1, v2)  same pattern
	if (edge0 == 1 && edge1 == 2) {
		result.push_back({v1, p0Idx, p1Idx});
		result.push_back({p0Idx, v2, p1Idx});
		result.push_back({p0Idx, p1Idx, v0});
		return result;
	}

	// Case 3: edges 0 and 2 (share v0)
	if (edge0 == 0 && edge1 == 2) {
		result.push_back({v0, p1Idx, p0Idx});
		result.push_back({p1Idx, v2, p0Idx});
		result.push_back({p1Idx, p0Idx, v1});
		return result;
	}

	// Same edge or invalid  return original
	result.push_back(triVerts);
	return result;
}

// Insert a point on an edge of the mesh, splitting the edge
// Returns the vertex index of the point (existing or new)
inline int insertPointOnEdge(Mesh& mesh, int v0Idx, int v1Idx, const Point& p, double eps) {
	const Vertex* v0 = mesh.findByIndex(v0Idx);
	const Vertex* v1 = mesh.findByIndex(v1Idx);
	if (!v0 || !v1) return -1;

	double d0 = Vec3(p.x - v0->x, p.y - v0->y, p.z - v0->z).cachedLength();
	double d1 = Vec3(p.x - v1->x, p.y - v1->y, p.z - v1->z).cachedLength();

	if (d0 < eps) return v0Idx;
	if (d1 < eps) return v1Idx;

	return mesh.addVertex(p.x, p.y, p.z)->index;
}

// Get vertex indices of a triangle's edge
inline std::pair<int, int> getEdge(const std::array<int, 3>& tri, int edgeIdx) {
	switch (edgeIdx) {
	case 0: return {tri[0], tri[1]};
	case 1: return {tri[1], tri[2]};
	case 2: return {tri[2], tri[0]};
	default: return {-1, -1};
	}
}

// Subdivide a mesh at intersection points with another mesh
// useTriA: true = subdivide mesh using triA indices, false = use triB indices
inline Mesh subdivideAtIntersection(const Mesh& targetMesh, const Mesh& otherMesh,
                                     const IntersectionResult& intResult,
                                     bool useTriA, double eps = 1e-8) {
	Mesh result;

	// Copy all vertices from target mesh
	int nVerts = static_cast<int>(targetMesh.vertexCount());
	std::vector<int> oldToNew(nVerts, -1);
	for (int i = 0; i < nVerts; ++i) {
		const Vertex* v = targetMesh.findByIndex(i);
		if (v) oldToNew[i] = result.addVertex(v->x, v->y, v->z)->index;
	}

	// For each triangle, collect intersection points on its edges
	struct SplitPoint {
		int edgeIdx;
		Point pos;
		int vertIdx;
	};

	std::unordered_map<int, std::vector<SplitPoint>> triSplits;

	for (const auto& curve : intResult.curves) {
		for (const auto& seg : curve.segments) {
			int triIdx = useTriA ? seg.triA : seg.triB;
			const Triangle* tri = targetMesh.triangle(triIdx);
			if (!tri) continue;

			Point v0(tri->vertex(0)->x, tri->vertex(0)->y, tri->vertex(0)->z);
			Point v1(tri->vertex(1)->x, tri->vertex(1)->y, tri->vertex(1)->z);
			Point v2(tri->vertex(2)->x, tri->vertex(2)->y, tri->vertex(2)->z);

			// Check start point
			int e0 = findEdge(seg.start, v0, v1, v2, eps);
			if (e0 >= 0) {
				std::array<int, 3> tv = {oldToNew[tri->vertex(0)->index],
				                         oldToNew[tri->vertex(1)->index],
				                         oldToNew[tri->vertex(2)->index]};
				auto [a, b] = getEdge(tv, e0);
				int vi = insertPointOnEdge(result, a, b, seg.start, eps);
				if (vi >= 0) triSplits[triIdx].push_back({e0, seg.start, vi});
			}

			// Check end point
			int e1 = findEdge(seg.end, v0, v1, v2, eps);
			if (e1 >= 0) {
				std::array<int, 3> tv = {oldToNew[tri->vertex(0)->index],
				                         oldToNew[tri->vertex(1)->index],
				                         oldToNew[tri->vertex(2)->index]};
				auto [a, b] = getEdge(tv, e1);
				int vi = insertPointOnEdge(result, a, b, seg.end, eps);
				if (vi >= 0) triSplits[triIdx].push_back({e1, seg.end, vi});
			}
		}
	}

	// Process each triangle
	for (int i = 0; i < static_cast<int>(targetMesh.triangleCount()); ++i) {
		const Triangle* tri = targetMesh.triangle(i);
		if (!tri) continue;

		std::array<int, 3> triVerts = {
			oldToNew[tri->vertex(0)->index],
			oldToNew[tri->vertex(1)->index],
			oldToNew[tri->vertex(2)->index]
		};

		auto it = triSplits.find(i);
		if (it == triSplits.end()) {
			result.addTriangle(triVerts[0], triVerts[1], triVerts[2]);
			continue;
		}

		auto& splits = it->second;

		// Deduplicate
		std::vector<SplitPoint> uniqueSplits;
		for (auto& sp : splits) {
			bool dup = false;
			for (auto& us : uniqueSplits) {
				if (us.edgeIdx == sp.edgeIdx) {
					double d = Vec3(us.pos.x - sp.pos.x, us.pos.y - sp.pos.y, us.pos.z - sp.pos.z).cachedLength();
					if (d < eps) { dup = true; break; }
				}
			}
			if (!dup) uniqueSplits.push_back(sp);
		}

		if (uniqueSplits.size() < 2) {
			result.addTriangle(triVerts[0], triVerts[1], triVerts[2]);
			continue;
		}

		if (uniqueSplits.size() == 2) {
			auto newTris = splitTriangle(triVerts,
				uniqueSplits[0].edgeIdx, uniqueSplits[0].vertIdx,
				uniqueSplits[1].edgeIdx, uniqueSplits[1].vertIdx);
			for (auto& nt : newTris) {
				result.addTriangle(nt[0], nt[1], nt[2]);
			}
		} else {
			result.addTriangle(triVerts[0], triVerts[1], triVerts[2]);
		}
	}

	return result;
}

// Merge duplicate vertices
inline int mergeDuplicateVertices(Mesh& mesh, double eps = 1e-6) {
	int merged = 0;
	int n = static_cast<int>(mesh.vertexCount());
	if (n == 0) return 0;

	// Spatial hash
	std::unordered_map<int64_t, std::vector<int>> buckets;
	auto hash = [&](double x, double y, double z) -> int64_t {
		int ix = static_cast<int>(std::floor(x / eps));
		int iy = static_cast<int>(std::floor(y / eps));
		int iz = static_cast<int>(std::floor(z / eps));
		return int64_t(ix) * 73856093LL ^ int64_t(iy) * 19349663LL ^ int64_t(iz) * 83492791LL;
	};

	for (int i = 0; i < n; ++i) {
		const Vertex* v = mesh.findByIndex(i);
		if (v) buckets[hash(v->x, v->y, v->z)].push_back(i);
	}

	// Union-find
	std::vector<int> parent(n);
	for (int i = 0; i < n; ++i) parent[i] = i;

	std::function<int(int)> find = [&](int x) -> int {
		while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
		return x;
	};
	auto unite = [&](int a, int b) { a = find(a); b = find(b); if (a != b) parent[b] = a; };

	for (int i = 0; i < n; ++i) {
		const Vertex* vi = mesh.findByIndex(i);
		if (!vi) continue;
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dy = -1; dy <= 1; ++dy) {
				for (int dz = -1; dz <= 1; ++dz) {
					int64_t h = hash(vi->x + dx * eps, vi->y + dy * eps, vi->z + dz * eps);
					auto it = buckets.find(h);
					if (it == buckets.end()) continue;
					for (int j : it->second) {
						if (j <= i) continue;
						const Vertex* vj = mesh.findByIndex(j);
						if (!vj) continue;
						double d = Vec3(vi->x - vj->x, vi->y - vj->y, vi->z - vj->z).cachedLength();
						if (d < eps) unite(i, j);
					}
				}
			}
		}
	}

	for (int i = 0; i < n; ++i) if (find(i) != i) merged++;
	if (merged == 0) return 0;

	// Rebuild
	Mesh newMesh;
	std::vector<int> oldToNew(n, -1);
	std::unordered_map<int, int> rootToNew;

	for (int i = 0; i < n; ++i) {
		int root = find(i);
		if (rootToNew.find(root) == rootToNew.end()) {
			const Vertex* v = mesh.findByIndex(root);
			if (v) rootToNew[root] = newMesh.addVertex(v->x, v->y, v->z)->index;
		}
		oldToNew[i] = rootToNew[root];
	}

	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;
		int v0 = oldToNew[tri->vertex(0)->index];
		int v1 = oldToNew[tri->vertex(1)->index];
		int v2 = oldToNew[tri->vertex(2)->index];
		if (v0 == v1 || v1 == v2 || v2 == v0) continue;
		newMesh.addTriangle(v0, v1, v2);
	}

	mesh = std::move(newMesh);
	return merged;
}

// Find intersection curves
inline IntersectionResult findIntersection(const Mesh& meshA, const Mesh& meshB,
                                            const geo::BVH& bvhA, const geo::BVH& bvhB,
                                            double eps = 1e-6) {
	IntersectionResult result;

	// Collect all intersection segments
	struct RawSegment {
		Point start, end;
		int triA, triB;
	};
	std::vector<RawSegment> rawSegments;

	for (std::size_t i = 0; i < meshA.triangleCount(); ++i) {
		const Triangle* triA = meshA.triangle(static_cast<int>(i));
		if (!triA) continue;

		geo::AABB triBoxA = geo::computeTriangleAABB(triA);
		auto candidates = bvhB.queryAABB(triBoxA);

		for (int j : candidates) {
			const Triangle* triB = meshB.triangle(j);
			if (!triB) continue;

			auto intResult = geo::intersectTriangles(triA, triB);
			if (intResult.type != geo::TriTriIntersectionType::None && intResult.points.size() >= 2) {
				rawSegments.push_back({intResult.points[0], intResult.points[1],
				                       static_cast<int>(i), j});
			}
		}
	}

	// Deduplicate segments (same start/end points)
	auto pointKey = [&](const Point& p) -> int64_t {
		int64_t ix = static_cast<int64_t>(std::round(p.x / eps));
		int64_t iy = static_cast<int64_t>(std::round(p.y / eps));
		int64_t iz = static_cast<int64_t>(std::round(p.z / eps));
		return ix * 73856093LL ^ iy * 19349663LL ^ iz * 83492791LL;
	};

	std::unordered_set<int64_t> seen;
	for (auto& seg : rawSegments) {
		int64_t key = pointKey(seg.start) ^ (pointKey(seg.end) * 31);
		if (seen.count(key)) continue;
		seen.insert(key);

		IntersectionCurve curve;
		curve.segments.push_back({seg.start, seg.end, seg.triA, seg.triB});
		result.curves.push_back(curve);
		result.hasIntersection = true;
	}

	return result;
}

} // namespace mesh_intersect
