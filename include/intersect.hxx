#pragma once
#include "mesh.hxx"
#include "bvh.hxx"
#include "geometry_utils.hxx"
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <set>

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

// Split a triangle by a line segment (two intersection points on edges)
// Returns the new triangles as vertex index triples
struct SplitTriangle {
	int v0, v1, v2;
};

// Split edge at a point, returns new vertex index
inline int splitEdgeAtPoint(Mesh& mesh, int v0idx, int v1idx, const Point& p, double eps) {
	const Vertex* v0 = mesh.findByIndex(v0idx);
	const Vertex* v1 = mesh.findByIndex(v1idx);
	if (!v0 || !v1) return -1;

	// Check if point is near an endpoint
	double d0 = Vec3(p.x - v0->x, p.y - v0->y, p.z - v0->z).cachedLength();
	double d1 = Vec3(p.x - v1->x, p.y - v1->y, p.z - v1->z).cachedLength();

	if (d0 < eps) return v0idx;
	if (d1 < eps) return v1idx;

	// Insert new vertex on edge
	return mesh.addVertex(p.x, p.y, p.z)->index;
}

// Find which edge of a triangle a point lies on (or near)
// Returns edge index (0,1,2) or -1
inline int pointOnTriangleEdge(const Point& p, const Triangle* tri, double eps) {
	for (int i = 0; i < 3; ++i) {
		const Vertex* v0 = tri->vertex(i);
		const Vertex* v1 = tri->vertex((i + 1) % 3);
		if (!v0 || !v1) continue;

		Point a(v0->x, v0->y, v0->z);
		Point b(v1->x, v1->y, v1->z);

		if (geo::pointOnSegment(p, a, b, eps)) return i;
	}
	return -1;
}

// Split a triangle at two intersection points on its edges
// Returns new triangles (as vertex index triples in the mesh)
inline std::vector<SplitTriangle> splitTriangleAtTwoPoints(
	Mesh& mesh, const Triangle* tri,
	int splitEdge0, const Point& p0, int newV0,
	int splitEdge1, const Point& p1, int newV1,
	double eps)
{
	std::vector<SplitTriangle> result;

	int vi[3] = { tri->vertex(0)->index, tri->vertex(1)->index, tri->vertex(2)->index };

	// Map edge index to vertex pairs
	// edge 0: v0-v1, edge 1: v1-v2, edge 2: v2-v0

	// Case: both points on same edge -> no split needed (degenerate)
	if (splitEdge0 == splitEdge1) return result;

	// Identify which edges are split and create the new vertex mapping
	// We need to handle all combinations of edge splits

	// Build edge->newVertex map
	int edgeNewVert[3] = { -1, -1, -1 };
	edgeNewVert[splitEdge0] = newV0;
	edgeNewVert[splitEdge1] = newV1;

	// The two split edges share a vertex
	// Find the shared vertex between the two split edges
	int sharedVert = -1;
	int otherEdge0 = -1, otherEdge1 = -1;

	// edge 0: v0-v1, edge 1: v1-v2, edge 2: v2-v0
	// Shared vertex between edge0 and edge1 is v1
	// Shared vertex between edge1 and edge2 is v2
	// Shared vertex between edge2 and edge0 is v0

	if ((splitEdge0 == 0 && splitEdge1 == 1) || (splitEdge0 == 1 && splitEdge1 == 0)) {
		sharedVert = vi[1]; // v1
		otherEdge0 = (splitEdge0 == 0) ? 0 : 1;
		otherEdge1 = (splitEdge0 == 0) ? 1 : 0;
	} else if ((splitEdge0 == 1 && splitEdge1 == 2) || (splitEdge0 == 2 && splitEdge1 == 1)) {
		sharedVert = vi[2]; // v2
		otherEdge0 = (splitEdge0 == 1) ? 1 : 2;
		otherEdge1 = (splitEdge0 == 1) ? 2 : 1;
	} else if ((splitEdge0 == 2 && splitEdge1 == 0) || (splitEdge0 == 0 && splitEdge1 == 2)) {
		sharedVert = vi[0]; // v0
		otherEdge0 = (splitEdge0 == 2) ? 2 : 0;
		otherEdge1 = (splitEdge0 == 2) ? 0 : 2;
	}

	if (sharedVert < 0) return result;

	// Get the new vertices on each edge
	int nv0 = edgeNewVert[splitEdge0];
	int nv1 = edgeNewVert[splitEdge1];

	if (nv0 < 0 || nv1 < 0) return result;

	// Find the vertex opposite to the shared vertex
	// That vertex is not on either split edge
	int oppositeVert = -1;
	for (int i = 0; i < 3; ++i) {
		if (vi[i] != sharedVert) {
			// Check if this vertex is on either split edge
			bool onEdge0 = false, onEdge1 = false;
			if (splitEdge0 == 0 && (i == 0 || i == 1)) onEdge0 = true;
			if (splitEdge0 == 1 && (i == 1 || i == 2)) onEdge0 = true;
			if (splitEdge0 == 2 && (i == 2 || i == 0)) onEdge0 = true;
			if (splitEdge1 == 0 && (i == 0 || i == 1)) onEdge1 = true;
			if (splitEdge1 == 1 && (i == 1 || i == 2)) onEdge1 = true;
			if (splitEdge1 == 2 && (i == 2 || i == 0)) onEdge1 = true;

			if (!onEdge0 && !onEdge1) {
				oppositeVert = vi[i];
				break;
			}
		}
	}

	if (oppositeVert < 0) return result;

	// Now we have:
	// sharedVert: the vertex shared by both split edges
	// oppositeVert: the vertex opposite to sharedVert
	// nv0: new vertex on splitEdge0
	// nv1: new vertex on splitEdge1

	// The triangle is split into 3 triangles:
	// 1. (sharedVert, nv0, nv1)
	// 2. (nv0, oppositeVert, nv1) - this is the middle quad split into 2
	// 3. Need to figure out the correct winding

	// Find which original vertex is on splitEdge0 but not sharedVert
	int edge0Other = -1;
	if (splitEdge0 == 0) edge0Other = (vi[0] == sharedVert) ? vi[1] : vi[0];
	else if (splitEdge0 == 1) edge0Other = (vi[1] == sharedVert) ? vi[2] : vi[1];
	else if (splitEdge0 == 2) edge0Other = (vi[2] == sharedVert) ? vi[0] : vi[2];

	int edge1Other = -1;
	if (splitEdge1 == 0) edge1Other = (vi[0] == sharedVert) ? vi[1] : vi[0];
	else if (splitEdge1 == 1) edge1Other = (vi[1] == sharedVert) ? vi[2] : vi[1];
	else if (splitEdge1 == 2) edge1Other = (vi[2] == sharedVert) ? vi[0] : vi[2];

	// Split into 3 triangles:
	// (sharedVert, nv0, nv1)
	// (nv0, edge0Other, oppositeVert)
	// (nv1, oppositeVert, edge1Other)
	// But we need to be careful about winding order

	result.push_back({sharedVert, nv0, nv1});
	result.push_back({nv0, oppositeVert, nv1});
	result.push_back({nv0, edge0Other, oppositeVert});
	result.push_back({nv1, oppositeVert, edge1Other});

	return result;
}

// Subdivide mesh at intersection curves with proper triangle splitting
inline Mesh subdivideAtIntersection(const Mesh& mesh,
                                     const std::vector<IntersectionCurve>& curves,
                                     double eps = 1e-6) {
	Mesh newMesh;
	int nVerts = static_cast<int>(mesh.vertexCount());

	// Copy all vertices
	std::vector<int> oldToNew(nVerts, -1);
	for (int i = 0; i < nVerts; ++i) {
		const Vertex* v = mesh.findByIndex(i);
		if (v) oldToNew[i] = newMesh.addVertex(v->x, v->y, v->z)->index;
	}

	// Collect all intersection points and which triangle/edge they're on
	struct IntPoint {
		Point pos;
		int triIdx;
		int edgeIdx; // which edge of the triangle (-1 if not on edge)
		int newVertIdx;
	};

	std::vector<IntPoint> intPoints;
	for (const auto& curve : curves) {
		for (const auto& seg : curve.segments) {
			const Triangle* triA = mesh.triangle(seg.triA);
			if (!triA) continue;

			// Check start point
			int edge0 = pointOnTriangleEdge(seg.start, triA, eps);
			intPoints.push_back({seg.start, seg.triA, edge0, -1});

			// Check end point
			int edge1 = pointOnTriangleEdge(seg.end, triA, eps);
			intPoints.push_back({seg.end, seg.triA, edge1, -1});
		}
	}

	// Create new vertices for intersection points (snap to edges if needed)
	for (auto& ip : intPoints) {
		if (ip.edgeIdx >= 0) {
			const Triangle* tri = mesh.triangle(ip.triIdx);
			int v0 = tri->vertex(ip.edgeIdx)->index;
			int v1 = tri->vertex((ip.edgeIdx + 1) % 3)->index;
			ip.newVertIdx = splitEdgeAtPoint(newMesh, oldToNew[v0], oldToNew[v1], ip.pos, eps);
		} else {
			ip.newVertIdx = newMesh.addVertex(ip.pos.x, ip.pos.y, ip.pos.z)->index;
		}
	}

	// Group intersection points by triangle
	std::unordered_map<int, std::vector<IntPoint*>> triSplits;
	for (auto& ip : intPoints) {
		triSplits[ip.triIdx].push_back(&ip);
	}

	// Process each triangle
	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;

		auto it = triSplits.find(static_cast<int>(i));
		if (it == triSplits.end()) {
			// No intersection - copy triangle as-is
			newMesh.addTriangle(
				oldToNew[tri->vertex(0)->index],
				oldToNew[tri->vertex(1)->index],
				oldToNew[tri->vertex(2)->index]);
			continue;
		}

		auto& ips = it->second;

		if (ips.size() == 2 && ips[0]->edgeIdx >= 0 && ips[1]->edgeIdx >= 0) {
			// Two intersection points on edges - split triangle
			auto splits = splitTriangleAtTwoPoints(
				newMesh, tri,
				ips[0]->edgeIdx, ips[0]->pos, ips[0]->newVertIdx,
				ips[1]->edgeIdx, ips[1]->pos, ips[1]->newVertIdx,
				eps);

			if (splits.empty()) {
				// Fallback: copy original triangle
				newMesh.addTriangle(
					oldToNew[tri->vertex(0)->index],
					oldToNew[tri->vertex(1)->index],
					oldToNew[tri->vertex(2)->index]);
			} else {
				for (auto& st : splits) {
					newMesh.addTriangle(st.v0, st.v1, st.v2);
				}
			}
		} else {
			// Other cases: copy triangle as-is (simplified)
			newMesh.addTriangle(
				oldToNew[tri->vertex(0)->index],
				oldToNew[tri->vertex(1)->index],
				oldToNew[tri->vertex(2)->index]);
		}
	}

	return newMesh;
}

// Merge duplicate vertices in a mesh (vertices within eps distance)
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

	// Find merge groups
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

	// Count merges
	for (int i = 0; i < n; ++i) if (find(i) != i) merged++;
	if (merged == 0) return 0;

	// Rebuild mesh
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
		if (v0 == v1 || v1 == v2 || v2 == v0) continue; // degenerate
		newMesh.addTriangle(v0, v1, v2);
	}

	mesh = std::move(newMesh);
	return merged;
}

// Find intersection curves between two meshes
inline IntersectionResult findIntersection(const Mesh& meshA, const Mesh& meshB,
                                            const geo::BVH& bvhA, const geo::BVH& bvhB) {
	IntersectionResult result;

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
				IntersectionSegment seg;
				seg.start = intResult.points[0];
				seg.end = intResult.points[1];
				seg.triA = static_cast<int>(i);
				seg.triB = j;

				IntersectionCurve curve;
				curve.segments.push_back(seg);
				result.curves.push_back(curve);
				result.hasIntersection = true;
			}
		}
	}

	return result;
}

} // namespace mesh_intersect
