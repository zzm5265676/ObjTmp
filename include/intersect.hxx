#pragma once
#include "mesh.hxx"
#include "bvh.hxx"
#include "geometry_utils.hxx"
#include <vector>
#include <unordered_map>
#include <algorithm>

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

struct SubdividedMesh {
	Mesh mesh;
	std::vector<int> newVertexIndices;
};

// Find intersection curves between two meshes
inline IntersectionResult findIntersection(const Mesh& meshA, const Mesh& meshB,
                                            const geo::BVH& bvhA, const geo::BVH& bvhB) {
	IntersectionResult result;

	// For each triangle in A, find intersecting triangles in B
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

	// TODO: Connect segments into curves

	return result;
}

// Subdivide mesh at intersection curves
inline SubdividedMesh subdivideAtIntersection(Mesh& mesh,
                                               const std::vector<IntersectionCurve>& curves) {
	SubdividedMesh result;

	// Start with a copy of the original mesh
	// For each intersection segment, insert new vertices and split triangles

	// Simplified: just add the intersection points as new vertices
	// A full implementation would split triangles along the intersection curves

	Mesh newMesh;
	std::vector<int> oldToNew(mesh.vertexCount(), -1);

	// Copy all vertices
	for (std::size_t i = 0; i < mesh.vertexCount(); ++i) {
		const Vertex* v = mesh.findByIndex(static_cast<int>(i));
		if (v) oldToNew[i] = newMesh.addVertex(v->x, v->y, v->z)->index;
	}

	// Copy all triangles
	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;
		newMesh.addTriangle(
			oldToNew[tri->vertex(0)->index],
			oldToNew[tri->vertex(1)->index],
			oldToNew[tri->vertex(2)->index]);
	}

	// Add intersection points as new vertices
	for (const auto& curve : curves) {
		for (const auto& seg : curve.segments) {
			int vi = newMesh.addVertex(seg.start.x, seg.start.y, seg.start.z)->index;
			result.newVertexIndices.push_back(vi);
			vi = newMesh.addVertex(seg.end.x, seg.end.y, seg.end.z)->index;
			result.newVertexIndices.push_back(vi);
		}
	}

	result.mesh = std::move(newMesh);
	return result;
}

} // namespace mesh_intersect
