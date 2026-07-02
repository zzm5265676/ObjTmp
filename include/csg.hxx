#pragma once
#include "mesh.hxx"
#include "bvh.hxx"
#include "intersect.hxx"
#include "geometry_utils.hxx"
#include "repair.hxx"
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <functional>

namespace csg {

enum class BooleanOp {
	Union,
	Intersection,
	Difference
};

struct CSGResult {
	Mesh mesh;
	bool success = false;
	std::string error;
};

// Stitch boundary edges at intersection
// For each boundary edge (a, b), find the reverse boundary edge (b, a)
// and create a quad (2 triangles) to fill the gap.
inline int stitchBoundary(Mesh& mesh, double eps = 1e-6) {
	int stitched = 0;

	// Step 1: Count directed edge occurrences
	std::unordered_map<uint64_t, int> edgeCount;
	auto edgeKey = [](int a, int b) -> uint64_t {
		return (uint64_t(a) << 32) | uint64_t(b);
	};

	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;
		for (int j = 0; j < 3; ++j) {
			int v0 = tri->vertex(j)->index;
			int v1 = tri->vertex((j + 1) % 3)->index;
			edgeCount[edgeKey(v0, v1)]++;
		}
	}

	// Step 2: Find boundary edges and pair them
	// Boundary edge: (a, b) exists but (b, a) doesn't
	// For stitching, we need to find (a, b) and (b, a) pairs
	// where both are boundary edges (from different meshes)

	std::vector<std::pair<int, int>> toStitch;
	std::unordered_set<uint64_t> processed;

	for (auto& [key, count] : edgeCount) {
		if (count == 0) continue;
		int a = static_cast<int>(key >> 32);
		int b = static_cast<int>(key & 0xFFFFFFFF);
		uint64_t revKey = edgeKey(b, a);

		// Check if reverse edge exists
		auto revIt = edgeCount.find(revKey);
		if (revIt != edgeCount.end() && revIt->second > 0) {
			// Both directions exist - this is an interior edge, skip
			continue;
		}

		// This is a boundary edge (a, b)
		// The reverse (b, a) doesn't exist, so we need to stitch
		// by creating a triangle (a, b, ?) - but we need a third vertex

		// Find the third vertex: look at the triangle that owns this edge
		// and find the vertex opposite to this edge
		int thirdVert = -1;
		for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
			const Triangle* tri = mesh.triangle(static_cast<int>(i));
			if (!tri) continue;
			for (int j = 0; j < 3; ++j) {
				int v0 = tri->vertex(j)->index;
				int v1 = tri->vertex((j + 1) % 3)->index;
				if (v0 == a && v1 == b) {
					thirdVert = tri->vertex((j + 2) % 3)->index;
					break;
				}
			}
			if (thirdVert >= 0) break;
		}

		if (thirdVert < 0) continue;

		// Create a triangle (b, a, thirdVert) - this is the reverse of the original
		// This fills the gap on the other side of the edge
		uint64_t stitchKey = edgeKey(b, a);
		if (!processed.count(stitchKey)) {
			// Check if this triangle already exists
			bool exists = false;
			for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
				const Triangle* tri = mesh.triangle(static_cast<int>(i));
				if (!tri) continue;
				int tv0 = tri->vertex(0)->index;
				int tv1 = tri->vertex(1)->index;
				int tv2 = tri->vertex(2)->index;
				if ((tv0 == b && tv1 == a && tv2 == thirdVert) ||
				    (tv0 == a && tv1 == thirdVert && tv2 == b) ||
				    (tv0 == thirdVert && tv1 == b && tv2 == a)) {
					exists = true;
					break;
				}
			}

			if (!exists) {
				// Check if triangle is non-degenerate
				const Vertex* vb = mesh.findByIndex(b);
				const Vertex* va = mesh.findByIndex(a);
				const Vertex* vt = mesh.findByIndex(thirdVert);
				if (vb && va && vt) {
					Vec3 e1(va->x - vb->x, va->y - vb->y, va->z - vb->z);
					Vec3 e2(vt->x - vb->x, vt->y - vb->y, vt->z - vb->z);
					double area = e1.cross(e2).cachedLength() * 0.5;
					if (area > eps) {
						mesh.addTriangle(b, a, thirdVert);
						processed.insert(stitchKey);
						stitched++;
					}
				}
			}
		}
	}

	return stitched;
}

// Core boolean operation with stitching
inline CSGResult booleanOperation(const Mesh& meshA, const Mesh& meshB, BooleanOp op) {
	CSGResult result;

	// Step 1: Build BVH for acceleration
	geo::BVH bvhA(meshA);
	geo::BVH bvhB(meshB);

	// Step 2: Find intersections
	auto intResult = mesh_intersect::findIntersection(meshA, meshB, bvhA, bvhB);

	if (!intResult.hasIntersection) {
		// No intersection - check containment
		geo::PointClass classA = classifyPointInMesh(
			Point(meshA.triangle(0)->vertex(0)->x,
			      meshA.triangle(0)->vertex(0)->y,
			      meshA.triangle(0)->vertex(0)->z), meshB);
		geo::PointClass classB = classifyPointInMesh(
			Point(meshB.triangle(0)->vertex(0)->x,
			      meshB.triangle(0)->vertex(0)->y,
			      meshB.triangle(0)->vertex(0)->z), meshA);

		bool aInB = (classA == geo::PointClass::Inside);
		bool bInA = (classB == geo::PointClass::Inside);

		// Helper to copy a mesh
		auto copyMesh = [](const Mesh& src) -> Mesh {
			Mesh out;
			for (std::size_t i = 0; i < src.vertexCount(); ++i) {
				const Vertex* v = src.findByIndex(static_cast<int>(i));
				if (v) out.addVertex(v->x, v->y, v->z);
			}
			for (std::size_t i = 0; i < src.triangleCount(); ++i) {
				const Triangle* t = src.triangle(static_cast<int>(i));
				if (t) out.addTriangle(t->vertex(0)->index, t->vertex(1)->index, t->vertex(2)->index);
			}
			return out;
		};

		switch (op) {
		case BooleanOp::Union:
			if (aInB) result.mesh = copyMesh(meshB);
			else if (bInA) result.mesh = copyMesh(meshA);
			else {
				Mesh out = copyMesh(meshA);
				int offset = static_cast<int>(out.vertexCount());
				for (std::size_t i = 0; i < meshB.vertexCount(); ++i) {
					const Vertex* v = meshB.findByIndex(static_cast<int>(i));
					if (v) out.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshB.triangleCount(); ++i) {
					const Triangle* t = meshB.triangle(static_cast<int>(i));
					if (t) out.addTriangle(t->vertex(0)->index + offset, t->vertex(1)->index + offset, t->vertex(2)->index + offset);
				}
				result.mesh = std::move(out);
			}
			break;
		case BooleanOp::Intersection:
			if (aInB) result.mesh = copyMesh(meshA);
			else if (bInA) result.mesh = copyMesh(meshB);
			break;
		case BooleanOp::Difference:
			if (aInB) result.mesh = Mesh();
			else if (bInA) result.mesh = copyMesh(meshA);
			else result.mesh = copyMesh(meshA);
			break;
		}

		result.success = true;
		return result;
	}

	// Step 3: Subdivide at intersection
	Mesh subdivA = mesh_intersect::subdivideAtIntersection(meshA, intResult.curves);
	Mesh subdivB = mesh_intersect::subdivideAtIntersection(meshB, intResult.curves);

	// Step 4: Merge duplicate vertices within each mesh
	mesh_intersect::mergeDuplicateVertices(subdivA, 1e-5);
	mesh_intersect::mergeDuplicateVertices(subdivB, 1e-5);

	// Step 5: Combine into output
	Mesh output;
	int offsetB = static_cast<int>(subdivA.vertexCount());

	for (std::size_t i = 0; i < subdivA.vertexCount(); ++i) {
		const Vertex* v = subdivA.findByIndex(static_cast<int>(i));
		if (v) output.addVertex(v->x, v->y, v->z);
	}
	for (std::size_t i = 0; i < subdivB.vertexCount(); ++i) {
		const Vertex* v = subdivB.findByIndex(static_cast<int>(i));
		if (v) output.addVertex(v->x, v->y, v->z);
	}

	// Classify and keep triangles from A
	for (std::size_t i = 0; i < subdivA.triangleCount(); ++i) {
		const Triangle* tri = subdivA.triangle(static_cast<int>(i));
		if (!tri) continue;

		Point centroid(
			(tri->vertex(0)->x + tri->vertex(1)->x + tri->vertex(2)->x) / 3.0,
			(tri->vertex(0)->y + tri->vertex(1)->y + tri->vertex(2)->y) / 3.0,
			(tri->vertex(0)->z + tri->vertex(1)->z + tri->vertex(2)->z) / 3.0);

		geo::PointClass cls = classifyPointInMesh(centroid, meshB);
		bool keep = false;
		switch (op) {
		case BooleanOp::Union:       keep = (cls == geo::PointClass::Outside); break;
		case BooleanOp::Intersection: keep = (cls == geo::PointClass::Inside); break;
		case BooleanOp::Difference:   keep = (cls == geo::PointClass::Outside); break;
		}

		if (keep) {
			output.addTriangle(tri->vertex(0)->index, tri->vertex(1)->index, tri->vertex(2)->index);
		}
	}

	// Classify and keep triangles from B
	for (std::size_t i = 0; i < subdivB.triangleCount(); ++i) {
		const Triangle* tri = subdivB.triangle(static_cast<int>(i));
		if (!tri) continue;

		Point centroid(
			(tri->vertex(0)->x + tri->vertex(1)->x + tri->vertex(2)->x) / 3.0,
			(tri->vertex(0)->y + tri->vertex(1)->y + tri->vertex(2)->y) / 3.0,
			(tri->vertex(0)->z + tri->vertex(1)->z + tri->vertex(2)->z) / 3.0);

		geo::PointClass cls = classifyPointInMesh(centroid, meshA);
		bool keep = false;
		switch (op) {
		case BooleanOp::Union:       keep = (cls == geo::PointClass::Outside); break;
		case BooleanOp::Intersection: keep = (cls == geo::PointClass::Inside); break;
		case BooleanOp::Difference:   keep = (cls == geo::PointClass::Inside); break;
		}

		if (keep) {
			int v0 = tri->vertex(0)->index + offsetB;
			int v1 = tri->vertex(1)->index + offsetB;
			int v2 = tri->vertex(2)->index + offsetB;
			if (op == BooleanOp::Difference) {
				output.addTriangle(v0, v2, v1);
			} else {
				output.addTriangle(v0, v1, v2);
			}
		}
	}

	// Step 6: Merge duplicate vertices at intersection boundary
	mesh_intersect::mergeDuplicateVertices(output, 1e-4);

	// Step 7: Stitch boundary edges
	stitchBoundary(output);

	// Step 8: Merge again after stitching
	mesh_intersect::mergeDuplicateVertices(output, 1e-4);

	// Step 9: Repair
	mesh_repair::removeDegenerateTriangles(output, 1e-12);
	mesh_repair::fixOrientation(output);
	mesh_repair::removeIsolatedVertices(output);

	result.mesh = std::move(output);
	result.success = true;
	return result;
}

inline CSGResult meshUnion(const Mesh& a, const Mesh& b) {
	return booleanOperation(a, b, BooleanOp::Union);
}

inline CSGResult meshIntersection(const Mesh& a, const Mesh& b) {
	return booleanOperation(a, b, BooleanOp::Intersection);
}

inline CSGResult meshDifference(const Mesh& a, const Mesh& b) {
	return booleanOperation(a, b, BooleanOp::Difference);
}

} // namespace csg
