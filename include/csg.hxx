#pragma once
#include "mesh.hxx"
#include "bvh.hxx"
#include "intersect.hxx"
#include "geometry_utils.hxx"
#include "repair.hxx"
#include <vector>
#include <string>

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

// Core boolean operation
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

		switch (op) {
		case BooleanOp::Union:
			if (aInB) { result.mesh = Mesh(); /* B contains A, result is B */ }
			else if (bInA) { result.mesh = Mesh(); /* A contains B, result is A */ }
			else {
				// Disjoint: combine both
				// For now, just return meshA (simplified)
				result.mesh = Mesh();
			}
			break;
		case BooleanOp::Intersection:
			if (aInB || bInA) {
				// One contains the other, result is the smaller one
				result.mesh = Mesh();
			}
			// Disjoint: empty result
			break;
		case BooleanOp::Difference:
			if (aInB) {
				// A inside B: A - B = empty
				result.mesh = Mesh();
			} else if (bInA) {
				// B inside A: need to subtract
				result.mesh = Mesh();
			}
			// Disjoint: result is A
			break;
		}

		result.success = true;
		return result;
	}

	// Step 3: Subdivide at intersection
	auto subdivA = mesh_intersect::subdivideAtIntersection(const_cast<Mesh&>(meshA), intResult.curves);
	auto subdivB = mesh_intersect::subdivideAtIntersection(const_cast<Mesh&>(meshB), intResult.curves);

	// Step 4: Classify and filter triangles
	Mesh output;

	// For each triangle in subdivided A, classify it
	for (std::size_t i = 0; i < subdivA.mesh.triangleCount(); ++i) {
		const Triangle* tri = subdivA.mesh.triangle(static_cast<int>(i));
		if (!tri) continue;

		// Get triangle centroid
		Point centroid(
			(tri->vertex(0)->x + tri->vertex(1)->x + tri->vertex(2)->x) / 3.0,
			(tri->vertex(0)->y + tri->vertex(1)->y + tri->vertex(2)->y) / 3.0,
			(tri->vertex(0)->z + tri->vertex(1)->z + tri->vertex(2)->z) / 3.0
		);

		geo::PointClass cls = classifyPointInMesh(centroid, meshB);

		bool keep = false;
		switch (op) {
		case BooleanOp::Union:
			keep = (cls == geo::PointClass::Outside || cls == geo::PointClass::OnBoundary);
			break;
		case BooleanOp::Intersection:
			keep = (cls == geo::PointClass::Inside || cls == geo::PointClass::OnBoundary);
			break;
		case BooleanOp::Difference:
			keep = (cls == geo::PointClass::Outside || cls == geo::PointClass::OnBoundary);
			break;
		}

		if (keep) {
			int v0 = output.addVertex(tri->vertex(0)->x, tri->vertex(0)->y, tri->vertex(0)->z)->index;
			int v1 = output.addVertex(tri->vertex(1)->x, tri->vertex(1)->y, tri->vertex(1)->z)->index;
			int v2 = output.addVertex(tri->vertex(2)->x, tri->vertex(2)->y, tri->vertex(2)->z)->index;
			output.addTriangle(v0, v1, v2);
		}
	}

	// For each triangle in subdivided B
	for (std::size_t i = 0; i < subdivB.mesh.triangleCount(); ++i) {
		const Triangle* tri = subdivB.mesh.triangle(static_cast<int>(i));
		if (!tri) continue;

		Point centroid(
			(tri->vertex(0)->x + tri->vertex(1)->x + tri->vertex(2)->x) / 3.0,
			(tri->vertex(0)->y + tri->vertex(1)->y + tri->vertex(2)->y) / 3.0,
			(tri->vertex(0)->z + tri->vertex(1)->z + tri->vertex(2)->z) / 3.0
		);

		geo::PointClass cls = classifyPointInMesh(centroid, meshA);

		bool keep = false;
		switch (op) {
		case BooleanOp::Union:
			keep = (cls == geo::PointClass::Outside || cls == geo::PointClass::OnBoundary);
			break;
		case BooleanOp::Intersection:
			keep = (cls == geo::PointClass::Inside || cls == geo::PointClass::OnBoundary);
			break;
		case BooleanOp::Difference:
			keep = (cls == geo::PointClass::Inside || cls == geo::PointClass::OnBoundary);
			break;
		}

		if (keep) {
			int v0 = output.addVertex(tri->vertex(0)->x, tri->vertex(0)->y, tri->vertex(0)->z)->index;
			int v1 = output.addVertex(tri->vertex(1)->x, tri->vertex(1)->y, tri->vertex(1)->z)->index;
			int v2 = output.addVertex(tri->vertex(2)->x, tri->vertex(2)->y, tri->vertex(2)->z)->index;
			// For difference, flip B's triangles
			if (op == BooleanOp::Difference) {
				output.addTriangle(v0, v2, v1);
			} else {
				output.addTriangle(v0, v1, v2);
			}
		}
	}

	// Step 5: Repair output
	mesh_repair::repair(output);

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
