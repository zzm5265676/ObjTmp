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

// Core boolean operation with improved pipeline
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
			if (aInB) {
				// Deep copy meshB
				Mesh out;
				for (std::size_t i = 0; i < meshB.vertexCount(); ++i) {
					const Vertex* v = meshB.findByIndex(static_cast<int>(i));
					if (v) out.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshB.triangleCount(); ++i) {
					const Triangle* t = meshB.triangle(static_cast<int>(i));
					if (t) out.addTriangle(t->vertex(0)->index, t->vertex(1)->index, t->vertex(2)->index);
				}
				result.mesh = std::move(out);
			} else if (bInA) {
				Mesh out;
				for (std::size_t i = 0; i < meshA.vertexCount(); ++i) {
					const Vertex* v = meshA.findByIndex(static_cast<int>(i));
					if (v) out.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshA.triangleCount(); ++i) {
					const Triangle* t = meshA.triangle(static_cast<int>(i));
					if (t) out.addTriangle(t->vertex(0)->index, t->vertex(1)->index, t->vertex(2)->index);
				}
				result.mesh = std::move(out);
			} else {
				// Disjoint: combine both
				Mesh out;
				int offset = static_cast<int>(meshA.vertexCount());
				for (std::size_t i = 0; i < meshA.vertexCount(); ++i) {
					const Vertex* v = meshA.findByIndex(static_cast<int>(i));
					if (v) out.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshB.vertexCount(); ++i) {
					const Vertex* v = meshB.findByIndex(static_cast<int>(i));
					if (v) out.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshA.triangleCount(); ++i) {
					const Triangle* t = meshA.triangle(static_cast<int>(i));
					if (t) out.addTriangle(t->vertex(0)->index, t->vertex(1)->index, t->vertex(2)->index);
				}
				for (std::size_t i = 0; i < meshB.triangleCount(); ++i) {
					const Triangle* t = meshB.triangle(static_cast<int>(i));
					if (t) out.addTriangle(t->vertex(0)->index + offset, t->vertex(1)->index + offset, t->vertex(2)->index + offset);
				}
				result.mesh = std::move(out);
			}
			break;
		case BooleanOp::Intersection:
			if (aInB) {
				Mesh out;
				for (std::size_t i = 0; i < meshA.vertexCount(); ++i) {
					const Vertex* v = meshA.findByIndex(static_cast<int>(i));
					if (v) out.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshA.triangleCount(); ++i) {
					const Triangle* t = meshA.triangle(static_cast<int>(i));
					if (t) out.addTriangle(t->vertex(0)->index, t->vertex(1)->index, t->vertex(2)->index);
				}
				result.mesh = std::move(out);
			} else if (bInA) {
				Mesh out;
				for (std::size_t i = 0; i < meshB.vertexCount(); ++i) {
					const Vertex* v = meshB.findByIndex(static_cast<int>(i));
					if (v) out.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshB.triangleCount(); ++i) {
					const Triangle* t = meshB.triangle(static_cast<int>(i));
					if (t) out.addTriangle(t->vertex(0)->index, t->vertex(1)->index, t->vertex(2)->index);
				}
				result.mesh = std::move(out);
			}
			// Disjoint: empty result
			break;
		case BooleanOp::Difference:
			if (aInB) {
				// A inside B: A - B = empty
				result.mesh = Mesh();
			} else if (bInA) {
				// B inside A: result is A with a hole
				Mesh out;
				for (std::size_t i = 0; i < meshA.vertexCount(); ++i) {
					const Vertex* v = meshA.findByIndex(static_cast<int>(i));
					if (v) out.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshA.triangleCount(); ++i) {
					const Triangle* t = meshA.triangle(static_cast<int>(i));
					if (t) out.addTriangle(t->vertex(0)->index, t->vertex(1)->index, t->vertex(2)->index);
				}
				result.mesh = std::move(out);
			} else {
				// Disjoint: result is A
				Mesh out;
				for (std::size_t i = 0; i < meshA.vertexCount(); ++i) {
					const Vertex* v = meshA.findByIndex(static_cast<int>(i));
					if (v) out.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshA.triangleCount(); ++i) {
					const Triangle* t = meshA.triangle(static_cast<int>(i));
					if (t) out.addTriangle(t->vertex(0)->index, t->vertex(1)->index, t->vertex(2)->index);
				}
				result.mesh = std::move(out);
			}
			break;
		}

		result.success = true;
		return result;
	}

	// Step 3: Subdivide at intersection with proper triangle splitting
	Mesh subdivA = mesh_intersect::subdivideAtIntersection(meshA, intResult.curves);
	Mesh subdivB = mesh_intersect::subdivideAtIntersection(meshB, intResult.curves);

	// Step 4: Merge duplicate vertices at intersection boundary
	mesh_intersect::mergeDuplicateVertices(subdivA, 1e-5);
	mesh_intersect::mergeDuplicateVertices(subdivB, 1e-5);

	// Step 5: Classify and filter triangles
	Mesh output;
	int offsetB = static_cast<int>(subdivA.vertexCount());

	// Copy all vertices from A
	for (std::size_t i = 0; i < subdivA.vertexCount(); ++i) {
		const Vertex* v = subdivA.findByIndex(static_cast<int>(i));
		if (v) output.addVertex(v->x, v->y, v->z);
	}

	// Copy all vertices from B
	for (std::size_t i = 0; i < subdivB.vertexCount(); ++i) {
		const Vertex* v = subdivB.findByIndex(static_cast<int>(i));
		if (v) output.addVertex(v->x, v->y, v->z);
	}

	// Classify triangles from A
	for (std::size_t i = 0; i < subdivA.triangleCount(); ++i) {
		const Triangle* tri = subdivA.triangle(static_cast<int>(i));
		if (!tri) continue;

		Point centroid(
			(tri->vertex(0)->x + tri->vertex(1)->x + tri->vertex(2)->x) / 3.0,
			(tri->vertex(0)->y + tri->vertex(1)->y + tri->vertex(2)->y) / 3.0,
			(tri->vertex(0)->z + tri->vertex(1)->z + tri->vertex(2)->z) / 3.0
		);

		geo::PointClass cls = classifyPointInMesh(centroid, meshB);

		bool keep = false;
		switch (op) {
		case BooleanOp::Union:       keep = (cls == geo::PointClass::Outside); break;
		case BooleanOp::Intersection: keep = (cls == geo::PointClass::Inside); break;
		case BooleanOp::Difference:   keep = (cls == geo::PointClass::Outside); break;
		}

		if (keep) {
			output.addTriangle(
				tri->vertex(0)->index,
				tri->vertex(1)->index,
				tri->vertex(2)->index);
		}
	}

	// Classify triangles from B
	for (std::size_t i = 0; i < subdivB.triangleCount(); ++i) {
		const Triangle* tri = subdivB.triangle(static_cast<int>(i));
		if (!tri) continue;

		Point centroid(
			(tri->vertex(0)->x + tri->vertex(1)->x + tri->vertex(2)->x) / 3.0,
			(tri->vertex(0)->y + tri->vertex(1)->y + tri->vertex(2)->y) / 3.0,
			(tri->vertex(0)->z + tri->vertex(1)->z + tri->vertex(2)->z) / 3.0
		);

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
			// For difference, flip B's triangles
			if (op == BooleanOp::Difference) {
				output.addTriangle(v0, v2, v1);
			} else {
				output.addTriangle(v0, v1, v2);
			}
		}
	}

	// Step 6: Merge duplicate vertices in output
	mesh_intersect::mergeDuplicateVertices(output, 1e-5);

	// Step 7: Merge duplicate vertices at intersection boundary
	mesh_intersect::mergeDuplicateVertices(output, 1e-4);

	// Step 8: Remove degenerate triangles created by merging
	mesh_repair::removeDegenerateTriangles(output, 1e-12);

	// Step 9: Fix orientation
	mesh_repair::fixOrientation(output);

	// Step 10: Remove isolated vertices
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
