#pragma once
#include "mesh.hxx"
#include "bvh.hxx"
#include "intersect.hxx"
#include "geometry_utils.hxx"
#include "repair.hxx"
#include <vector>
#include <string>

namespace csg {

enum class BooleanOp { Union, Intersection, Difference };

struct CSGResult {
	Mesh mesh;
	bool success = false;
	std::string error;
};

inline Mesh copyMesh(const Mesh& src) {
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
}

struct MergedMesh {
	Mesh mesh;
	std::vector<int> mapA;
	std::vector<int> mapB;
};

inline MergedMesh mergeMeshes(const Mesh& a, const Mesh& b, double eps) {
	MergedMesh result;
	result.mapA.resize(a.vertexCount(), -1);
	result.mapB.resize(b.vertexCount(), -1);

	// Add all vertices from A
	for (std::size_t i = 0; i < a.vertexCount(); ++i) {
		const Vertex* v = a.findByIndex(static_cast<int>(i));
		if (v) result.mapA[i] = result.mesh.addVertex(v->x, v->y, v->z)->index;
	}

	// Add vertices from B, merging with A
	for (std::size_t i = 0; i < b.vertexCount(); ++i) {
		const Vertex* v = b.findByIndex(static_cast<int>(i));
		if (!v) continue;

		int match = -1;
		for (int j = 0; j < static_cast<int>(a.vertexCount()); ++j) {
			const Vertex* u = a.findByIndex(j);
			if (!u) continue;
			double dx = std::abs(v->x - u->x);
			double dy = std::abs(v->y - u->y);
			double dz = std::abs(v->z - u->z);
			if (dx < eps && dy < eps && dz < eps) {
				match = result.mapA[j];
				break;
			}
		}

		if (match >= 0) {
			result.mapB[i] = match;
		} else {
			result.mapB[i] = result.mesh.addVertex(v->x, v->y, v->z)->index;
		}
	}

	return result;
}

inline CSGResult booleanOperation(const Mesh& meshA, const Mesh& meshB, BooleanOp op) {
	CSGResult result;
	double eps = 1e-4;

	// Check for intersection
	geo::BVH bvhA(meshA), bvhB(meshB);
	auto intResult = mesh_intersect::findIntersection(meshA, meshB, bvhA, bvhB);

	if (!intResult.hasIntersection) {
		// Handle containment/disjoint
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
			if (aInB) result.mesh = copyMesh(meshB);
			else if (bInA) result.mesh = copyMesh(meshA);
			else {
				result.mesh = copyMesh(meshA);
				int off = static_cast<int>(result.mesh.vertexCount());
				for (std::size_t i = 0; i < meshB.vertexCount(); ++i) {
					const Vertex* v = meshB.findByIndex(static_cast<int>(i));
					if (v) result.mesh.addVertex(v->x, v->y, v->z);
				}
				for (std::size_t i = 0; i < meshB.triangleCount(); ++i) {
					const Triangle* t = meshB.triangle(static_cast<int>(i));
					if (t) result.mesh.addTriangle(t->vertex(0)->index+off, t->vertex(1)->index+off, t->vertex(2)->index+off);
				}
			}
			break;
		case BooleanOp::Intersection:
			if (aInB) result.mesh = copyMesh(meshA);
			else if (bInA) result.mesh = copyMesh(meshB);
			break;
		case BooleanOp::Difference:
			if (aInB) result.mesh = Mesh();
			else result.mesh = copyMesh(meshA);
			break;
		}
		result.success = true;
		return result;
	}

	// Merge both meshes into one combined mesh
	auto merged = mergeMeshes(meshA, meshB, eps);
	Mesh& combined = merged.mesh;
	auto& mapA = merged.mapA;
	auto& mapB = merged.mapB;

	// Add triangles from A
	for (std::size_t i = 0; i < meshA.triangleCount(); ++i) {
		const Triangle* tri = meshA.triangle(static_cast<int>(i));
		if (!tri) continue;

		Point c((tri->vertex(0)->x + tri->vertex(1)->x + tri->vertex(2)->x) / 3.0,
		        (tri->vertex(0)->y + tri->vertex(1)->y + tri->vertex(2)->y) / 3.0,
		        (tri->vertex(0)->z + tri->vertex(1)->z + tri->vertex(2)->z) / 3.0);

		geo::PointClass cls = classifyPointInMesh(c, meshB);
		bool keep = false;
		switch (op) {
		case BooleanOp::Union:       keep = (cls != geo::PointClass::Inside); break;
		case BooleanOp::Intersection: keep = (cls != geo::PointClass::Outside); break;
		case BooleanOp::Difference:   keep = (cls != geo::PointClass::Inside); break;
		}

		if (keep) {
			int v0 = mapA[tri->vertex(0)->index];
			int v1 = mapA[tri->vertex(1)->index];
			int v2 = mapA[tri->vertex(2)->index];
			if (v0 >= 0 && v1 >= 0 && v2 >= 0 && v0 != v1 && v1 != v2 && v2 != v0) {
				combined.addTriangle(v0, v1, v2);
			}
		}
	}

	// Add triangles from B
	for (std::size_t i = 0; i < meshB.triangleCount(); ++i) {
		const Triangle* tri = meshB.triangle(static_cast<int>(i));
		if (!tri) continue;

		Point c((tri->vertex(0)->x + tri->vertex(1)->x + tri->vertex(2)->x) / 3.0,
		        (tri->vertex(0)->y + tri->vertex(1)->y + tri->vertex(2)->y) / 3.0,
		        (tri->vertex(0)->z + tri->vertex(1)->z + tri->vertex(2)->z) / 3.0);

		geo::PointClass cls = classifyPointInMesh(c, meshA);
		bool keep = false;
		switch (op) {
		case BooleanOp::Union:       keep = (cls != geo::PointClass::Inside); break;
		case BooleanOp::Intersection: keep = (cls != geo::PointClass::Outside); break;
		case BooleanOp::Difference:   keep = (cls != geo::PointClass::Outside); break;
		}

		if (keep) {
			int v0 = mapB[tri->vertex(0)->index];
			int v1 = mapB[tri->vertex(1)->index];
			int v2 = mapB[tri->vertex(2)->index];
			if (v0 >= 0 && v1 >= 0 && v2 >= 0 && v0 != v1 && v1 != v2 && v2 != v0) {
				if (op == BooleanOp::Difference) combined.addTriangle(v0, v2, v1);
				else combined.addTriangle(v0, v1, v2);
			}
		}
	}

	// Remove degenerate triangles
	mesh_repair::removeDegenerateTriangles(combined, 1e-12);
	mesh_repair::fixOrientation(combined);
	mesh_repair::removeIsolatedVertices(combined);

	result.mesh = std::move(combined);
	result.success = true;
	return result;
}

inline CSGResult meshUnion(const Mesh& a, const Mesh& b) { return booleanOperation(a, b, BooleanOp::Union); }
inline CSGResult meshIntersection(const Mesh& a, const Mesh& b) { return booleanOperation(a, b, BooleanOp::Intersection); }
inline CSGResult meshDifference(const Mesh& a, const Mesh& b) { return booleanOperation(a, b, BooleanOp::Difference); }

} // namespace csg
