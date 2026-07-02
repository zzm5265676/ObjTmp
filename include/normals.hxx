#pragma once
#include "vec3.hxx"
#include "edge.hxx"
#include "vertex.hxx"
#include "triangle.hxx"
#include <cmath>
#include <vector>

class NormalComputer {
public:
	// ===== Per-element normal computation =====

	// Edge normal: average of adjacent face normals
	// Uses the half-edge structure: Edge::triangles() + Edge::opposite()->triangles()
	static Vec3 edgeNormal(const Edge& edge) {
		Vec3 sum(0, 0, 0);
		int count = 0;

		// Triangles on this directed edge
		for (const Triangle* tri : edge.triangles()) {
			if (tri) {
				sum += tri->normal();
				++count;
			}
		}
		// Triangles on the opposite directed edge
		const Edge* opp = edge.opposite();
		if (opp) {
			for (const Triangle* tri : opp->triangles()) {
				if (tri) {
					sum += tri->normal();
					++count;
				}
			}
		}

		if (count == 0) return Vec3(0, 0, 0);
		return (sum / static_cast<double>(count)).normalize();
	}

	// Edge dihedral angle: angle between the two adjacent face normals
	// Returns radians in [0, pi]. 0 = coplanar, pi = folded flat
	static double edgeDihedralAngle(const Edge& edge) {
		const Edge* opp = edge.opposite();
		if (!opp) return 0.0;  // boundary edge

		const Triangle* t1 = nullptr;
		const Triangle* t2 = nullptr;
		for (Triangle* t : edge.triangles()) { t1 = t; break; }
		for (Triangle* t : opp->triangles()) { t2 = t; break; }
		if (!t1 || !t2) return 0.0;

		double d = t1->normal().dot(t2->normal());
		d = (d < -1.0) ? -1.0 : (d > 1.0 ? 1.0 : d);
		return std::acos(d);
	}

	// ===== Vertex normal: simple average of adjacent face normals =====
	static Vec3 vertexNormalSimple(const Vertex& v) {
		const auto& tris = v.getNeiTriangles();
		if (tris.empty()) return Vec3(0, 0, 0);

		Vec3 sum(0, 0, 0);
		for (const Triangle* tri : tris) {
			if (tri) sum += tri->normal();
		}
		return sum.normalize();
	}

	// ===== Vertex normal: area-weighted average =====
	// N = sum(Ni * Ai) / |sum(Ni * Ai)|
	// Larger faces contribute more
	static Vec3 vertexNormalAreaWeighted(const Vertex& v) {
		const auto& tris = v.getNeiTriangles();
		if (tris.empty()) return Vec3(0, 0, 0);

		Vec3 sum(0, 0, 0);
		for (const Triangle* tri : tris) {
			if (tri) {
				sum += tri->normal() * tri->area;
			}
		}
		return sum.normalize();
	}

	// ===== Vertex normal: angle-weighted average =====
	// N = sum(Ni * theta_i) / |sum(Ni * theta_i)|
	// theta_i = angle at this vertex in triangle i
	static Vec3 vertexNormalAngleWeighted(const Vertex& v) {
		const auto& tris = v.getNeiTriangles();
		if (tris.empty()) return Vec3(0, 0, 0);

		Vec3 sum(0, 0, 0);
		for (const Triangle* tri : tris) {
			if (!tri) continue;
			double angle = angleAtVertex(*tri, v);
			sum += tri->normal() * angle;
		}
		return sum.normalize();
	}

	// ===== Vertex normal: inherited from edge normals (half-edge property) =====
	// Computes edge normals first, then averages them at the vertex.
	// This respects sharp edges: if two faces meet at a sharp edge,
	// the edge normal captures that discontinuity.
	static Vec3 vertexNormalFromEdges(const Vertex& v) {
		const auto& edges = v.getNeiEdges();
		if (edges.empty()) return Vec3(0, 0, 0);

		Vec3 sum(0, 0, 0);
		for (const Edge* edge : edges) {
			if (edge) sum += edgeNormal(*edge);
		}
		return sum.normalize();
	}

	// ===== Helper: angle at vertex in a triangle =====
	static double angleAtVertex(const Triangle& tri, const Vertex& v) {
		const Vertex* v0 = tri.vertex(0);
		const Vertex* v1 = tri.vertex(1);
		const Vertex* v2 = tri.vertex(2);
		if (!v0 || !v1 || !v2) return 0.0;

		Vec3 e1, e2;
		if (&v == v0) {
			e1 = Vec3(v1->x - v0->x, v1->y - v0->y, v1->z - v0->z);
			e2 = Vec3(v2->x - v0->x, v2->y - v0->y, v2->z - v0->z);
		} else if (&v == v1) {
			e1 = Vec3(v0->x - v1->x, v0->y - v1->y, v0->z - v1->z);
			e2 = Vec3(v2->x - v1->x, v2->y - v1->y, v2->z - v1->z);
		} else if (&v == v2) {
			e1 = Vec3(v0->x - v2->x, v0->y - v2->y, v0->z - v2->z);
			e2 = Vec3(v1->x - v2->x, v1->y - v2->y, v1->z - v2->z);
		} else {
			return 0.0;
		}

		double len1 = e1.cachedLength();
		double len2 = e2.cachedLength();
		if (len1 < 1e-15 || len2 < 1e-15) return 0.0;

		double cosAngle = e1.dot(e2) / (len1 * len2);
		cosAngle = (cosAngle < -1.0) ? -1.0 : (cosAngle > 1.0 ? 1.0 : cosAngle);
		return std::acos(cosAngle);
	}

	// ===== Batch computation helpers (call from Mesh) =====

	// Compute normals for all edges in a mesh
	static std::vector<Vec3> computeAllEdgeNormals(
		const std::vector<std::unique_ptr<Edge>>& edges) {
		std::vector<Vec3> normals(edges.size());
		for (std::size_t i = 0; i < edges.size(); ++i) {
			if (edges[i]) {
				normals[i] = edgeNormal(*edges[i]);
			}
		}
		return normals;
	}

	// Vertex normal strategy enum
	enum class VertexNormalMethod {
		Simple,          // Simple average
		AreaWeighted,    // Area-weighted (default in most renderers)
		AngleWeighted,   // Angle-weighted
		FromEdges        // Inherited from edge normals (half-edge)
	};

	// Compute normals for all vertices in a mesh
	static std::vector<Vec3> computeAllVertexNormals(
		const std::vector<std::unique_ptr<Vertex>>& vertices,
		VertexNormalMethod method) {
		std::vector<Vec3> normals(vertices.size());
		for (std::size_t i = 0; i < vertices.size(); ++i) {
			if (!vertices[i]) continue;
			switch (method) {
			case VertexNormalMethod::Simple:
				normals[i] = vertexNormalSimple(*vertices[i]);
				break;
			case VertexNormalMethod::AreaWeighted:
				normals[i] = vertexNormalAreaWeighted(*vertices[i]);
				break;
			case VertexNormalMethod::AngleWeighted:
				normals[i] = vertexNormalAngleWeighted(*vertices[i]);
				break;
			case VertexNormalMethod::FromEdges:
				normals[i] = vertexNormalFromEdges(*vertices[i]);
				break;
			}
		}
		return normals;
	}
};
