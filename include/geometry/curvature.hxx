#pragma once
#include "mesh/mesh.hxx"
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mesh_curvature {

struct CurvatureResult {
	std::vector<double> gaussian;   // K = k1 * k2
	std::vector<double> mean;      // H = (k1 + k2) / 2
	std::vector<double> k1;        // principal curvature 1
	std::vector<double> k2;        // principal curvature 2
};

// Compute per-vertex curvature using angle deficit and Laplace-Beltrami
inline CurvatureResult computeCurvature(const Mesh& mesh) {
	int n = static_cast<int>(mesh.vertexCount());
	CurvatureResult result;
	result.gaussian.resize(n, 0.0);
	result.mean.resize(n, 0.0);
	result.k1.resize(n, 0.0);
	result.k2.resize(n, 0.0);

	for (int i = 0; i < n; ++i) {
		const Vertex* v = mesh.findByIndex(i);
		if (!v) continue;

		const auto& neiTris = v->getNeiTriangles();
		const auto& neiEdges = v->getNeiEdges();

		if (neiTris.empty()) continue;

		// === Gaussian curvature: angle deficit ===
		// K = 2*pi - sum(angles at v in adjacent triangles)
		double angleSum = 0.0;
		double areaSum = 0.0;

		for (const Triangle* tri : neiTris) {
			if (!tri) continue;

			// Find angle at v in this triangle
			double angle = NormalComputer::angleAtVertex(*tri, *v);
			angleSum += angle;

			// Mixed area for mean curvature
			areaSum += tri->area;
		}

		// Gaussian curvature (angle deficit)
		result.gaussian[i] = (2.0 * M_PI - angleSum) / (areaSum / 3.0);

		// === Mean curvature: Laplace-Beltrami ===
		// H = |sum over edges of (cot(alpha) + cot(beta)) * (vj - vi)| / (4 * area)
		Vec3 laplace(0, 0, 0);
		double totalArea = 0.0;

		for (const Edge* edge : neiEdges) {
			if (!edge) continue;

			// Get the two vertices of the edge
			const Vertex* v0 = edge->from();
			const Vertex* v1 = edge->to();
			const Vertex* other = (v0 == v) ? v1 : v0;
			if (!other) continue;

			// Find the two triangles sharing this edge
			const Edge* opp = edge->opposite();
			double cotSum = 0.0;

			// Cotangent from triangles on this edge
			auto computeCot = [](const Triangle* tri, const Vertex* apex) -> double {
				if (!tri) return 0.0;
				// Find the angle at apex
				double angle = NormalComputer::angleAtVertex(*tri, *apex);
				if (std::abs(std::sin(angle)) < 1e-12) return 0.0;
				return std::cos(angle) / std::sin(angle);
			};

			// Triangles on this directed edge
			for (const Triangle* tri : edge->triangles()) {
				// Find the vertex opposite to this edge in the triangle
				for (int j = 0; j < 3; ++j) {
					const Vertex* apex = tri->vertex(j);
					if (apex != v0 && apex != v1) {
						cotSum += computeCot(tri, apex);
						break;
					}
				}
			}

			// Triangles on the opposite edge
			if (opp) {
				for (const Triangle* tri : opp->triangles()) {
					for (int j = 0; j < 3; ++j) {
						const Vertex* apex = tri->vertex(j);
						if (apex != v0 && apex != v1) {
							cotSum += computeCot(tri, apex);
							break;
						}
					}
				}
			}

			Vec3 diff(other->x - v->x, other->y - v->y, other->z - v->z);
			laplace = laplace + diff * cotSum;
			totalArea += areaSum;
		}

		if (totalArea > 1e-12) {
			Vec3 meanVec = laplace / (2.0 * totalArea);
			result.mean[i] = meanVec.cachedLength() * 0.5;
		}

		// Principal curvatures from K and H
		double H = result.mean[i];
		double K = result.gaussian[i];
		double disc = H * H - K;
		if (disc < 0) disc = 0;
		double sqrtDisc = std::sqrt(disc);
		result.k1[i] = H + sqrtDisc;
		result.k2[i] = H - sqrtDisc;
	}

	return result;
}

} // namespace mesh_curvature
