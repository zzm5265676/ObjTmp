#pragma once
#include "mesh/mesh.hxx"
#include <vector>

namespace mesh_smooth {

// Laplacian smoothing
// Each vertex moves toward the average of its neighbors
// lambda: step size (0, 1], typically 0.1-0.5
// iterations: number of smoothing passes
inline void smoothLaplacian(Mesh& mesh, int iterations = 1, double lambda = 0.5) {
	if (lambda <= 0 || lambda > 1.0) lambda = 0.5;
	if (iterations < 1) iterations = 1;

	for (int iter = 0; iter < iterations; ++iter) {
		// Compute new positions first, then apply (to avoid order dependency)
		std::vector<Vec3> newPositions(mesh.vertexCount());

		for (std::size_t i = 0; i < mesh.vertexCount(); ++i) {
			const Vertex* v = mesh.findByIndex(static_cast<int>(i));
			if (!v) continue;

			const auto& neighbors = v->getNeiVertics();
			if (neighbors.empty()) {
				newPositions[i] = Vec3(v->x, v->y, v->z);
				continue;
			}

			// Average of neighbors
			Vec3 avg(0, 0, 0);
			for (const Vertex* n : neighbors) {
				if (n) {
					avg.x += n->x;
					avg.y += n->y;
					avg.z += n->z;
				}
			}
			avg = avg / static_cast<double>(neighbors.size());

			// Move toward average
			Vec3 current(v->x, v->y, v->z);
			newPositions[i] = current + (avg - current) * lambda;
		}

		// Apply new positions
		for (std::size_t i = 0; i < mesh.vertexCount(); ++i) {
			Vertex* v = mesh.findByIndex(static_cast<int>(i));
			if (!v) continue;
			v->x = newPositions[i].x;
			v->y = newPositions[i].y;
			v->z = newPositions[i].z;
		}
	}

	// Recompute all triangle normals and areas
	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (tri) tri->computeNormalAndArea();
	}
}

// Taubin smoothing (shrink-free Laplacian)
// Alternates between Laplacian (shrink) and anti-Laplacian (expand)
// This preserves volume better than pure Laplacian
// mu: expansion factor, typically -lambda * 1.05 to -lambda * 1.1
inline void smoothTaubin(Mesh& mesh, int iterations = 1, double lambda = 0.5, double mu = -0.53) {
	if (lambda <= 0 || lambda > 1.0) lambda = 0.5;
	if (mu >= 0 || mu < -1.0) mu = -lambda * 1.05;
	if (iterations < 1) iterations = 1;

	for (int iter = 0; iter < iterations; ++iter) {
		// Shrink step (positive lambda)
		smoothLaplacian(mesh, 1, lambda);
		// Expand step (negative mu)
		smoothLaplacian(mesh, 1, mu);
	}
}

} // namespace mesh_smooth
