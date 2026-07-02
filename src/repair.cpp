#include "repair.hxx"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <queue>

namespace mesh_repair {

// Spatial hash for vertex merging
struct SpatialHash {
	double eps;
	std::unordered_map<int64_t, std::vector<int>> buckets;

	SpatialHash(double eps) : eps(eps) {}

	int64_t hash(double x, double y, double z) const {
		int ix = static_cast<int>((std::floor)(x / eps));
		int iy = static_cast<int>((std::floor)(y / eps));
		int iz = static_cast<int>((std::floor)(z / eps));
		// Simple hash combine
		int64_t h = ix * 73856093LL;
		h ^= iy * 19349663LL;
		h ^= iz * 83492791LL;
		return h;
	}

	void insert(int idx, double x, double y, double z) {
		buckets[hash(x, y, z)].push_back(idx);
	}

	const std::vector<int>* find(double x, double y, double z) const {
		auto it = buckets.find(hash(x, y, z));
		return (it != buckets.end()) ? &it->second : nullptr;
	}
};

int mergeCloseVertices(Mesh& mesh, double eps) {
	int merged = 0;
	int n = static_cast<int>(mesh.vertexCount());
	if (n == 0) return 0;

	// Build spatial hash
	SpatialHash sh(eps);
	for (int i = 0; i < n; ++i) {
		const Vertex* v = mesh.findByIndex(i);
		if (v) sh.insert(i, v->x, v->y, v->z);
	}

	// Find merge groups
	std::vector<int> parent(n);
	for (int i = 0; i < n; ++i) parent[i] = i;

	auto find = [&](int x) -> int {
		while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
		return x;
	};
	auto unite = [&](int a, int b) {
		a = find(a); b = find(b);
		if (a != b) parent[b] = a;
	};

	for (int i = 0; i < n; ++i) {
		const Vertex* vi = mesh.findByIndex(i);
		if (!vi) continue;
		// Check 27 neighboring cells
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dy = -1; dy <= 1; ++dy) {
				for (int dz = -1; dz <= 1; ++dz) {
					double nx = vi->x + dx * eps;
					double ny = vi->y + dy * eps;
					double nz = vi->z + dz * eps;
					const auto* bucket = sh.find(nx, ny, nz);
					if (!bucket) continue;
					for (int j : *bucket) {
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

	// Count actual merges
	std::unordered_map<int, int> groupToNew;
	for (int i = 0; i < n; ++i) {
		int root = find(i);
		if (root != i) merged++;
	}

	// If no merges needed, return early
	if (merged == 0) return 0;

	// Rebuild mesh with merged vertices
	// This is a destructive operation - we need to rebuild everything
	Mesh newMesh;

	// Create new vertex mapping
	std::vector<int> oldToNew(n, -1);
	for (int i = 0; i < n; ++i) {
		int root = find(i);
		if (groupToNew.find(root) == groupToNew.end()) {
			const Vertex* v = mesh.findByIndex(root);
			if (v) {
				groupToNew[root] = newMesh.addVertex(v->x, v->y, v->z)->index;
			}
		}
		oldToNew[i] = groupToNew[root];
	}

	// Rebuild triangles
	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;
		int v0 = oldToNew[tri->vertex(0)->index];
		int v1 = oldToNew[tri->vertex(1)->index];
		int v2 = oldToNew[tri->vertex(2)->index];
		// Skip degenerate triangles (duplicate vertices)
		if (v0 == v1 || v1 == v2 || v2 == v0) continue;
		newMesh.addTriangle(v0, v1, v2);
	}

	mesh = std::move(newMesh);
	return merged;
}

int removeDegenerateTriangles(Mesh& mesh, double eps) {
	int removed = 0;
	// Mark degenerate triangles
	std::vector<bool> toRemove(mesh.triangleCount(), false);
	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri || tri->area < eps) toRemove[i] = true;
	}

	// Count
	for (bool b : toRemove) if (b) removed++;
	if (removed == 0) return 0;

	// Rebuild mesh without degenerate triangles
	Mesh newMesh;
	std::vector<int> oldToNew(mesh.vertexCount(), -1);

	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		if (toRemove[i]) continue;
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;

		// Add vertices if not already added
		int v[3];
		for (int j = 0; j < 3; ++j) {
			int oi = tri->vertex(j)->index;
			if (oldToNew[oi] < 0) {
				oldToNew[oi] = newMesh.addVertex(
					tri->vertex(j)->x, tri->vertex(j)->y, tri->vertex(j)->z)->index;
			}
			v[j] = oldToNew[oi];
		}
		newMesh.addTriangle(v[0], v[1], v[2]);
	}

	mesh = std::move(newMesh);
	return removed;
}

int removeDuplicateTriangles(Mesh& mesh) {
	int removed = 0;
	std::unordered_set<uint64_t> seen;
	std::vector<bool> toKeep(mesh.triangleCount(), true);

	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;

		// Create canonical key (sorted vertex indices)
		int v[3] = { tri->vertex(0)->index, tri->vertex(1)->index, tri->vertex(2)->index };
		std::sort(v, v + 3);
		uint64_t key = (uint64_t(v[0]) << 40) | (uint64_t(v[1]) << 20) | uint64_t(v[2]);

		if (seen.count(key)) {
			toKeep[i] = false;
			removed++;
		} else {
			seen.insert(key);
		}
	}

	if (removed == 0) return 0;

	// Rebuild
	Mesh newMesh;
	std::vector<int> oldToNew(mesh.vertexCount(), -1);

	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		if (!toKeep[i]) continue;
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;

		int v[3];
		for (int j = 0; j < 3; ++j) {
			int oi = tri->vertex(j)->index;
			if (oldToNew[oi] < 0) {
				oldToNew[oi] = newMesh.addVertex(
					tri->vertex(j)->x, tri->vertex(j)->y, tri->vertex(j)->z)->index;
			}
			v[j] = oldToNew[oi];
		}
		newMesh.addTriangle(v[0], v[1], v[2]);
	}

	mesh = std::move(newMesh);
	return removed;
}

int removeIsolatedVertices(Mesh& mesh) {
	// Count vertices used by triangles
	std::vector<bool> used(mesh.vertexCount(), false);
	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;
		for (int j = 0; j < 3; ++j) used[tri->vertex(j)->index] = true;
	}

	int removed = 0;
	for (bool b : used) if (!b) removed++;
	if (removed == 0) return 0;

	// Rebuild
	Mesh newMesh;
	std::vector<int> oldToNew(mesh.vertexCount(), -1);

	for (int i = 0; i < static_cast<int>(mesh.vertexCount()); ++i) {
		if (!used[i]) continue;
		const Vertex* v = mesh.findByIndex(i);
		if (v) oldToNew[i] = newMesh.addVertex(v->x, v->y, v->z)->index;
	}

	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;
		newMesh.addTriangle(
			oldToNew[tri->vertex(0)->index],
			oldToNew[tri->vertex(1)->index],
			oldToNew[tri->vertex(2)->index]);
	}

	mesh = std::move(newMesh);
	return removed;
}

int fixOrientation(Mesh& mesh) {
	int flipped = 0;
	int n = static_cast<int>(mesh.triangleCount());
	if (n == 0) return 0;

	// Build adjacency: edge -> triangles
	std::unordered_map<uint64_t, std::vector<int>> edgeTris;
	for (int i = 0; i < n; ++i) {
		const Triangle* tri = mesh.triangle(i);
		if (!tri) continue;
		for (int j = 0; j < 3; ++j) {
			int v0 = tri->vertex(j)->index;
			int v1 = tri->vertex((j + 1) % 3)->index;
			uint64_t key = (uint64_t((std::min)(v0, v1)) << 32) | uint64_t((std::max)(v0, v1));
			edgeTris[key].push_back(i);
		}
	}

	// BFS orientation propagation
	std::vector<bool> visited(n, false);
	std::vector<bool> flippedTri(n, false);

	for (int start = 0; start < n; ++start) {
		if (visited[start]) continue;
		visited[start] = true;

		std::queue<int> q;
		q.push(start);

		while (!q.empty()) {
			int cur = q.front(); q.pop();
			const Triangle* triCur = mesh.triangle(cur);
			if (!triCur) continue;

			for (int j = 0; j < 3; ++j) {
				int v0 = triCur->vertex(j)->index;
				int v1 = triCur->vertex((j + 1) % 3)->index;
				uint64_t key = (uint64_t((std::min)(v0, v1)) << 32) | uint64_t((std::max)(v0, v1));
				auto it = edgeTris.find(key);
				if (it == edgeTris.end()) continue;

				for (int neighbor : it->second) {
					if (neighbor == cur || visited[neighbor]) continue;
					visited[neighbor] = true;

					// Check if neighbor has opposite orientation
					const Triangle* triNei = mesh.triangle(neighbor);
					if (!triNei) continue;

					// Find the shared edge direction in neighbor
					bool sameDir = false;
					for (int k = 0; k < 3; ++k) {
						int nv0 = triNei->vertex(k)->index;
						int nv1 = triNei->vertex((k + 1) % 3)->index;
						if (nv0 == v0 && nv1 == v1) { sameDir = true; break; }
					}

					// If same direction, neighbor needs to be flipped
					if (sameDir) {
						flippedTri[neighbor] = !flippedTri[cur];
					} else {
						flippedTri[neighbor] = flippedTri[cur];
					}

					q.push(neighbor);
				}
			}
		}
	}

	// Apply flips
	for (int i = 0; i < n; ++i) {
		if (flippedTri[i]) flipped++;
	}

	if (flipped == 0) return 0;

	// Rebuild with flipped triangles
	Mesh newMesh;
	std::vector<int> oldToNew(mesh.vertexCount(), -1);

	for (int i = 0; i < static_cast<int>(mesh.vertexCount()); ++i) {
		const Vertex* v = mesh.findByIndex(i);
		if (v) oldToNew[i] = newMesh.addVertex(v->x, v->y, v->z)->index;
	}

	for (int i = 0; i < n; ++i) {
		const Triangle* tri = mesh.triangle(i);
		if (!tri) continue;
		int v0 = oldToNew[tri->vertex(0)->index];
		int v1 = oldToNew[tri->vertex(1)->index];
		int v2 = oldToNew[tri->vertex(2)->index];
		if (flippedTri[i]) {
			newMesh.addTriangle(v0, v2, v1);  // Swap winding
		} else {
			newMesh.addTriangle(v0, v1, v2);
		}
	}

	mesh = std::move(newMesh);
	return flipped;
}

int fillHoles(Mesh& mesh) {
	int filled = 0;

	// Find boundary edges (edges with only 1 triangle)
	// Use the directed_edge_map_ approach
	std::unordered_map<uint64_t, int> edgeCount;
	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;
		for (int j = 0; j < 3; ++j) {
			int v0 = tri->vertex(j)->index;
			int v1 = tri->vertex((j + 1) % 3)->index;
			uint64_t key = (uint64_t(v0) << 32) | uint64_t(v1);
			edgeCount[key]++;
		}
	}

	// Collect boundary edges (count == 1)
	std::unordered_map<int, std::vector<int>> boundaryAdj;
	for (auto& [key, count] : edgeCount) {
		if (count == 1) {
			int v0 = int(key >> 32);
			int v1 = int(key & 0xFFFFFFFF);
			boundaryAdj[v0].push_back(v1);
		}
	}

	// Trace boundary loops
	std::unordered_set<int> visited;
	for (auto& [start, neighbors] : boundaryAdj) {
		if (visited.count(start)) continue;

		// Trace one loop
		std::vector<int> loop;
		int cur = start;
		do {
			loop.push_back(cur);
			visited.insert(cur);

			auto it = boundaryAdj.find(cur);
			if (it == boundaryAdj.end()) break;

			bool found = false;
			for (int next : it->second) {
				if (!visited.count(next) || (next == start && loop.size() >= 3)) {
					cur = next;
					found = true;
					break;
				}
			}
			if (!found) break;
		} while (cur != start && loop.size() < 10000);

		// Close the loop
		if (cur == start && loop.size() >= 3) {
			// Fan triangulation from first vertex
			for (std::size_t i = 1; i + 1 < loop.size(); ++i) {
				mesh.addTriangle(loop[0], loop[i], loop[i + 1]);
				filled++;
			}
		}
	}

	return filled;
}

} // namespace mesh_repair
