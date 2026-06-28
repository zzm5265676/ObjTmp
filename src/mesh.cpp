#include "mesh.hxx"
class Triangle;
class Vertex;
class Edge;
MeshValidationReport Mesh::validateBasicTopology(double eps) const {
	MeshValidationReport report;
	// 1. 检查 Vertex index 是否和 vertices_ 下标一致
	for (std::size_t i = 0; i < vertices_.size(); i++) {
		auto verpt = vertices_[i].get();
		if (!verpt || verpt->index != i) {
			report.invalidVertexIndices.push_back(static_cast<int>(i));
		}
	}
	// 2. 检查 Triangle 的顶点、面积和边是否一致
	for (const auto& tript : triangles_) {
		const Triangle* tri = tript.get();
		if (!tri) continue;

		Vertex* v0 = tri->vertex(0);
		Vertex* v1 = tri->vertex(1);
		Vertex* v2 = tri->vertex(2);

		if (!v0 || !v1 || !v2) {
			report.invalidTriangleIndices.push_back(tri->index);
			continue;
		}
		if (v0 == v1 || v1 == v2 || v2 == v0 || tri->area < eps) {
			report.degenerateTriangles.push_back(tri->index);
			continue;
		}

		Edge* e0 = tri->edge(0);
		Edge* e1 = tri->edge(1);
		Edge* e2 = tri->edge(2);

		bool edgeOk = true;

		if (!e0 || e0->from() != v0 || e0->to() != v1) {
			edgeOk = false;
		}

		if (!e1 || e1->from() != v1 || e1->to() != v2) {
			edgeOk = false;
		}

		if (!e2 || e2->from() != v2 || e2->to() != v0) {
			edgeOk = false;
		}

		if (!edgeOk) {
			report.triangleEdgeMismatch.push_back(tri->index);
		}

	}
	// 3. 检查 opposite 是否对称
	for (const auto& edgept : edges_) {
		const Edge* edg = edgept.get();
		if (!edg) continue;

		Edge* oppedg = edg->opposite();

		if (oppedg) {
			if (oppedg->opposite() != edg) {
				report.brokenOppositeEdges.push_back(edg->index);
				continue;
			}
			if (edg->from() != oppedg->to() || edg->to() != oppedg->from()) {
				report.brokenOppositeEdges.push_back(edg->index);
				continue;
			}
		}
	}

	return report;
}

std::size_t Mesh::directedTriangleCount(const Edge* e) const {
	if (!e) {
		return 0;
	}

	return e->triangles().size();
}

std::size_t Mesh::undirectedTriangleCount(const EdgePair& pair) const {
	std::size_t count = 0;

	if (pair.ab) {
		count += pair.ab->triangles().size();
	}

	if (pair.ba) {
		count += pair.ba->triangles().size();
	}

	return count;
}

void Mesh::printEdgeUsageSummary() const {
	std::size_t boundaryCount = 0;
	std::size_t manifoldCount = 0;
	std::size_t nonManifoldCount = 0;
	std::size_t inconsistentOrientationCount = 0;

	for (const auto& kv : edge_map_) {
		const UndirectedEdgeIndexKey& key = kv.first;
		const EdgePair& pair = kv.second;

		std::size_t abCount = pair.ab ? pair.ab->triangles().size() : 0;
		std::size_t baCount = pair.ba ? pair.ba->triangles().size() : 0;

		std::size_t total = abCount + baCount;

		if (total == 1) {
			++boundaryCount;
		}
		else if (total == 2) {
			++manifoldCount;

			// 正常方向一致的两个相邻面，应该是一正一反
			if (!(abCount == 1 && baCount == 1)) {
				++inconsistentOrientationCount;
			}
		}
		else if (total > 2) {
			++nonManifoldCount;
		}

		std::cout << "edge {" << key.a << ", " << key.b << "} "
			<< "abCount = " << abCount << ", "
			<< "baCount = " << baCount << ", "
			<< "total = " << total << std::endl;
	}

	std::cout << "boundary edges: " << boundaryCount << std::endl;
	std::cout << "manifold edges: " << manifoldCount << std::endl;
	std::cout << "non-manifold edges: " << nonManifoldCount << std::endl;
	std::cout << "inconsistent orientation edges: "
		<< inconsistentOrientationCount << std::endl;
}