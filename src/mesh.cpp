#include "mesh.hxx"
#include "geometry_utils.hxx"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace {

	using IndexSet = std::unordered_set<int>;

	// 辅助：检查 vector 中是否包含某指针
	template <typename T>
	bool vecContains(const std::vector<T*>& vec, const T* ptr) {
		return std::find(vec.begin(), vec.end(), ptr) != vec.end();
	}

	template <typename T>
	bool owns(const std::unordered_set<const T*>& objects, const T* object) {
		return object != nullptr && objects.find(object) != objects.end();
	}

	bool triangleContainsVertex(const Triangle* triangle, const Vertex* vertex) {
		return triangle && vertex
			&& (triangle->vertex(0) == vertex
				|| triangle->vertex(1) == vertex
				|| triangle->vertex(2) == vertex);
	}

	bool triangleContainsEdge(const Triangle* triangle, const Edge* edge) {
		return triangle && edge
			&& (triangle->edge(0) == edge
				|| triangle->edge(1) == edge
				|| triangle->edge(2) == edge);
	}

	std::vector<int> sortedIndices(const IndexSet& indices) {
		std::vector<int> result(indices.begin(), indices.end());
		std::sort(result.begin(), result.end());
		return result;
	}

} // namespace

MeshValidationReport Mesh::validateBasicTopology(double eps) const {
	MeshValidationReport report;
	eps = std::abs(eps);

	// �����ֶα���ͷ�ļ��еļ��нӿڡ����ü����ռ����󣬿ɱ���һ��������
	// ���������ͬʱʧ�ܶ��ظ����֣�����ǰ�����򣬱�֤�������ȶ���
	IndexSet invalidVertices;
	IndexSet invalidTriangles;
	IndexSet degenerateTriangles;
	IndexSet triangleEdgeMismatches;
	IndexSet brokenEdges;

	// Mesh ���ڽӹ�ϵ��������������Ƿ�ӵ����ָ�롣�κν�����֮ǰ��ͨ��
	// ����Ȩ����ȷ��ָ�����ڵ�ǰ Mesh������У��������ʱ�ٴδ���δ������Ϊ��
	std::unordered_set<const Vertex*> ownedVertices;
	std::unordered_set<const Edge*> ownedEdges;
	std::unordered_set<const Triangle*> ownedTriangles;
	ownedVertices.reserve(vertices_.size());
	ownedEdges.reserve(edges_.size());
	ownedTriangles.reserve(triangles_.size());

	for (const auto& vertex : vertices_) {
		if (vertex) ownedVertices.insert(vertex.get());
	}
	for (const auto& edge : edges_) {
		if (edge) ownedEdges.insert(edge.get());
	}
	for (const auto& triangle : triangles_) {
		if (triangle) ownedTriangles.insert(triangle.get());
	}

	// 1. Vertex У�飺
	//    a) index ������������±꣬ȷ�� README Լ�����ȶ� ID��
	//    b) �����ڽӶ�������ɵ�ǰ Mesh ӵ�У�
	//    c) �ڽӶ���Ӧ˫���ڽӱ�/�������ʵ������ǰ���㡣
	for (std::size_t i = 0; i < vertices_.size(); ++i) {
		const Vertex* vertex = vertices_[i].get();
		const int index = static_cast<int>(i);
		if (!vertex) {
			invalidVertices.insert(index);
			continue;
		}
		if (vertex->index != index) {
			invalidVertices.insert(index);
		}

		for (const Vertex* neighbor : vertex->getNeiVertics()) {
			if (!owns(ownedVertices, neighbor) || neighbor == vertex
				|| !vecContains(neighbor->getNeiVertics(), vertex)) {
				invalidVertices.insert(index);
			}
		}
		for (const Edge* edge : vertex->getNeiEdges()) {
			if (!owns(ownedEdges, edge)
				|| (edge->from() != vertex && edge->to() != vertex)) {
				invalidVertices.insert(index);
			}
		}
		for (const Triangle* triangle : vertex->getNeiTriangles()) {
			if (!owns(ownedTriangles, triangle)
				|| !triangleContainsVertex(triangle, vertex)) {
				invalidVertices.insert(index);
			}
		}
	}

	// 2. Triangle У�飺
	//    a) index �������±�һ�£������������ڵ�ǰ Mesh��
	//    b) ���㲻���ظ�����������������Ҳ�С�� eps��
	//    c) edge(0..2) �������ζ�Ӧ v0->v1��v1->v2��v2->v0��
	//    d) Vertex/Edge �� Triangle �ķ����ڽӱ�����ڡ�
	for (std::size_t i = 0; i < triangles_.size(); ++i) {
		const Triangle* triangle = triangles_[i].get();
		const int index = static_cast<int>(i);
		if (!triangle) {
			invalidTriangles.insert(index);
			continue;
		}
		if (triangle->index != index) {
			invalidTriangles.insert(index);
		}

		Vertex* v0 = triangle->vertex(0);
		Vertex* v1 = triangle->vertex(1);
		Vertex* v2 = triangle->vertex(2);
		if (!owns(ownedVertices, v0)
			|| !owns(ownedVertices, v1)
			|| !owns(ownedVertices, v2)) {
			invalidTriangles.insert(index);
			continue;
		}
		if (v0 == v1 || v1 == v2 || v2 == v0
			|| !std::isfinite(triangle->area)
			|| triangle->area < eps) {
			degenerateTriangles.insert(index);
		}

		if (!vecContains(v0->getNeiTriangles(), triangle)
			|| !vecContains(v1->getNeiTriangles(), triangle)
			|| !vecContains(v2->getNeiTriangles(), triangle)) {
			invalidTriangles.insert(index);
		}

		Edge* e0 = triangle->edge(0);
		Edge* e1 = triangle->edge(1);
		Edge* e2 = triangle->edge(2);
		const bool edgesMatch =
			owns(ownedEdges, e0) && e0->from() == v0 && e0->to() == v1
			&& owns(ownedEdges, e1) && e1->from() == v1 && e1->to() == v2
			&& owns(ownedEdges, e2) && e2->from() == v2 && e2->to() == v0;

		if (!edgesMatch) {
			triangleEdgeMismatches.insert(index);
			continue;
		}

		if (e0->triangles().find(const_cast<Triangle*>(triangle))
			== e0->triangles().end()
			|| e1->triangles().find(const_cast<Triangle*>(triangle))
			== e1->triangles().end()
			|| e2->triangles().find(const_cast<Triangle*>(triangle))
			== e2->triangles().end()) {
			triangleEdgeMismatches.insert(index);
		}
	}

	// 3. Edge У�飺
	//    a) index���˵�����Ȩ�� directed_edge_map_ ����һ�£�
	//    b) EdgePair �е� ab/ba ��λ������淶���˵㷽��һ�£�
	//    c) opposite �������ڱ� Mesh�������෴��˫��Գƣ�
	//    d) triangles_ �е��������ʵ���ø�����ߣ�
	//    e) �����˵���뱣��ñ߼��˴˵��ڽӹ�ϵ��
	for (std::size_t i = 0; i < edges_.size(); ++i) {
		const Edge* edge = edges_[i].get();
		const int index = static_cast<int>(i);
		if (!edge) {
			brokenEdges.insert(index);
			continue;
		}

		bool valid = edge->index == index;
		const Vertex* from = edge->from();
		const Vertex* to = edge->to();
		if (!owns(ownedVertices, from) || !owns(ownedVertices, to) || from == to) {
			brokenEdges.insert(index);
			continue;
		}

		const DirectedEdgeIndexKey dkey(from->index, to->index);
		const auto dit = directed_edge_map_.find(dkey);
		if (dit == directed_edge_map_.end() || dit->second != edge) {
			valid = false;
		}

		// 验证无向边对：检查 opposite 边是否在 directed_edge_map_ 中
		{
			EdgePair pair = getUndirectedEdgePair(from->index, to->index);
			const Edge* expected =
				from->index < to->index ? pair.ab : pair.ba;
			if (expected != edge) valid = false;
		}

		const Edge* opposite = edge->opposite();
		if (opposite
			&& (!owns(ownedEdges, opposite)
				|| opposite == edge
				|| opposite->opposite() != edge
				|| opposite->from() != to
				|| opposite->to() != from)) {
			valid = false;
		}

		for (const Triangle* triangle : edge->triangles()) {
			if (!owns(ownedTriangles, triangle)
				|| !triangleContainsEdge(triangle, edge)) {
				valid = false;
				if (owns(ownedTriangles, triangle)) {
					triangleEdgeMismatches.insert(triangle->index);
				}
			}
		}

		if (!vecContains(from->getNeiEdges(), edge)
			|| !vecContains(to->getNeiEdges(), edge)
			|| !vecContains(from->getNeiVertics(), to)
			|| !vecContains(to->getNeiVertics(), from)) {
			valid = false;
		}

		if (!valid) brokenEdges.insert(index);
	}

	// 4. �� map ���������������� key ��������ָ���Լ�δ�� edges_ ӵ�еĶ���
	for (const auto& entry : directed_edge_map_) {
		const DirectedEdgeIndexKey& key = entry.first;
		const Edge* edge = entry.second;
		if (!owns(ownedEdges, edge)
			|| !owns(ownedVertices, edge->from())
			|| !owns(ownedVertices, edge->to())
			|| edge->from()->index != key.from
			|| edge->to()->index != key.to) {
			brokenEdges.insert(edge ? edge->index : -1);
		}
	}

	// 4b. 验证无向边对的 opposite 关系
	for (const auto& entry : collectUndirectedEdges()) {
		const UndirectedEdgeIndexKey& key = entry.first;
		const EdgePair& pair = entry.second;

		const auto slotValid = [&](const Edge* edge, bool isAB) {
			if (!edge) return true;
			if (!owns(ownedEdges, edge)
				|| !owns(ownedVertices, edge->from())
				|| !owns(ownedVertices, edge->to())) {
				return false;
			}
			return isAB
				? edge->from()->index == key.a && edge->to()->index == key.b
				: edge->from()->index == key.b && edge->to()->index == key.a;
			};

		if (!slotValid(pair.ab, true)) {
			brokenEdges.insert(pair.ab ? pair.ab->index : -1);
		}
		if (!slotValid(pair.ba, false)) {
			brokenEdges.insert(pair.ba ? pair.ba->index : -1);
		}

		if (pair.ab && pair.ba) {
			if (pair.ab->opposite() != pair.ba || pair.ba->opposite() != pair.ab) {
				brokenEdges.insert(pair.ab->index);
				brokenEdges.insert(pair.ba->index);
			}
		}
		else {
			const Edge* existing = pair.ab ? pair.ab : pair.ba;
			if (existing && existing->opposite()) {
				brokenEdges.insert(existing->index);
			}
		}
	}

	report.invalidVertexIndices = sortedIndices(invalidVertices);
	report.invalidTriangleIndices = sortedIndices(invalidTriangles);
	report.degenerateTriangles = sortedIndices(degenerateTriangles);
	report.triangleEdgeMismatch = sortedIndices(triangleEdgeMismatches);
	report.brokenOppositeEdges = sortedIndices(brokenEdges);
	return report;
}

std::size_t Mesh::directedTriangleCount(const Edge* e) const {
	// �����ֻͳ��ֱ�Ӵ�������� triangles_ �е��档
	return e ? e->triangles().size() : 0;
}

std::size_t Mesh::undirectedTriangleCount(const EdgePair& pair) const {
	// �������������ϲ�Ӧ���ͬһ�� Triangle������ʹ�ü��ϲ��������Ǽ���ӣ�
	// ��ʹ�����Ѿ��𻵣�Ҳ�����ͬһ��������ͳ�����Ρ�
	std::unordered_set<const Triangle*> triangles;
	if (pair.ab) {
		triangles.insert(pair.ab->triangles().begin(),
			pair.ab->triangles().end());
	}
	if (pair.ba) {
		triangles.insert(pair.ba->triangles().begin(),
			pair.ba->triangles().end());
	}
	return triangles.size();
}

void Mesh::printEdgeUsageSummary() const {
	std::size_t emptyCount = 0;
	std::size_t boundaryCount = 0;
	std::size_t manifoldCount = 0;
	std::size_t nonManifoldCount = 0;
	std::size_t inconsistentOrientationCount = 0;
	std::size_t brokenPairCount = 0;

	// 先做所有权校验，只引用当前 Mesh 拥有的 Edge/Vertex
	std::unordered_set<const Edge*> ownedEdges;
	std::unordered_set<const Vertex*> ownedVertices;
	ownedEdges.reserve(edges_.size());
	ownedVertices.reserve(vertices_.size());
	for (const auto& edge : edges_) {
		if (edge) ownedEdges.insert(edge.get());
	}
	for (const auto& vertex : vertices_) {
		if (vertex) ownedVertices.insert(vertex.get());
	}

	// 通过 directed_edge_map_ 收集所有无向边
	auto undirectedEdges = collectUndirectedEdges();

	for (const auto& entry : undirectedEdges) {
		const UndirectedEdgeIndexKey& key = entry.first;
		const EdgePair& pair = entry.second;

		const bool abOwned = !pair.ab || owns(ownedEdges, pair.ab);
		const bool baOwned = !pair.ba || owns(ownedEdges, pair.ba);
		bool pairBroken = !abOwned || !baOwned;

		const std::size_t abCount =
			abOwned ? directedTriangleCount(pair.ab) : 0;
		const std::size_t baCount =
			baOwned ? directedTriangleCount(pair.ba) : 0;
		EdgePair safePair{
			abOwned ? pair.ab : nullptr,
			baOwned ? pair.ba : nullptr
		};
		const std::size_t uniqueTotal = undirectedTriangleCount(safePair);

		if (abOwned && pair.ab
			&& (!owns(ownedVertices, pair.ab->from())
				|| !owns(ownedVertices, pair.ab->to())
				|| pair.ab->from()->index != key.a
				|| pair.ab->to()->index != key.b)) {
			pairBroken = true;
		}
		if (baOwned && pair.ba
			&& (!owns(ownedVertices, pair.ba->from())
				|| !owns(ownedVertices, pair.ba->to())
				|| pair.ba->from()->index != key.b
				|| pair.ba->to()->index != key.a)) {
			pairBroken = true;
		}

		if (abOwned && baOwned && pair.ab && pair.ba) {
			if (pair.ab->opposite() != pair.ba || pair.ba->opposite() != pair.ab) {
				pairBroken = true;
			}
		}
		else if (abOwned && baOwned) {
			const Edge* existing = pair.ab ? pair.ab : pair.ba;
			if (existing && existing->opposite()) {
				pairBroken = true;
			}
		}
		if (pairBroken) {
			++brokenPairCount;
		}

		if (uniqueTotal == 0) {
			++emptyCount;
		}
		else if (uniqueTotal == 1) {
			++boundaryCount;
		}
		else if (uniqueTotal == 2) {
			++manifoldCount;
			if (abCount != 1 || baCount != 1) {
				++inconsistentOrientationCount;
			}
		}
		else {
			++nonManifoldCount;
		}

		if (pairBroken
			|| uniqueTotal == 0
			|| uniqueTotal > 2
			|| (uniqueTotal == 2 && (abCount != 1 || baCount != 1))) {
			std::cout << "abnormal edge {" << key.a << ", " << key.b << "}: "
				<< "abCount=" << abCount << ", "
				<< "baCount=" << baCount << ", "
				<< "uniqueTotal=" << uniqueTotal << ", "
				<< "pairBroken=" << (pairBroken ? "true" : "false")
				<< '\n';
		}
	}

	std::cout << "undirected edges: " << undirectedEdges.size() << '\n';
	std::cout << "empty edges: " << emptyCount << '\n';
	std::cout << "boundary edges: " << boundaryCount << std::endl;
	std::cout << "manifold edges: " << manifoldCount << std::endl;
	std::cout << "non-manifold edges: " << nonManifoldCount << std::endl;
	std::cout << "inconsistent orientation edges: "
		<< inconsistentOrientationCount << std::endl;
	std::cout << "broken edge pairs: " << brokenPairCount << std::endl;
}

//���κ�ˮ�ܼ��
//struct MeshCheckReport {
//	int boundaryEdgeCount = 0;
//	int nonManifoldEdgeCount = 0;
//	int inconsistentOrientationEdgeCount = 0;
//	int nonManifoldVertexCount = 0;
//	int isolatedVertexCount = 0;
//	int degenerateTriangleCount = 0;
//};
//��һ�����߼����
void Mesh::checkEdgeManifoldAndBoundary(MeshCheckReport& report) const {
	auto undirectedEdges = collectUndirectedEdges();
	for (const auto& entry : undirectedEdges) {
		const UndirectedEdgeIndexKey& key = entry.first;
		const EdgePair& pair = entry.second;
		int abCount = pair.ab ? static_cast<int>(pair.ab->triangles().size()) : 0;
		int baCount = pair.ba ? static_cast<int>(pair.ba->triangles().size()) : 0;

		int total = abCount + baCount;
		int edgeIdx = pair.ab ? pair.ab->index : (pair.ba ? pair.ba->index : -1);
		if (total == 1) {
			report.boundaryEdgeCount++;
			if (edgeIdx >= 0) report.boundaryEdgeIndices.push_back(edgeIdx);
		} else if (total >= 3) {
			report.nonManifoldEdgeCount++;
			if (edgeIdx >= 0) report.nonManifoldEdgeIndices.push_back(edgeIdx);
		}
		// Note: total==2 orientation check moved to checkOrientationConsistency
	}
}

//�ڶ���������һ���Լ��
void Mesh::checkOrientationConsistency(MeshCheckReport& report) const {
	// Check each triangle: edge[i] direction must match vertex ordering
	// Expected: edge[0] = v0→v1, edge[1] = v1→v2, edge[2] = v2→v0
	std::unordered_set<int> inconsistentEdges;

	for (const auto& triptr : triangles_) {
		const Triangle* tri = triptr.get();
		if (!tri) continue;

		const Vertex* v0 = tri->vertex(0);
		const Vertex* v1 = tri->vertex(1);
		const Vertex* v2 = tri->vertex(2);
		if (!v0 || !v1 || !v2) continue;

		const Edge* e0 = tri->edge(0);
		const Edge* e1 = tri->edge(1);
		const Edge* e2 = tri->edge(2);

		if (e0 && (e0->from() != v0 || e0->to() != v1)) {
			inconsistentEdges.insert(e0->index);
		}
		if (e1 && (e1->from() != v1 || e1->to() != v2)) {
			inconsistentEdges.insert(e1->index);
		}
		if (e2 && (e2->from() != v2 || e2->to() != v0)) {
			inconsistentEdges.insert(e2->index);
		}
	}

	report.inconsistentOrientationEdgeCount = static_cast<int>(inconsistentEdges.size());
	report.inconsistentOrientationEdgeIndices.assign(inconsistentEdges.begin(), inconsistentEdges.end());
	std::sort(report.inconsistentOrientationEdgeIndices.begin(),
		report.inconsistentOrientationEdgeIndices.end());
}

//���������˻������μ��
void Mesh::checkDegenerateTriangles(MeshCheckReport& report) const {
	for (const auto& triptr : triangles_) {
		if (triptr->area < 1e-12) {
			report.degenerateTriangleCount++;
			report.degenerateTriangleIndices.push_back(triptr->index);
		}
	}
}

//���Ĳ����㼶�����μ��
void Mesh::checkNonManifoldVertices(MeshCheckReport& report) const {
	for (const auto& vtptr : vertices_) {
		const Vertex* v = vtptr.get();
		if (!v) continue;

		const auto& neiTris = v->getNeiTriangles();

		if (neiTris.empty()) {
			report.isolatedVertexCount++;
			report.isolatedVertexIndices.push_back(v->index);
			continue;
		}

		if (isNonManifoldVertex(v)) {
			report.nonManifoldVertexCount++;
			report.nonManifoldVertexIndices.push_back(v->index);
		}
	}
}
//��鵥�������Ƿ��Ƿ����ε�
bool Mesh::isNonManifoldVertex(const Vertex* center) const {
	if (!center) {
		return false;
	}

	const auto& triSet = center->getNeiTriangles();

	if (triSet.size() <= 1) {
		return false;
	}

	std::vector<Triangle*> incidentTris;
	incidentTris.reserve(triSet.size());

	for (Triangle* tri : triSet) {
		if (tri) {
			incidentTris.push_back(tri);
		}
	}

	// 建立局部 edge→triangles 映射（用 vertex index pair 做 key）
	// 只关注经过 center 的边
	std::unordered_map<UndirectedEdgeIndexKey, std::vector<Triangle*>, UndirectedEdgeIndexKeyHash> edgeTriMap;

	for (Triangle* tri : incidentTris) {
		for (int i = 0; i < 3; ++i) {
			const Vertex* a = tri->vertex(i);
			const Vertex* b = tri->vertex((i + 1) % 3);
			// 只记录包含 center 的边
			if (a != center && b != center) continue;
			UndirectedEdgeIndexKey key(a->index, b->index);
			edgeTriMap[key].push_back(tri);
		}
	}

	// BFS 遍历连通分量：通过共享边连接三角形
	std::unordered_set<Triangle*> visited;
	int localComponentCount = 0;

	for (Triangle* start : incidentTris) {
		if (visited.find(start) != visited.end()) continue;

		++localComponentCount;
		if (localComponentCount > 1) return true;

		std::queue<Triangle*> q;
		q.push(start);
		visited.insert(start);

		while (!q.empty()) {
			Triangle* cur = q.front();
			q.pop();

			// 遍历 cur 的三条边，找到共享边的邻居三角形
			for (int i = 0; i < 3; ++i) {
				const Vertex* a = cur->vertex(i);
				const Vertex* b = cur->vertex((i + 1) % 3);
				if (a != center && b != center) continue;

				UndirectedEdgeIndexKey key(a->index, b->index);
				auto it = edgeTriMap.find(key);
				if (it == edgeTriMap.end()) continue;

				for (Triangle* neighbor : it->second) {
					if (neighbor && visited.insert(neighbor).second) {
						q.push(neighbor);
					}
				}
			}
		}
	}

	return false;
}

// ===== Self-intersection detection (using geo::) =====

void Mesh::checkSelfIntersection(MeshCheckReport& report) const {
	if (triangles_.size() < 2) return;

	// Compute AABB for each triangle
	std::vector<geo::AABB> aabbs(triangles_.size());
	for (std::size_t i = 0; i < triangles_.size(); ++i) {
		if (triangles_[i]) aabbs[i] = geo::computeTriangleAABB(triangles_[i].get());
	}

	// Broad phase: AABB overlap + Narrow phase: exact intersection
	for (std::size_t i = 0; i < triangles_.size(); ++i) {
		if (!triangles_[i]) continue;
		for (std::size_t j = i + 1; j < triangles_.size(); ++j) {
			if (!triangles_[j]) continue;
			// Skip adjacent triangles (share vertex or edge)
			int sharedVerts = 0;
			for (int vi = 0; vi < 3; ++vi) {
				for (int vj = 0; vj < 3; ++vj) {
					if (triangles_[i]->vertex(vi) == triangles_[j]->vertex(vj)) {
						sharedVerts++;
						break;
					}
				}
			}
			if (sharedVerts >= 2) continue;  // share edge
			if (sharedVerts == 1) {
				double dotN = triangles_[i]->normal().dot(triangles_[j]->normal());
				if (std::abs(dotN) > 0.99) continue;  // nearly coplanar neighbors
			}

			if (!geo::overlap(aabbs[i], aabbs[j])) continue;

			auto result = geo::intersectTriangles(triangles_[i].get(), triangles_[j].get());
			if (result.type != geo::TriTriIntersectionType::None) {
				report.selfIntersectingTriangleCount++;
				report.selfIntersectingPairs.emplace_back(
					static_cast<int>(i), static_cast<int>(j));
			}
		}
	}
}

// Combined check (legacy)
MeshCheckReport Mesh::checkManifoldAndWatertight() const {
	MeshCheckReport report;
	checkEdgeManifoldAndBoundary(report);
	checkOrientationConsistency(report);
	checkDegenerateTriangles(report);
	checkNonManifoldVertices(report);
	return report;
}

// Unified entry: all checks
MeshCheckReport Mesh::validateAll() const {
	MeshCheckReport report;
	checkEdgeManifoldAndBoundary(report);
	checkOrientationConsistency(report);
	checkDegenerateTriangles(report);
	checkNonManifoldVertices(report);
	checkSelfIntersection(report);
	return report;
}
