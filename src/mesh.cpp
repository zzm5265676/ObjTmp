#include "mesh.hxx"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace {

	using IndexSet = std::unordered_set<int>;

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

	// 报告字段保持头文件中的既有接口。先用集合收集错误，可避免一个对象因
	// 多个不变量同时失败而重复出现；返回前再排序，保证报告结果稳定。
	IndexSet invalidVertices;
	IndexSet invalidTriangles;
	IndexSet degenerateTriangles;
	IndexSet triangleEdgeMismatches;
	IndexSet brokenEdges;

	// Mesh 的邻接关系和索引表保存的是非拥有裸指针。任何解引用之前先通过
	// 所有权集合确认指针属于当前 Mesh，避免校验损坏数据时再次触发未定义行为。
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

	// 1. Vertex 校验：
	//    a) index 必须等于容器下标，确保 README 约定的稳定 ID；
	//    b) 三类邻接对象必须由当前 Mesh 拥有；
	//    c) 邻接顶点应双向，邻接边/面必须真实包含当前顶点。
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
				|| neighbor->getNeiVertics().find(
					const_cast<Vertex*>(vertex)) == neighbor->getNeiVertics().end()) {
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

	// 2. Triangle 校验：
	//    a) index 与容器下标一致，三个顶点属于当前 Mesh；
	//    b) 顶点不能重复，面积必须是有限且不小于 eps；
	//    c) edge(0..2) 必须依次对应 v0->v1、v1->v2、v2->v0；
	//    d) Vertex/Edge 对 Triangle 的反向邻接必须存在。
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

		if (v0->getNeiTriangles().find(const_cast<Triangle*>(triangle))
			== v0->getNeiTriangles().end()
			|| v1->getNeiTriangles().find(const_cast<Triangle*>(triangle))
			== v1->getNeiTriangles().end()
			|| v2->getNeiTriangles().find(const_cast<Triangle*>(triangle))
			== v2->getNeiTriangles().end()) {
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

	// 3. Edge 校验：
	//    a) index、端点所有权及 directed_edge_map_ 必须一致；
	//    b) EdgePair 中的 ab/ba 槽位必须与规范化端点方向一致；
	//    c) opposite 必须属于本 Mesh、方向相反且双向对称；
	//    d) triangles_ 中的面必须真实引用该有向边；
	//    e) 两个端点必须保存该边及彼此的邻接关系。
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

		const UndirectedEdgeIndexKey ukey(from->index, to->index);
		const auto uit = edge_map_.find(ukey);
		if (uit == edge_map_.end()) {
			valid = false;
		}
		else {
			const Edge* expected =
				from->index == ukey.a ? uit->second.ab : uit->second.ba;
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

		if (from->getNeiEdges().find(const_cast<Edge*>(edge))
			== from->getNeiEdges().end()
			|| to->getNeiEdges().find(const_cast<Edge*>(edge))
			== to->getNeiEdges().end()
			|| from->getNeiVertics().find(const_cast<Vertex*>(to))
			== from->getNeiVertics().end()
			|| to->getNeiVertics().find(const_cast<Vertex*>(from))
			== to->getNeiVertics().end()) {
			valid = false;
		}

		if (!valid) brokenEdges.insert(index);
	}

	// 4. 从 map 反向检查容器，捕获 key 错误、悬空指针以及未被 edges_ 拥有的对象。
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

	for (const auto& entry : edge_map_) {
		const UndirectedEdgeIndexKey& key = entry.first;
		const EdgePair& pair = entry.second;

		const auto slotValid = [&](const Edge* edge, bool isAB) {
			if (!edge) return true; // 空槽位代表边界边，不是结构错误。
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

		// 两个方向都存在时必须互为 opposite；仅一个方向存在时是边界边，
		// 已有方向不应再指向其他 Edge。
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
	// 有向边只统计直接存放在自身 triangles_ 中的面。
	return e ? e->triangles().size() : 0;
}

std::size_t Mesh::undirectedTriangleCount(const EdgePair& pair) const {
	// 正反方向理论上不应存放同一个 Triangle。这里使用集合并集而不是简单相加，
	// 即使数据已经损坏，也不会把同一个面错误地统计两次。
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

	// 汇总也可能用于诊断已经损坏的 Mesh，因此不能直接信任 edge_map_ 中的裸指针。
	// 先建立所有权集合；只有属于当前 Mesh 的 Edge/Vertex 才允许解引用。
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

	// 按无向边分类。正常且绕序一致的内部流形边应满足：
	// abCount == 1、baCount == 1、uniqueTotal == 2。
	for (const auto& entry : edge_map_) {
		const UndirectedEdgeIndexKey& key = entry.first;
		const EdgePair& pair = entry.second;

		const bool abOwned = !pair.ab || owns(ownedEdges, pair.ab);
		const bool baOwned = !pair.ba || owns(ownedEdges, pair.ba);
		bool pairBroken = !abOwned || !baOwned;

		// 对不受 Mesh 拥有的指针绝不解引用；其面数按 0 处理，同时标记 pairBroken。
		const std::size_t abCount =
			abOwned ? directedTriangleCount(pair.ab) : 0;
		const std::size_t baCount =
			baOwned ? directedTriangleCount(pair.ba) : 0;
		EdgePair safePair{
			abOwned ? pair.ab : nullptr,
			baOwned ? pair.ba : nullptr
		};
		const std::size_t uniqueTotal = undirectedTriangleCount(safePair);

		// EdgePair 槽位方向必须与规范化 key 一致。检查端点前先检查空指针。
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

		// 两个方向同时存在时必须互为 opposite；单方向存在时必须没有 opposite。
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

			// 两个面都使用同一方向时，拓扑仍是两面边，但面绕序不一致。
			if (abCount != 1 || baCount != 1) {
				++inconsistentOrientationCount;
			}
		}
		else {
			++nonManifoldCount;
		}

		// Summary 不逐条打印正常边，只输出异常边，避免大型网格产生海量日志。
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

	std::cout << "undirected edges: " << edge_map_.size() << '\n';
	std::cout << "empty edges: " << emptyCount << '\n';
	std::cout << "boundary edges: " << boundaryCount << std::endl;
	std::cout << "manifold edges: " << manifoldCount << std::endl;
	std::cout << "non-manifold edges: " << nonManifoldCount << std::endl;
	std::cout << "inconsistent orientation edges: "
		<< inconsistentOrientationCount << std::endl;
	std::cout << "broken edge pairs: " << brokenPairCount << std::endl;
}

//流形和水密检查
//struct MeshCheckReport {
//	int boundaryEdgeCount = 0;
//	int nonManifoldEdgeCount = 0;
//	int inconsistentOrientationEdgeCount = 0;
//	int nonManifoldVertexCount = 0;
//	int isolatedVertexCount = 0;
//	int degenerateTriangleCount = 0;
//};
//第一步：边级检查
void Mesh::checkEdgeManifoldAndBoundary(MeshCheckReport& report) const {
	//遍历每个无向边
	//std::cout << edge_map_.size();
	for (const auto& udeptr : edge_map_) {
		UndirectedEdgeIndexKey udkey = udeptr.first;
		int abCount = udeptr.second.ab ? udeptr.second.ab->triangles().size() : 0;
		int baCount = udeptr.second.ba ? udeptr.second.ba->triangles().size() : 0;

		int totle = abCount + baCount;
		switch (totle) {
		case 1:
			report.boundaryEdgeCount++;
			break;
		case 2:
			if (!(abCount == 1 && baCount == 1))
				report.inconsistentOrientationEdgeCount++;
			break;
		case 3:
			report.nonManifoldEdgeCount++;
		}
	}

}

//第二步：方向一致性检查
void Mesh::checkOrientationConsistency() const {}

//第三步：退化三角形检查
void Mesh::checkDegenerateTriangles(MeshCheckReport& report) const {
	for (const auto& triptr : triangles_) {
		if (triptr->area < 1e-12)
			report.degenerateTriangleCount++;
	}
}

//第四步：点级非流形检查
void Mesh::checkNonManifoldVertices(MeshCheckReport& report) const {
	for (const auto& vtptr : vertices_) {
		const Vertex* v = vtptr.get();

		if (!v) {
			continue;
		}

		const auto& neiTris = v->getNeiTriangles();

		if (neiTris.empty()) {
			report.isolatedVertexCount++;
			continue;
		}

		if (isNonManifoldVertex(v)) {
			report.nonManifoldVertexCount++;
		}
	}

}
//判断两个三角形是否共享一条包含 center 的边
bool Mesh::trianglesShareEdgeAtVertex(const Triangle* t0, const Triangle* t1, const Vertex* center)const {
	if (!t0 || !t1 || !center) {
		return false;
	}

	bool shareCenter = false;
	bool shareAnotherVertex = false;

	for (int i = 0; i < 3; ++i) {
		const Vertex* a = t0->vertex(i);

		for (int j = 0; j < 3; ++j) {
			const Vertex* b = t1->vertex(j);

			if (a == b) {
				if (a == center) {
					shareCenter = true;
				}
				else {
					shareAnotherVertex = true;
				}
			}
		}
	}

	return shareCenter && shareAnotherVertex;
}
//检查单个顶点是否是非流形点
bool Mesh::isNonManifoldVertex(const Vertex* center) const {
	if (!center) {
		return false;
	}

	const auto& triSet = center->getNeiTriangles();

	// 没有三角形，这是孤立点，不在这里判为非流形
	if (triSet.empty()) {
		return false;
	}

	// 只有一个三角形，只有一个局部扇区
	// 是否为边界，由边界边检查负责
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

	std::unordered_set<Triangle*> visited;

	int localComponentCount = 0;

	for (Triangle* start : incidentTris) {
		if (!start) {
			continue;
		}

		if (visited.find(start) != visited.end()) {
			continue;
		}

		++localComponentCount;

		if (localComponentCount > 1) {
			return true;
		}

		std::queue<Triangle*> q;
		q.push(start);
		visited.insert(start);

		while (!q.empty()) {
			Triangle* cur = q.front();
			q.pop();

			for (Triangle* next : incidentTris) {
				if (!next) {
					continue;
				}

				if (visited.find(next) != visited.end()) {
					continue;
				}

				if (trianglesShareEdgeAtVertex(cur, next, center)) {
					visited.insert(next);
					q.push(next);
				}
			}
		}
	}

	return false;
}
//第六步：汇总
MeshCheckReport Mesh::checkManifoldAndWatertight() const {
	MeshCheckReport report;
	checkEdgeManifoldAndBoundary(report);
	return report;
}
