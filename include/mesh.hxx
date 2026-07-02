/*****************************************************************//**
 * \file   mesh.hxx
 * \brief  Mesh ��Ψһ�����ߡ�
 *         vertices_ / edges_ / triangles_ �����������ڣ�
 *         map ֻ���������������ͷš�
 * \author zzm
 * \date   June 2026
 *********************************************************************/
#pragma once 
#include "point.hxx"
#include "triangle.hxx"
#include "edge.hxx"
#include "mat4.hxx"
#include "quat.hxx"
#include "normals.hxx"
#include "geometry_utils.hxx"
#include <memory>
#include <unordered_map>
#include <vector>
#include <functional>
#include <fstream>
#include <sstream>
#include <queue>
#include <stdexcept>
#include <string>
#include <iomanip>
#include <utility>
#include <algorithm>
inline int parseObjVertexIndex(const std::string& token) {
	std::size_t pos = token.find('/');

	std::string indexStr;

	if (pos == std::string::npos) {
		indexStr = token;
	}
	else {
		indexStr = token.substr(0, pos);
	}

	return std::stoi(indexStr);
}
//�����equal��hash
struct DirectedEdgeIndexKey {
	int from = -1;
	int to = -1;
	DirectedEdgeIndexKey() = default;
	DirectedEdgeIndexKey(int f, int t) : from(f), to(t) {}
	bool operator==(const DirectedEdgeIndexKey& other) const noexcept {
		return from == other.from && to == other.to;
	}
};

struct DirectedEdgeIndexKeyHash {
	std::size_t operator()(const DirectedEdgeIndexKey& key) const noexcept {
		std::size_t h0 = std::hash<int>{}(key.from);
		std::size_t h1 = std::hash<int>{}(key.to);
		return h0 ^ (h1 + 0x9e3779b9 + (h0 << 6) + (h0 >> 2));
	}
};
//�����equal��hash
struct UndirectedEdgeIndexKey {
	int a = -1;
	int b = -1;
	UndirectedEdgeIndexKey() = default;
	UndirectedEdgeIndexKey(int i0, int i1) {
		if (i0 <= i1) { a = i0;	b = i1; }
		else { a = i1; b = i0; }
	}
	bool operator==(const UndirectedEdgeIndexKey& other) const noexcept {
		return a == other.a && b == other.b;
	}
};

struct UndirectedEdgeIndexKeyHash {
	std::size_t operator()(const UndirectedEdgeIndexKey& key) const noexcept {
		std::size_t h0 = std::hash<int>{}(key.a);
		std::size_t h1 = std::hash<int>{}(key.b);
		return h0 ^ (h1 + 0x9e3779b9 + (h0 << 6) + (h0 >> 2));
	}
};

struct EdgePair {
	Edge* ab = nullptr;
	Edge* ba = nullptr;
};

//�����鱨��
//���������� Mesh ��û������ì��
struct MeshValidationReport {
	std::vector<int> invalidVertexIndices;
	std::vector<int> invalidTriangleIndices;
	std::vector<int> degenerateTriangles;
	std::vector<int> triangleEdgeMismatch;
	std::vector<int> brokenOppositeEdges;

	bool ok() const {
		return invalidVertexIndices.empty()
			&& invalidTriangleIndices.empty()
			&& degenerateTriangles.empty()
			&& triangleEdgeMismatch.empty()
			&& brokenOppositeEdges.empty();
	}
};
/**
 *  �ж���
 *  �з����αߣ�
 *  �з����ε㣿
 *  �淽��һ�£�
 *  ���˻������Σ�.
 */
struct MeshCheckReport {
	// Counts
	int boundaryEdgeCount = 0;
	int nonManifoldEdgeCount = 0;
	int inconsistentOrientationEdgeCount = 0;
	int nonManifoldVertexCount = 0;
	int isolatedVertexCount = 0;
	int degenerateTriangleCount = 0;
	int selfIntersectingTriangleCount = 0;

	// Indices of problematic elements
	std::vector<int> boundaryEdgeIndices;
	std::vector<int> nonManifoldEdgeIndices;
	std::vector<int> inconsistentOrientationEdgeIndices;
	std::vector<int> nonManifoldVertexIndices;
	std::vector<int> isolatedVertexIndices;
	std::vector<int> degenerateTriangleIndices;
	std::vector<std::pair<int,int>> selfIntersectingPairs;

	// Derived flags
	bool isWatertight() const { return boundaryEdgeCount == 0; }
	bool isManifold() const {
		return nonManifoldEdgeCount == 0 && nonManifoldVertexCount == 0;
	}
	bool isOriented() const { return inconsistentOrientationEdgeCount == 0; }
	bool isDegenerateFree() const { return degenerateTriangleCount == 0; }
	bool hasSelfIntersection() const { return selfIntersectingTriangleCount > 0; }

	bool ok() const {
		return isWatertight() && isManifold() && isOriented()
			&& isDegenerateFree() && !hasSelfIntersection();
	}
};
class Mesh {
private:
	std::vector<std::unique_ptr<Vertex>> vertices_;
	std::vector<std::unique_ptr<Edge>> edges_;
	std::vector<std::unique_ptr<Triangle>> triangles_;
	std::unordered_map<DirectedEdgeIndexKey, Edge*, DirectedEdgeIndexKeyHash> directed_edge_map_;

	// OBJ 解析时存储法线和纹理坐标
	std::vector<std::array<double, 3>> normals_;
	std::vector<std::array<double, 2>> texCoords_;
	std::vector<std::string> parseErrors_;


public:
	Mesh(const Mesh&) = delete;
	Mesh& operator=(const Mesh&) = delete;

	Mesh(Mesh&&) noexcept = default;
	Mesh& operator=(Mesh&&) noexcept = default;

	Mesh() = default;
	// OBJ 文件加载
	Mesh(const std::string& filePath) {
		std::ifstream input(filePath);

		if (!input.is_open()) {
			throw std::runtime_error("Failed to open obj file: " + filePath);
		}

		int lineNumber = 0;
		std::string line;

		while (std::getline(input, line)) {
			++lineNumber;
			if (line.empty()) continue;
			if (line[0] == '#') continue;

			std::istringstream iss(line);
			std::string type;
			iss >> type;

			if (type == "v") {
				double x, y, z;
				if (!(iss >> x >> y >> z)) {
					parseErrors_.push_back("Line " + std::to_string(lineNumber) + ": invalid vertex");
					continue;
				}
				addVertex(x, y, z);
			}
			else if (type == "vn") {
				double nx, ny, nz;
				if (!(iss >> nx >> ny >> nz)) {
					parseErrors_.push_back("Line " + std::to_string(lineNumber) + ": invalid normal");
					continue;
				}
				normals_.push_back({nx, ny, nz});
			}
			else if (type == "vt") {
				double u, v;
				if (!(iss >> u >> v)) {
					parseErrors_.push_back("Line " + std::to_string(lineNumber) + ": invalid texcoord");
					continue;
				}
				texCoords_.push_back({u, v});
			}
			else if (type == "f") {
				std::vector<int> indices;
				std::string token;

				while (iss >> token) {
					try {
						int objIndex = parseObjVertexIndex(token);
						indices.push_back(objIndex - 1);
					}
					catch (const std::exception& e) {
						parseErrors_.push_back("Line " + std::to_string(lineNumber)
							+ ": invalid face token '" + token + "': " + e.what());
					}
				}

				if (indices.size() < 3) {
					parseErrors_.push_back("Line " + std::to_string(lineNumber)
						+ ": face has fewer than 3 vertices");
					continue;
				}

				// 检查索引范围
				bool validRange = true;
				for (int idx : indices) {
					if (idx < 0 || idx >= static_cast<int>(vertices_.size())) {
						parseErrors_.push_back("Line " + std::to_string(lineNumber)
							+ ": vertex index " + std::to_string(idx + 1) + " out of range");
						validRange = false;
						break;
					}
				}
				if (!validRange) continue;

				// 三角形：直接添加
				if (indices.size() == 3) {
					addTriangle(indices[0], indices[1], indices[2]);
				}
				else {
					// 多边形：扇形三角化（适用于凸多边形）
					for (std::size_t i = 1; i + 1 < indices.size(); ++i) {
						addTriangle(indices[0], indices[i], indices[i + 1]);
					}
				}
			}
			// 忽略其他行（o, g, s, mtllib, usemtl 等）
		}
	}

	// 获取解析错误
	const std::vector<std::string>& parseErrors() const noexcept {
		return parseErrors_;
	}

	// 获取法线数据
	const std::vector<std::array<double, 3>>& normals() const noexcept {
		return normals_;
	}

	// 获取纹理坐标数据
	const std::vector<std::array<double, 2>>& texCoords() const noexcept {
		return texCoords_;
	}
	// �����vertices_
	Vertex* addVertex(double x, double y, double z) {
		int idx = static_cast<int>(vertices_.size());
		vertices_.push_back(std::make_unique<Vertex>(idx, x, y, z));
		return vertices_.back().get();
	}
	// ͨ���������ص�
	Vertex* findByIndex(int idx) {
		if (idx < 0 || idx >= static_cast<int>(vertices_.size())) {
			return nullptr;
		}
		return vertices_[idx].get();
	}
	const Vertex* findByIndex(int idx) const {
		if (idx < 0 || idx >= static_cast<int>(vertices_.size())) {
			return nullptr;
		}
		return vertices_[idx].get();
	}
	// 通过两个顶点索引获取无向边对（从 directed_edge_map_ 中查找）
	EdgePair getUndirectedEdgePair(int i0, int i1) const {
		EdgePair pair;
		auto it_ab = directed_edge_map_.find(DirectedEdgeIndexKey(i0, i1));
		if (it_ab != directed_edge_map_.end()) pair.ab = it_ab->second;
		auto it_ba = directed_edge_map_.find(DirectedEdgeIndexKey(i1, i0));
		if (it_ba != directed_edge_map_.end()) pair.ba = it_ba->second;
		return pair;
	}

	// 收集所有唯一的无向边对（用于遍历无向边）
	std::vector<std::pair<UndirectedEdgeIndexKey, EdgePair>> collectUndirectedEdges() const {
		std::unordered_set<UndirectedEdgeIndexKey, UndirectedEdgeIndexKeyHash> seen;
		std::vector<std::pair<UndirectedEdgeIndexKey, EdgePair>> result;
		for (const auto& entry : directed_edge_map_) {
			const Edge* e = entry.second;
			if (!e || !e->from() || !e->to()) continue;
			UndirectedEdgeIndexKey ukey(e->from()->index, e->to()->index);
			if (seen.insert(ukey).second) {
				result.emplace_back(ukey, getUndirectedEdgePair(ukey.a, ukey.b));
			}
		}
		return result;
	}

	//ͨ����Ѱ�ұ�
	//�ҵ����أ�û�ҵ����벢����
	Edge* findOrAddEdge(Vertex* v0, Vertex* v1) {
		if (!v0 || !v1) { return nullptr; }
		if (v0 == v1) { return nullptr; }

		// ���������
		// ����������
		// �������������
		DirectedEdgeIndexKey dkey(v0->index, v1->index);
		// ���Ҳ�����
		auto dit = directed_edge_map_.find(dkey);
		if (dit != directed_edge_map_.end()) {
			return dit->second;
		}
		// û�����죬׼������
		int idx = static_cast<int>(edges_.size());
		auto edge = std::make_unique<Edge>(idx, v0, v1);
		Edge* raw = edge.get();





		// ���������
		// ����ķ����
		// ���췴�������
		DirectedEdgeIndexKey reverse_key(v1->index, v0->index);
		// ���Ҳ�����
		auto rit = directed_edge_map_.find(reverse_key);
		if (rit != directed_edge_map_.end()) {
			Edge* opposite = rit->second;
			// ������ Edge ��֧�� opposite�����Դ�������
			raw->setOpposite(opposite);
			//opposite->setOpposite(raw);
		}

		// ����ߣ�Ϊ���������ߣ�����edges_
		edges_.push_back(std::move(edge));
		// ����ߣ�Ϊ���������ߣ�����directed_edge_map_
		directed_edge_map_.emplace(dkey, raw);

		v0->addNeiVertex(v1);
		v1->addNeiVertex(v0);

		v0->addNeiEdge(raw);
		v1->addNeiEdge(raw);

		return raw;
	}
	Triangle* addTriangle(int firstIdx, int secondIdx, int thirdIdx) {
		// ��ȡ��
		Vertex* firstVertex = findByIndex(firstIdx);
		Vertex* secondVertex = findByIndex(secondIdx);
		Vertex* thirdVertex = findByIndex(thirdIdx);
		if (!firstVertex || !secondVertex || !thirdVertex) {
			return nullptr;
		}
		if (firstVertex == secondVertex ||
			secondVertex == thirdVertex ||
			thirdVertex == firstVertex) {
			return nullptr;
		}
		// ������
		// �߼����
		// ����߼���edges_
		Edge* edge_1 = findOrAddEdge(firstVertex, secondVertex);
		Edge* edge_2 = findOrAddEdge(secondVertex, thirdVertex);
		Edge* edge_3 = findOrAddEdge(thirdVertex, firstVertex);
		//Edge* edge_4 = findOrAddEdge(secondVertex, firstVertex);
		//Edge* edge_5 = findOrAddEdge(thirdVertex, secondVertex);
		//Edge* edge_6 = findOrAddEdge(firstVertex, thirdVertex);
		if (!edge_1 || !edge_2 || !edge_3) {
			return nullptr;
		}
		//����������
		int triIndex = triangles_.size();
		std::unique_ptr<Triangle> tri = std::make_unique<Triangle>(triIndex, firstVertex, secondVertex, thirdVertex);
		Triangle* raw = tri.get();
		if (raw->area < 1e-12) {
			return nullptr;
		}
		//�����μ���triangles_
		triangles_.push_back(std::move(tri));
		//�����μ����
		firstVertex->addNeiTri(raw);
		secondVertex->addNeiTri(raw);
		thirdVertex->addNeiTri(raw);
		//�����μ����
		edge_1->addTriangle(raw);
		edge_2->addTriangle(raw);
		edge_3->addTriangle(raw);
		//�߼���������
		raw->setEdges(edge_1, edge_2, edge_3);
		return raw;
	}

	std::size_t vertexCount() const noexcept {
		return vertices_.size();
	}
	std::size_t edgeCount() const noexcept {
		return edges_.size();
	}
	std::size_t triangleCount() const noexcept {
		return triangles_.size();
	}

	// Access triangle by index
	const Triangle* triangle(int idx) const {
		if (idx < 0 || idx >= static_cast<int>(triangles_.size())) return nullptr;
		return triangles_[idx].get();
	}
	Triangle* triangle(int idx) {
		if (idx < 0 || idx >= static_cast<int>(triangles_.size())) return nullptr;
		return triangles_[idx].get();
	}

	//��ͨ���ж�
	//ͨ������ж�
	std::vector<std::vector<Vertex*>> connectedVertexComponents() const {
		std::vector<std::vector<Vertex*>> components;
		std::unordered_set<Vertex*> visited;
		for (const std::unique_ptr<Vertex>& vptr : vertices_) {
			Vertex* start = vptr.get();

			// ��Ϊ�ջ��ߵ��Ѿ������� �������һ����Ĳ�ѯ
			if (!start) continue;
			if (visited.find(start) != visited.end()) continue;

			std::vector<Vertex*> component;
			std::queue<Vertex*> q;

			visited.insert(start);
			q.push(start);

			while (!q.empty()) {
				//��ȡ���е�һ��������
				Vertex* cur = q.front();
				q.pop();
				component.push_back(cur);

				//������������ھӵ�
				for (Vertex* nei : cur->getNeiVertics()) {
					// �жϵ��Ƿ�Ϊ�պͷ��ʹ�
					if (!nei) continue;
					if (visited.find(nei) != visited.end()) continue;

					visited.insert(nei);
					q.push(nei);
				}

			}
			components.push_back(std::move(component));
		}
		return components;

	}
	//ͨ���㼯��ȡÿ����ͨ�������漯
	std::vector<Triangle*> collectTriangleFromVertexComponent(const std::vector<Vertex*>& component) {
		std::unordered_set<Triangle*> triSet;
		for (Vertex* vt : component) {
			if (!vt) continue;
			for (Triangle* tri : vt->getNeiTriangles()) {
				if (!tri) continue;
				triSet.insert(tri);
			}
		}
		return std::vector<Triangle*>(triSet.begin(), triSet.end());
	}
	//���ڵ㼯���зָ�
	std::vector<Mesh> splitComponents() {
		std::vector<Mesh> results;
		//��ȡ��ͨ����
		std::vector<std::vector<Vertex*>> components = connectedVertexComponents();
		//��ÿ����ͨ�������д���������
		for (std::vector<Vertex*> component : components) {
			Mesh subMesh;
			std::unordered_map<Vertex*, int> oldToNewIndex;

			//�����㣬����֮ǰ�ģ��൱�ڵ���
			for (Vertex* oldVertex : component) {
				Vertex* newVertex = subMesh.addVertex(oldVertex->x, oldVertex->y, oldVertex->z);
				//�ɵ�-������
				oldToNewIndex[oldVertex] = newVertex->index;
			}
			//�����棬����֮ǰ�ģ��൱�ڵ���
			std::vector<Triangle*> tris = collectTriangleFromVertexComponent(component);
			for (Triangle* oldTri : tris) {
				//��ȡ�ɵ�
				Vertex* ov0 = oldTri->vertex(0);
				Vertex* ov1 = oldTri->vertex(1);
				Vertex* ov2 = oldTri->vertex(2);

				if (!ov0 || !ov1 || !ov2) {
					continue;
				}
				//��ȡ�ɵ�-��������
				std::unordered_map<Vertex*, int>::iterator it0 = oldToNewIndex.find(ov0);
				std::unordered_map<Vertex*, int>::iterator it1 = oldToNewIndex.find(ov1);
				std::unordered_map<Vertex*, int>::iterator it2 = oldToNewIndex.find(ov2);

				if (it0 == oldToNewIndex.end() ||
					it1 == oldToNewIndex.end() ||
					it2 == oldToNewIndex.end()) {
					continue;
				}
				//�����µ������index
				subMesh.addTriangle(it0->second, it1->second, it2->second);
			}
			results.push_back(std::move(subMesh));
		}
		return results;
	}

	//ͨ�������ж�
	std::vector<std::vector<Triangle*>> connectedTriangleComponents() const {
		std::vector<std::vector<Triangle*>> components;
		std::unordered_set<Triangle*> visited;

		for (const auto& tptr : triangles_) {
			Triangle* start = tptr.get();

			if (!start || visited.find(start) != visited.end()) {
				continue;
			}

			std::vector<Triangle*> component;
			std::queue<Triangle*> queue;

			visited.insert(start);
			queue.push(start);

			while (!queue.empty()) {
				Triangle* current = queue.front();
				queue.pop();

				component.push_back(current);

				for (int i = 0; i < 3; ++i) {
					Edge* edge = current->edge(i);
					if (!edge) {
						continue;
					}

					auto enqueueTriangles =
						[&](const std::unordered_set<Triangle*>& triangles) {
						for (Triangle* neighbor : triangles) {
							if (neighbor &&
								visited.insert(neighbor).second) {
								queue.push(neighbor);
							}
						}
						};

					// ����ͬ���򹲱ߵ������
					enqueueTriangles(edge->triangles());

					// ���������£�������λ�ڷ���ߡ�
					Edge* opposite = edge->opposite();
					if (opposite) {
						enqueueTriangles(opposite->triangles());
					}
				}
			}

			components.push_back(std::move(component));
		}

		return components;
	}







	//��������
	bool exportObj(const std::string& filePath) const {
		std::ofstream output(filePath);

		if (!output.is_open()) {
			return false;
		}

		output << "# Exported by Mesh::exportObj\n";
		output << "# Vertices: " << vertices_.size() << "\n";
		output << "# Triangles: " << triangles_.size() << "\n\n";

		std::unordered_map<const Vertex*, int> vertexToObjIndex;
		vertexToObjIndex.reserve(vertices_.size());

		output << std::fixed << std::setprecision(10);

		// 1. д������
		for (std::size_t i = 0; i < vertices_.size(); ++i) {
			const Vertex* v = vertices_[i].get();

			if (!v) {
				continue;
			}

			int objIndex = static_cast<int>(i) + 1;
			vertexToObjIndex[v] = objIndex;

			output << "v "
				<< v->x << " "
				<< v->y << " "
				<< v->z << "\n";
		}

		output << "\n";

		// 2. д��������
		for (const auto& triPtr : triangles_) {
			const Triangle* tri = triPtr.get();

			if (!tri) {
				continue;
			}

			const Vertex* v0 = tri->vertex(0);
			const Vertex* v1 = tri->vertex(1);
			const Vertex* v2 = tri->vertex(2);

			if (!v0 || !v1 || !v2) {
				continue;
			}

			auto it0 = vertexToObjIndex.find(v0);
			auto it1 = vertexToObjIndex.find(v1);
			auto it2 = vertexToObjIndex.find(v2);

			if (it0 == vertexToObjIndex.end() ||
				it1 == vertexToObjIndex.end() ||
				it2 == vertexToObjIndex.end()) {
				continue;
			}

			output << "f "
				<< it0->second << " "
				<< it1->second << " "
				<< it2->second << "\n";
		}

		return true;
	}

	//���˼�鱨��
	MeshValidationReport validateBasicTopology(double eps = 1e-12) const;
	std::size_t directedTriangleCount(const Edge* e) const;
	std::size_t undirectedTriangleCount(const EdgePair& pair) const;
	void printEdgeUsageSummary() const;

	//���κ�ˮ�ܼ��
	//��һ�����߼����
	void checkEdgeManifoldAndBoundary(MeshCheckReport& report) const;

	//�ڶ���������һ���Լ��
	void checkOrientationConsistency(MeshCheckReport& report) const;

	//���������˻������μ��
	void checkDegenerateTriangles(MeshCheckReport& report) const;

	//���Ĳ����㼶�����μ��
	void checkNonManifoldVertices(MeshCheckReport& report) const;
	//��鵥�������Ƿ��Ƿ����ε�
	bool isNonManifoldVertex(const Vertex* center) const;
	// Self-intersection detection
	void checkSelfIntersection(MeshCheckReport& report) const;
	// Combined check (legacy)
	MeshCheckReport checkManifoldAndWatertight() const;
	// Unified entry: all checks
	MeshCheckReport validateAll() const;

	// ===== 姿态变换 =====

	// 对所有顶点应用 4×4 变换矩阵，自动重算法线和面积
	void transform(const Mat4& mat) {
		for (auto& vptr : vertices_) {
			if (!vptr) continue;
			Vec3 pos = mat.transformPoint(Vec3(vptr->x, vptr->y, vptr->z));
			vptr->x = pos.x;
			vptr->y = pos.y;
			vptr->z = pos.z;
		}
		// 重算所有三角形法线和面积
		for (auto& triptr : triangles_) {
			if (triptr) triptr->computeNormalAndArea();
		}
	}

	// 平移
	void translate(const Vec3& offset) {
		transform(Mat4::translation(offset));
	}

	void translate(double dx, double dy, double dz) {
		transform(Mat4::translation(dx, dy, dz));
	}

	// 用四元数旋转
	void rotate(const Quat& q) {
		transform(q.toMat4());
	}

	// 绕轴旋转
	void rotate(const Vec3& axis, double radians) {
		transform(Mat4::rotationAxis(axis, radians));
	}

	// 欧拉角旋转（XYZ 内旋，弧度）
	void rotateEuler(double pitch, double yaw, double roll) {
		transform(Quat::fromEuler(pitch, yaw, roll).toMat4());
	}

	// 缩放
	void scale(double sx, double sy, double sz) {
		transform(Mat4::scaling(sx, sy, sz));
	}

	void scale(double s) {
		transform(Mat4::scaling(s));
	}

	// Compute axis-aligned bounding box
	// Returns: first = min corner, second = max corner
	std::pair<Vec3, Vec3> boundingBox() const {
		Vec3 minV(1e30, 1e30, 1e30);
		Vec3 maxV(-1e30, -1e30, -1e30);
		for (const auto& vptr : vertices_) {
			if (!vptr) continue;
			minV.x = (std::min)(minV.x, vptr->x);
			minV.y = (std::min)(minV.y, vptr->y);
			minV.z = (std::min)(minV.z, vptr->z);
			maxV.x = (std::max)(maxV.x, vptr->x);
			maxV.y = (std::max)(maxV.y, vptr->y);
			maxV.z = (std::max)(maxV.z, vptr->z);
		}
		return {minV, maxV};
	}

	// Normalize mesh to fit within [-1, 1] bounding box
	// 1. Translate so bounding box center is at origin
	// 2. Uniform scale so longest axis spans [-1, 1]
	void normalizeToUnit() {
		auto [minV, maxV] = boundingBox();
		Vec3 center(
			(minV.x + maxV.x) * 0.5,
			(minV.y + maxV.y) * 0.5,
			(minV.z + maxV.z) * 0.5
		);
		Vec3 extent(maxV.x - minV.x, maxV.y - minV.y, maxV.z - minV.z);
		double maxExtent = (std::max)({extent.x, extent.y, extent.z});
		if (maxExtent < 1e-15) return;
		double s = 2.0 / maxExtent;
		// Order: first translate to origin, then scale
		Mat4 mat = Mat4::scaling(s) * Mat4::translation(-center);
		transform(mat);
	}

	// ===== Normal computation =====

	// Compute normals for all vertices (default: area-weighted)
	std::vector<Vec3> computeVertexNormals(
		NormalComputer::VertexNormalMethod method =
			NormalComputer::VertexNormalMethod::AreaWeighted) const {
		return NormalComputer::computeAllVertexNormals(vertices_, method);
	}

	// Compute normals for all edges
	std::vector<Vec3> computeEdgeNormals() const {
		return NormalComputer::computeAllEdgeNormals(edges_);
	}

	// Access individual edge normal
	Vec3 edgeNormal(int edgeIdx) const {
		if (edgeIdx < 0 || edgeIdx >= static_cast<int>(edges_.size())) return Vec3(0,0,0);
		if (!edges_[edgeIdx]) return Vec3(0,0,0);
		return NormalComputer::edgeNormal(*edges_[edgeIdx]);
	}

	// Access individual vertex normal
	Vec3 vertexNormal(int vertexIdx,
		NormalComputer::VertexNormalMethod method =
			NormalComputer::VertexNormalMethod::AreaWeighted) const {
		if (vertexIdx < 0 || vertexIdx >= static_cast<int>(vertices_.size())) return Vec3(0,0,0);
		if (!vertices_[vertexIdx]) return Vec3(0,0,0);
		switch (method) {
		case NormalComputer::VertexNormalMethod::Simple:
			return NormalComputer::vertexNormalSimple(*vertices_[vertexIdx]);
		case NormalComputer::VertexNormalMethod::AreaWeighted:
			return NormalComputer::vertexNormalAreaWeighted(*vertices_[vertexIdx]);
		case NormalComputer::VertexNormalMethod::AngleWeighted:
			return NormalComputer::vertexNormalAngleWeighted(*vertices_[vertexIdx]);
		case NormalComputer::VertexNormalMethod::FromEdges:
			return NormalComputer::vertexNormalFromEdges(*vertices_[vertexIdx]);
		}
		return Vec3(0,0,0);
	}

};

// classifyPointInMesh: ray casting point-in-mesh test
// Uses multiple rays to handle degenerate cases
inline geo::PointClass classifyPointInMesh(const Point& p, const Mesh& mesh, double eps = 1e-8) {
	// Check if point is on any triangle face
	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (tri && geo::pointInTriangle(p, tri, eps)) {
			return geo::PointClass::OnBoundary;
		}
	}

	// Cast ray in +X direction, count unique intersection points
	Point rayEnd(p.x + 1e6, p.y, p.z);
	std::vector<double> hitTs;

	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;

		Vertex* v0 = tri->vertex(0);
		Vertex* v1 = tri->vertex(1);
		Vertex* v2 = tri->vertex(2);
		if (!v0 || !v1 || !v2) continue;

		Point hit;
		if (geo::segmentTriangleIntersection(p, rayEnd, tri, hit, eps)) {
			// Use t parameter to deduplicate
			double t = hit.x - p.x;  // ray is along +X
			bool duplicate = false;
			for (double existing : hitTs) {
				if (std::abs(existing - t) < eps * 100) {
					duplicate = true;
					break;
				}
			}
			if (!duplicate) {
				hitTs.push_back(t);
			}
		}
	}

	return hitTs.size() % 2 == 1 ? geo::PointClass::Inside : geo::PointClass::Outside;
}


