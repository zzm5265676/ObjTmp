#pragma once 
#include "point.hxx"
#include "triangle.hxx"
#include "edge.hxx"
#include <memory>
#include <unordered_map>
#include <vector>
#include <functional>
#include <fstream>
#include <sstream>
#include <queue>
#include <iomanip>
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
//有向边equal和hash
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
//无向边equal和hash
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
class Mesh {
private:
	std::vector<std::unique_ptr<Vertex>> vertices_;//不需要equal和hash，只有点
	std::vector<std::unique_ptr<Edge>> edges_;//不去重，有向边
	std::vector<std::unique_ptr<Triangle>> triangles_;//三角面片，不需要去重
	std::unordered_map<DirectedEdgeIndexKey, Edge*, DirectedEdgeIndexKeyHash> directed_edge_map_;
	std::unordered_map<UndirectedEdgeIndexKey, EdgePair, UndirectedEdgeIndexKeyHash> edge_map_;


public:
	Mesh(const Mesh&) = delete;
	Mesh& operator=(const Mesh&) = delete;

	Mesh(Mesh&&) noexcept = default;
	Mesh& operator=(Mesh&&) noexcept = default;

	Mesh() = default;
	// 路径构造
	Mesh(std::string filePath) {
		std::ifstream input(filePath);

		if (!input.is_open()) {
			throw std::runtime_error("Failed to open obj file: " + filePath);
		}
		std::string line;

		while (std::getline(input, line)) {
			if (line.empty()) {
				continue;
			}

			if (line[0] == '#') {
				continue;
			}

			std::istringstream iss(line);

			std::string type;
			iss >> type;

			if (type == "v") {
				double x, y, z;

				if (!(iss >> x >> y >> z)) {
					std::cout << "Invalid vertex line: " << line << std::endl;
					continue;
				}

				addVertex(x, y, z);
			}
			else if (type == "f") {
				std::vector<int> indices;

				std::string token;
				while (iss >> token) {
					try {
						int objIndex = parseObjVertexIndex(token);

						// OBJ 顶点索引从 1 开始，Mesh 内部 index 从 0 开始
						indices.push_back(objIndex - 1);
					}
					catch (...) {
						std::cout << "Invalid face token: " << token << std::endl;
					}
				}

				if (indices.size() < 3) {
					std::cout << "Invalid face line: " << line << std::endl;
					continue;
				}

				// 如果是三角形
				if (indices.size() == 3) {
					addTriangle(indices[0], indices[1], indices[2]);
				}
				else {
					// 如果是四边形或多边形，使用扇形剖分
					// f v0 v1 v2 v3 -> (v0,v1,v2), (v0,v2,v3)
					for (std::size_t i = 1; i + 1 < indices.size(); ++i) {
						addTriangle(indices[0], indices[i], indices[i + 1]);
					}
				}
			}
		}

	}
	// 点加入vertices_
	Vertex* addVertex(double x, double y, double z) {
		int idx = static_cast<int>(vertices_.size());
		vertices_.push_back(std::make_unique<Vertex>(idx, x, y, z));
		return vertices_.back().get();
	}
	// 通过索引返回点
	Vertex* findByIndex(int idx) {
		return vertices_[idx].get();
	}
	//通过点寻找边
	//找到返回，没找到加入并返回
	Edge* findOrAddEdge(Vertex* v0, Vertex* v1) {
		if (!v0 || !v1) { return nullptr; }
		if (v0 == v1) { return nullptr; }

		// 处理有向边
		// 传入的正向边
		// 构造正向边索引
		DirectedEdgeIndexKey dkey(v0->index, v1->index);
		// 查找并返回
		auto dit = directed_edge_map_.find(dkey);
		if (dit != directed_edge_map_.end()) {
			return dit->second;
		}
		// 没有则构造，准备插入
		int idx = static_cast<int>(edges_.size());
		auto edge = std::make_unique<Edge>(idx, v0, v1);
		Edge* raw = edge.get();

		// 有向边（为传入的正向边）插入edges_
		edges_.push_back(std::move(edge));
		// 有向边（为传入的正向边）插入directed_edge_map_
		directed_edge_map_.emplace(dkey, raw);



		// 处理有向边
		// 传入的反向边
		// 构造反向边索引
		DirectedEdgeIndexKey reverse_key(v1->index, v0->index);
		// 查找并返回
		auto rit = directed_edge_map_.find(reverse_key);
		if (rit != directed_edge_map_.end()) {
			Edge* opposite = rit->second;
			// 如果你的 Edge 类支持 opposite，可以打开这两句
			raw->setOpposite(opposite);
			opposite->setOpposite(raw);
		}



		// 无向边处理
		// 构造无向边索引
		UndirectedEdgeIndexKey ukey(v0->index, v1->index);
		// 构造无向边对象（包含正反2条边的结构体）
		EdgePair& pair = edge_map_[ukey];

		if (v0->index == ukey.a && v1->index == ukey.b) {
			pair.ab = raw;
		}
		else {
			pair.ba = raw;
		}

		v0->addNeiVertex(v1);
		v1->addNeiVertex(v0);

		v0->addNeiEdge(raw);
		v1->addNeiEdge(raw);

		return raw;
	}
	Triangle* addTriangle(int firstIdx, int secondIdx, int thirdIdx) {
		// 获取点
		Vertex* firstVertex = findByIndex(firstIdx);
		Vertex* secondVertex = findByIndex(secondIdx);
		Vertex* thirdVertex = findByIndex(thirdIdx);
		// 点加入点
		// 边加入点
		// 有向边加入edges_
		// 无向边加入edge_map_
		Edge* edge_1 = findOrAddEdge(firstVertex, secondVertex);
		Edge* edge_2 = findOrAddEdge(secondVertex, thirdVertex);
		Edge* edge_3 = findOrAddEdge(thirdVertex, firstVertex);
		//Edge* edge_4 = findOrAddEdge(secondVertex, firstVertex);
		//Edge* edge_5 = findOrAddEdge(thirdVertex, secondVertex);
		//Edge* edge_6 = findOrAddEdge(firstVertex, thirdVertex);
		if (!edge_1 || !edge_2 || !edge_3) {
			return nullptr;
		}
		//处理三角形
		int triIndex = triangles_.size();
		std::unique_ptr<Triangle> tri = std::make_unique<Triangle>(triIndex, firstVertex, secondVertex, thirdVertex);
		Triangle* raw = tri.get();
		//三角形加入triangles_
		triangles_.push_back(std::move(tri));
		//三角形加入点
		firstVertex->addNeiTri(raw);
		secondVertex->addNeiTri(raw);
		thirdVertex->addNeiTri(raw);
		//三角形加入边
		edge_1->addTriangle(raw);
		edge_2->addTriangle(raw);
		edge_3->addTriangle(raw);
		//边加入三角形
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

	//连通性判定
	//通过点边判定
	std::vector<std::vector<Vertex*>> connectedVertexComponents() const {
		std::vector<std::vector<Vertex*>> components;
		std::unordered_set<Vertex*> visited;
		for (const std::unique_ptr<Vertex>& vptr : vertices_) {
			Vertex* start = vptr.get();

			// 点为空或者点已经被访问 则进行下一个点的查询
			if (!start) continue;
			if (visited.find(start) != visited.end()) continue;

			std::vector<Vertex*> component;
			std::queue<Vertex*> q;

			visited.insert(start);
			q.push(start);

			while (!q.empty()) {
				//获取队列第一个并弹出
				Vertex* cur = q.front();
				q.pop();
				component.push_back(cur);

				//遍历弹出点的邻居点
				for (Vertex* nei : cur->getNeiVertics()) {
					// 判断点是否为空和访问过
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
	//通过点集获取每个连通分量的面集
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
	//基于点集进行分割
	std::vector<Mesh> splitComponents() {
		std::vector<Mesh> results;
		//获取连通分量
		std::vector<std::vector<Vertex*>> components = connectedVertexComponents();
		//将每个连通分量进行处理和完善
		for (std::vector<Vertex*> component : components) {
			Mesh subMesh;
			std::unordered_map<Vertex*, int> oldToNewIndex;

			//处理点，复用之前的，相当于导入
			for (Vertex* oldVertex : component) {
				Vertex* newVertex = subMesh.addVertex(oldVertex->x, oldVertex->y, oldVertex->z);
				//旧点-新索引
				oldToNewIndex[oldVertex] = newVertex->index;
			}
			//处理面，复用之前的，相当于导入
			std::vector<Triangle*> tris = collectTriangleFromVertexComponent(component);
			for (Triangle* oldTri : tris) {
				//获取旧点
				Vertex* ov0 = oldTri->vertex(0);
				Vertex* ov1 = oldTri->vertex(1);
				Vertex* ov2 = oldTri->vertex(2);

				if (!ov0 || !ov1 || !ov2) {
					continue;
				}
				//获取旧点-新索引对
				std::unordered_map<Vertex*, int>::iterator it0 = oldToNewIndex.find(ov0);
				std::unordered_map<Vertex*, int>::iterator it1 = oldToNewIndex.find(ov1);
				std::unordered_map<Vertex*, int>::iterator it2 = oldToNewIndex.find(ov2);

				if (it0 == oldToNewIndex.end() ||
					it1 == oldToNewIndex.end() ||
					it2 == oldToNewIndex.end()) {
					continue;
				}
				//依赖新点的三个index
				subMesh.addTriangle(it0->second, it1->second, it2->second);
			}
			results.push_back(std::move(subMesh));
		}
		return results;
	}

	//通过边面判定
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

					// 兼容同方向共边的情况。
					enqueueTriangles(edge->triangles());

					// 正常绕序下，相邻面位于反向边。
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







	//导出方法
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

		// 1. 写出顶点
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

		// 2. 写出三角面
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
};
