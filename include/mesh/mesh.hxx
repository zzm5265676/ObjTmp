// mesh.hxx - lightweight OBJ mesh with directed edges and component splitting
#pragma once

#include "core/edge.hxx"
#include "core/point.hxx"
#include "core/triangle.hxx"
#include "core/vertex.hxx"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

inline int parseObjVertexIndex(const std::string& token) {
	std::size_t pos = token.find('/');
	std::string indexStr = pos == std::string::npos ? token : token.substr(0, pos);
	return std::stoi(indexStr);
}

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

struct UndirectedEdgeIndexKey {
	int a = -1;
	int b = -1;

	UndirectedEdgeIndexKey() = default;
	UndirectedEdgeIndexKey(int i0, int i1) {
		if (i0 <= i1) {
			a = i0;
			b = i1;
		}
		else {
			a = i1;
			b = i0;
		}
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

struct QuantizedVertexKey {
	long long x = 0;
	long long y = 0;
	long long z = 0;

	bool operator==(const QuantizedVertexKey& other) const noexcept {
		return x == other.x && y == other.y && z == other.z;
	}
};

struct QuantizedVertexKeyHash {
	std::size_t operator()(const QuantizedVertexKey& key) const noexcept {
		std::size_t seed = std::hash<long long>{}(key.x);
		seed ^= std::hash<long long>{}(key.y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
		seed ^= std::hash<long long>{}(key.z) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
		return seed;
	}
};

class Mesh {
private:
	std::vector<std::unique_ptr<Vertex>> vertices_;
	std::vector<std::unique_ptr<Edge>> edges_;
	std::vector<std::unique_ptr<Triangle>> triangles_;
	std::unordered_map<DirectedEdgeIndexKey, Edge*, DirectedEdgeIndexKeyHash> directed_edge_map_;

	std::vector<std::array<double, 3>> normals_;
	std::vector<std::array<double, 2>> texCoords_;
	std::vector<std::string> parseErrors_;

	static bool isDegenerateTriangle(const Vertex* v0, const Vertex* v1, const Vertex* v2) {
		if (!v0 || !v1 || !v2) return true;

		Vec3 e1(v1->x - v0->x, v1->y - v0->y, v1->z - v0->z);
		Vec3 e2(v2->x - v0->x, v2->y - v0->y, v2->z - v0->z);
		return e1.cross(e2).cachedLength() < 1e-12;
	}

	static QuantizedVertexKey quantizeVertex(double x, double y, double z, double tolerance) {
		return {
			static_cast<long long>(std::llround(x / tolerance)),
			static_cast<long long>(std::llround(y / tolerance)),
			static_cast<long long>(std::llround(z / tolerance))
		};
	}

	int resolveObjVertexIndex(int objIndex, int objVertexCount) const {
		if (objIndex > 0) return objIndex - 1;
		if (objIndex < 0) return objVertexCount + objIndex;
		return -1;
	}

public:
	Mesh(const Mesh&) = delete;
	Mesh& operator=(const Mesh&) = delete;

	Mesh(Mesh&&) noexcept = default;
	Mesh& operator=(Mesh&&) noexcept = default;

	Mesh() = default;

	explicit Mesh(const std::string& filePath, double connectTolerance = 0.0) {
		if (connectTolerance < 0.0) {
			throw std::runtime_error("connectTolerance must be non-negative");
		}

		std::ifstream input(filePath);
		if (!input.is_open()) {
			throw std::runtime_error("Failed to open obj file: " + filePath);
		}

		std::vector<int> objToMeshVertexIndex;
		std::unordered_map<QuantizedVertexKey, int, QuantizedVertexKeyHash> weldedVertexMap;

		int lineNumber = 0;
		std::string line;
		while (std::getline(input, line)) {
			++lineNumber;
			if (line.empty() || line[0] == '#') continue;

			std::istringstream iss(line);
			std::string type;
			iss >> type;

			if (type == "v") {
				double x = 0.0;
				double y = 0.0;
				double z = 0.0;
				if (!(iss >> x >> y >> z)) {
					parseErrors_.push_back("Line " + std::to_string(lineNumber) + ": invalid vertex");
					continue;
				}

				int meshVertexIndex = -1;
				if (connectTolerance > 0.0) {
					QuantizedVertexKey key = quantizeVertex(x, y, z, connectTolerance);
					auto found = weldedVertexMap.find(key);
					if (found != weldedVertexMap.end()) {
						meshVertexIndex = found->second;
					}
					else {
						meshVertexIndex = addVertex(x, y, z)->index;
						weldedVertexMap.emplace(key, meshVertexIndex);
					}
				}
				else {
					meshVertexIndex = addVertex(x, y, z)->index;
				}
				objToMeshVertexIndex.push_back(meshVertexIndex);
			}
			else if (type == "vn") {
				double nx = 0.0;
				double ny = 0.0;
				double nz = 0.0;
				if (!(iss >> nx >> ny >> nz)) {
					parseErrors_.push_back("Line " + std::to_string(lineNumber) + ": invalid normal");
					continue;
				}
				normals_.push_back({ nx, ny, nz });
			}
			else if (type == "vt") {
				double u = 0.0;
				double v = 0.0;
				if (!(iss >> u >> v)) {
					parseErrors_.push_back("Line " + std::to_string(lineNumber) + ": invalid texcoord");
					continue;
				}
				texCoords_.push_back({ u, v });
			}
			else if (type == "f") {
				std::vector<int> indices;
				std::string token;
				while (iss >> token) {
					try {
						int objVertexIndex = resolveObjVertexIndex(
							parseObjVertexIndex(token),
							static_cast<int>(objToMeshVertexIndex.size()));
						if (objVertexIndex >= 0
							&& objVertexIndex < static_cast<int>(objToMeshVertexIndex.size())) {
							indices.push_back(objToMeshVertexIndex[static_cast<std::size_t>(objVertexIndex)]);
						}
						else {
							indices.push_back(-1);
						}
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

				bool validRange = true;
				for (int idx : indices) {
					if (idx < 0 || idx >= static_cast<int>(vertices_.size())) {
						parseErrors_.push_back("Line " + std::to_string(lineNumber)
							+ ": vertex index out of range");
						validRange = false;
						break;
					}
				}
				if (!validRange) continue;

				for (std::size_t i = 1; i + 1 < indices.size(); ++i) {
					if (!addTriangle(indices[0], indices[i], indices[i + 1])) {
						parseErrors_.push_back("Line " + std::to_string(lineNumber)
							+ ": skipped degenerate triangle");
					}
				}
			}
		}
	}

	const std::vector<std::string>& parseErrors() const noexcept {
		return parseErrors_;
	}

	const std::vector<std::array<double, 3>>& normals() const noexcept {
		return normals_;
	}

	const std::vector<std::array<double, 2>>& texCoords() const noexcept {
		return texCoords_;
	}

	Vertex* addVertex(double x, double y, double z) {
		int idx = static_cast<int>(vertices_.size());
		vertices_.push_back(std::make_unique<Vertex>(idx, x, y, z));
		return vertices_.back().get();
	}

	Vertex* findByIndex(int idx) {
		if (idx < 0 || idx >= static_cast<int>(vertices_.size())) return nullptr;
		return vertices_[idx].get();
	}

	const Vertex* findByIndex(int idx) const {
		if (idx < 0 || idx >= static_cast<int>(vertices_.size())) return nullptr;
		return vertices_[idx].get();
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

	Triangle* triangle(int idx) {
		if (idx < 0 || idx >= static_cast<int>(triangles_.size())) return nullptr;
		return triangles_[idx].get();
	}

	const Triangle* triangle(int idx) const {
		if (idx < 0 || idx >= static_cast<int>(triangles_.size())) return nullptr;
		return triangles_[idx].get();
	}

	EdgePair getUndirectedEdgePair(int i0, int i1) const {
		EdgePair pair;
		auto itAB = directed_edge_map_.find(DirectedEdgeIndexKey(i0, i1));
		if (itAB != directed_edge_map_.end()) pair.ab = itAB->second;

		auto itBA = directed_edge_map_.find(DirectedEdgeIndexKey(i1, i0));
		if (itBA != directed_edge_map_.end()) pair.ba = itBA->second;
		return pair;
	}

	std::vector<std::pair<UndirectedEdgeIndexKey, EdgePair>> collectUndirectedEdges() const {
		std::unordered_set<UndirectedEdgeIndexKey, UndirectedEdgeIndexKeyHash> seen;
		std::vector<std::pair<UndirectedEdgeIndexKey, EdgePair>> result;
		result.reserve(directed_edge_map_.size());

		for (const auto& entry : directed_edge_map_) {
			const Edge* edge = entry.second;
			if (!edge || !edge->from() || !edge->to()) continue;

			UndirectedEdgeIndexKey key(edge->from()->index, edge->to()->index);
			if (seen.insert(key).second) {
				result.emplace_back(key, getUndirectedEdgePair(key.a, key.b));
			}
		}
		return result;
	}

	Edge* findOrAddEdge(Vertex* v0, Vertex* v1) {
		if (!v0 || !v1 || v0 == v1) return nullptr;

		DirectedEdgeIndexKey key(v0->index, v1->index);
		auto it = directed_edge_map_.find(key);
		if (it != directed_edge_map_.end()) return it->second;

		int idx = static_cast<int>(edges_.size());
		auto edge = std::make_unique<Edge>(idx, v0, v1);
		Edge* raw = edge.get();

		auto oppositeIt = directed_edge_map_.find(DirectedEdgeIndexKey(v1->index, v0->index));
		if (oppositeIt != directed_edge_map_.end()) {
			raw->setOpposite(oppositeIt->second);
		}

		edges_.push_back(std::move(edge));
		directed_edge_map_.emplace(key, raw);

		v0->addNeiVertex(v1);
		v1->addNeiVertex(v0);
		v0->addNeiEdge(raw);
		v1->addNeiEdge(raw);
		return raw;
	}

	Triangle* addTriangle(int firstIdx, int secondIdx, int thirdIdx) {
		Vertex* firstVertex = findByIndex(firstIdx);
		Vertex* secondVertex = findByIndex(secondIdx);
		Vertex* thirdVertex = findByIndex(thirdIdx);

		if (!firstVertex || !secondVertex || !thirdVertex) return nullptr;
		if (firstVertex == secondVertex || secondVertex == thirdVertex || thirdVertex == firstVertex) {
			return nullptr;
		}
		if (isDegenerateTriangle(firstVertex, secondVertex, thirdVertex)) {
			return nullptr;
		}

		Edge* edge0 = findOrAddEdge(firstVertex, secondVertex);
		Edge* edge1 = findOrAddEdge(secondVertex, thirdVertex);
		Edge* edge2 = findOrAddEdge(thirdVertex, firstVertex);
		if (!edge0 || !edge1 || !edge2) return nullptr;

		int triIndex = static_cast<int>(triangles_.size());
		auto tri = std::make_unique<Triangle>(triIndex, firstVertex, secondVertex, thirdVertex);
		Triangle* raw = tri.get();

		triangles_.push_back(std::move(tri));

		firstVertex->addNeiTri(raw);
		secondVertex->addNeiTri(raw);
		thirdVertex->addNeiTri(raw);

		edge0->addTriangle(raw);
		edge1->addTriangle(raw);
		edge2->addTriangle(raw);
		raw->setEdges(edge0, edge1, edge2);
		return raw;
	}

	std::vector<std::vector<Vertex*>> connectedVertexComponents() const {
		std::vector<std::vector<Vertex*>> components;
		std::unordered_set<Vertex*> visited;
		visited.reserve(vertices_.size());

		for (const auto& vertexPtr : vertices_) {
			Vertex* start = vertexPtr.get();
			if (!start || visited.find(start) != visited.end()) continue;

			std::vector<Vertex*> component;
			std::queue<Vertex*> queue;

			visited.insert(start);
			queue.push(start);

			while (!queue.empty()) {
				Vertex* current = queue.front();
				queue.pop();
				component.push_back(current);

				for (Vertex* neighbor : current->getNeiVertics()) {
					if (!neighbor || visited.find(neighbor) != visited.end()) continue;
					visited.insert(neighbor);
					queue.push(neighbor);
				}
			}

			components.push_back(std::move(component));
		}

		return components;
	}

	std::vector<Triangle*> collectTriangleFromVertexComponent(const std::vector<Vertex*>& component) const {
		std::unordered_set<Triangle*> triSet;
		for (Vertex* vertex : component) {
			if (!vertex) continue;
			for (Triangle* tri : vertex->getNeiTriangles()) {
				if (tri) triSet.insert(tri);
			}
		}

		std::vector<Triangle*> triangles(triSet.begin(), triSet.end());
		std::sort(triangles.begin(), triangles.end(), [](const Triangle* lhs, const Triangle* rhs) {
			return lhs->index < rhs->index;
		});
		return triangles;
	}

	std::vector<std::vector<Triangle*>> connectedTriangleComponents() const {
		std::vector<std::vector<Triangle*>> components;
		std::unordered_set<Triangle*> visited;
		visited.reserve(triangles_.size());

		for (const auto& triPtr : triangles_) {
			Triangle* start = triPtr.get();
			if (!start || visited.find(start) != visited.end()) continue;

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
					if (!edge) continue;

					auto enqueueTriangles = [&](const std::unordered_set<Triangle*>& triangles) {
						for (Triangle* neighbor : triangles) {
							if (neighbor && visited.insert(neighbor).second) {
								queue.push(neighbor);
							}
						}
					};

					enqueueTriangles(edge->triangles());
					if (edge->opposite()) {
						enqueueTriangles(edge->opposite()->triangles());
					}
				}
			}

			components.push_back(std::move(component));
		}

		return components;
	}

	std::vector<Mesh> splitComponents() const {
		std::vector<Mesh> results;
		std::vector<std::vector<Vertex*>> components = connectedVertexComponents();
		results.reserve(components.size());

		for (const auto& component : components) {
			Mesh subMesh;
			std::unordered_map<Vertex*, int> oldToNewIndex;
			oldToNewIndex.reserve(component.size());

			for (Vertex* oldVertex : component) {
				if (!oldVertex) continue;
				Vertex* newVertex = subMesh.addVertex(oldVertex->x, oldVertex->y, oldVertex->z);
				oldToNewIndex[oldVertex] = newVertex->index;
			}

			std::vector<Triangle*> tris = collectTriangleFromVertexComponent(component);
			for (Triangle* oldTri : tris) {
				if (!oldTri) continue;

				Vertex* ov0 = oldTri->vertex(0);
				Vertex* ov1 = oldTri->vertex(1);
				Vertex* ov2 = oldTri->vertex(2);
				if (!ov0 || !ov1 || !ov2) continue;

				auto it0 = oldToNewIndex.find(ov0);
				auto it1 = oldToNewIndex.find(ov1);
				auto it2 = oldToNewIndex.find(ov2);
				if (it0 == oldToNewIndex.end()
					|| it1 == oldToNewIndex.end()
					|| it2 == oldToNewIndex.end()) {
					continue;
				}

				subMesh.addTriangle(it0->second, it1->second, it2->second);
			}

			results.push_back(std::move(subMesh));
		}

		return results;
	}

	bool exportObj(const std::string& filePath) const {
		std::ofstream output(filePath);
		if (!output.is_open()) return false;

		output << "# Exported by Mesh::exportObj\n";
		output << "# Vertices: " << vertices_.size() << "\n";
		output << "# Triangles: " << triangles_.size() << "\n\n";
		output << std::fixed << std::setprecision(10);

		std::unordered_map<const Vertex*, int> vertexToObjIndex;
		vertexToObjIndex.reserve(vertices_.size());

		for (std::size_t i = 0; i < vertices_.size(); ++i) {
			const Vertex* vertex = vertices_[i].get();
			if (!vertex) continue;

			int objIndex = static_cast<int>(i) + 1;
			vertexToObjIndex[vertex] = objIndex;
			output << "v " << vertex->x << " " << vertex->y << " " << vertex->z << "\n";
		}

		output << "\n";

		for (const auto& triPtr : triangles_) {
			const Triangle* tri = triPtr.get();
			if (!tri) continue;

			const Vertex* v0 = tri->vertex(0);
			const Vertex* v1 = tri->vertex(1);
			const Vertex* v2 = tri->vertex(2);
			if (!v0 || !v1 || !v2) continue;

			auto it0 = vertexToObjIndex.find(v0);
			auto it1 = vertexToObjIndex.find(v1);
			auto it2 = vertexToObjIndex.find(v2);
			if (it0 == vertexToObjIndex.end()
				|| it1 == vertexToObjIndex.end()
				|| it2 == vertexToObjIndex.end()) {
				continue;
			}

			output << "f " << it0->second << " " << it1->second << " " << it2->second << "\n";
		}

		return true;
	}
};
