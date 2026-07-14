#pragma once

#include "mesh/mesh.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace mesh_outer {

	struct AABB {
		double minX = std::numeric_limits<double>::max();
		double minY = std::numeric_limits<double>::max();
		double minZ = std::numeric_limits<double>::max();
		double maxX = -std::numeric_limits<double>::max();
		double maxY = -std::numeric_limits<double>::max();
		double maxZ = -std::numeric_limits<double>::max();

		void expand(const Vertex& vertex) {
			minX = (std::min)(minX, vertex.x);
			minY = (std::min)(minY, vertex.y);
			minZ = (std::min)(minZ, vertex.z);
			maxX = (std::max)(maxX, vertex.x);
			maxY = (std::max)(maxY, vertex.y);
			maxZ = (std::max)(maxZ, vertex.z);
		}
	};

	inline AABB computeAABB(const Mesh& mesh) {
		AABB box;
		for (std::size_t i = 0; i < mesh.vertexCount(); ++i) {
			const Vertex* vertex = mesh.findByIndex(static_cast<int>(i));
			if (vertex) box.expand(*vertex);
		}
		return box;
	}

	inline bool containsAABB(const AABB& outer, const AABB& inner, double eps) {
		return outer.minX <= inner.minX + eps
			&& outer.minY <= inner.minY + eps
			&& outer.minZ <= inner.minZ + eps
			&& outer.maxX + eps >= inner.maxX
			&& outer.maxY + eps >= inner.maxY
			&& outer.maxZ + eps >= inner.maxZ;
	}

	inline bool overlapsAABB(const AABB& a, const AABB& b, double eps) {
		return a.minX <= b.maxX + eps && a.maxX + eps >= b.minX
			&& a.minY <= b.maxY + eps && a.maxY + eps >= b.minY
			&& a.minZ <= b.maxZ + eps && a.maxZ + eps >= b.minZ;
	}

	inline double extent(const AABB& box, int axis) {
		if (axis == 0) return box.maxX - box.minX;
		if (axis == 1) return box.maxY - box.minY;
		return box.maxZ - box.minZ;
	}

	inline double centroidAxis(const AABB& box, int axis) {
		if (axis == 0) return (box.minX + box.maxX) * 0.5;
		if (axis == 1) return (box.minY + box.maxY) * 0.5;
		return (box.minZ + box.maxZ) * 0.5;
	}

	inline bool rayIntersectsAABB(const Point& origin, const Vec3& direction, const AABB& box, double& tMin, double& tMax) {
		tMin = 0.0;
		tMax = std::numeric_limits<double>::max();
		const double o[3] = { origin.x, origin.y, origin.z };
		const double d[3] = { direction.x, direction.y, direction.z };
		const double mn[3] = { box.minX, box.minY, box.minZ };
		const double mx[3] = { box.maxX, box.maxY, box.maxZ };

		for (int axis = 0; axis < 3; ++axis) {
			if (std::abs(d[axis]) < 1e-15) {
				if (o[axis] < mn[axis] || o[axis] > mx[axis]) return false;
				continue;
			}

			double invD = 1.0 / d[axis];
			double t0 = (mn[axis] - o[axis]) * invD;
			double t1 = (mx[axis] - o[axis]) * invD;
			if (t0 > t1) std::swap(t0, t1);
			tMin = (std::max)(tMin, t0);
			tMax = (std::min)(tMax, t1);
			if (tMax < tMin) return false;
		}

		return true;
	}

	inline bool rayIntersectsTriangle(const Point& origin, const Vec3& direction, const Triangle& triangle, double& t) {
		const Vertex* v0 = triangle.vertex(0);
		const Vertex* v1 = triangle.vertex(1);
		const Vertex* v2 = triangle.vertex(2);
		if (!v0 || !v1 || !v2) return false;

		Vec3 edge1(v1->x - v0->x, v1->y - v0->y, v1->z - v0->z);
		Vec3 edge2(v2->x - v0->x, v2->y - v0->y, v2->z - v0->z);
		Vec3 h = direction.cross(edge2);
		double a = edge1.dot(h);
		if (std::abs(a) < 1e-12) return false;

		double f = 1.0 / a;
		Vec3 s(origin.x - v0->x, origin.y - v0->y, origin.z - v0->z);
		double u = f * s.dot(h);
		if (u < -1e-12 || u > 1.0 + 1e-12) return false;

		Vec3 q = s.cross(edge1);
		double v = f * direction.dot(q);
		if (v < -1e-12 || u + v > 1.0 + 1e-12) return false;

		t = f * edge2.dot(q);
		return t > 1e-9;
	}

	class BVH {
	private:
		struct TriRef {
			int index = -1;
			AABB box;
		};

		struct Node {
			AABB box;
			int left = -1;
			int right = -1;
			std::vector<int> triangles;
		};

		const Mesh* mesh_ = nullptr;
		std::vector<TriRef> triRefs_;
		std::vector<Node> nodes_;

		int build(int begin, int end) {
			Node node;
			for (int i = begin; i < end; ++i) {
				node.box.minX = (std::min)(node.box.minX, triRefs_[i].box.minX);
				node.box.minY = (std::min)(node.box.minY, triRefs_[i].box.minY);
				node.box.minZ = (std::min)(node.box.minZ, triRefs_[i].box.minZ);
				node.box.maxX = (std::max)(node.box.maxX, triRefs_[i].box.maxX);
				node.box.maxY = (std::max)(node.box.maxY, triRefs_[i].box.maxY);
				node.box.maxZ = (std::max)(node.box.maxZ, triRefs_[i].box.maxZ);
			}

			int nodeIndex = static_cast<int>(nodes_.size());
			nodes_.push_back(std::move(node));

			if (end - begin <= 8) {
				for (int i = begin; i < end; ++i) nodes_[nodeIndex].triangles.push_back(triRefs_[i].index);
				return nodeIndex;
			}

			int axis = 0;
			if (extent(nodes_[nodeIndex].box, 1) > extent(nodes_[nodeIndex].box, axis)) axis = 1;
			if (extent(nodes_[nodeIndex].box, 2) > extent(nodes_[nodeIndex].box, axis)) axis = 2;

			int mid = begin + (end - begin) / 2;
			std::nth_element(triRefs_.begin() + begin, triRefs_.begin() + mid, triRefs_.begin() + end,
				[axis](const TriRef& lhs, const TriRef& rhs) {
					return centroidAxis(lhs.box, axis) < centroidAxis(rhs.box, axis);
				});

			nodes_[nodeIndex].left = build(begin, mid);
			nodes_[nodeIndex].right = build(mid, end);
			return nodeIndex;
		}

		void collectHits(int nodeIndex, const Point& origin, const Vec3& direction, std::vector<double>& hits) const {
			if (nodeIndex < 0) return;

			double tMin = 0.0;
			double tMax = 0.0;
			const Node& node = nodes_[static_cast<std::size_t>(nodeIndex)];
			if (!rayIntersectsAABB(origin, direction, node.box, tMin, tMax)) return;

			if (node.left < 0 && node.right < 0) {
				for (int triIndex : node.triangles) {
					const Triangle* tri = mesh_->triangle(triIndex);
					double t = 0.0;
					if (tri && rayIntersectsTriangle(origin, direction, *tri, t)) hits.push_back(t);
				}
				return;
			}

			collectHits(node.left, origin, direction, hits);
			collectHits(node.right, origin, direction, hits);
		}

	public:
		explicit BVH(const Mesh& mesh) : mesh_(&mesh) {
			triRefs_.reserve(mesh.triangleCount());
			for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
				const Triangle* tri = mesh.triangle(static_cast<int>(i));
				if (!tri) continue;

				TriRef ref;
				ref.index = static_cast<int>(i);
				for (int j = 0; j < 3; ++j) {
					const Vertex* vertex = tri->vertex(j);
					if (vertex) ref.box.expand(*vertex);
				}
				triRefs_.push_back(ref);
			}

			if (!triRefs_.empty()) build(0, static_cast<int>(triRefs_.size()));
		}

		bool containsPoint(const Point& point) const {
			if (nodes_.empty()) return false;

			const std::array<Vec3, 3> directions = {
				Vec3(1.0, 0.0, 0.0),
				Vec3(0.0, 1.0, 0.0),
				Vec3(0.0, 0.0, 1.0)
			};

			int insideVotes = 0;
			for (const Vec3& direction : directions) {
				std::vector<double> hits;
				collectHits(0, point, direction, hits);
				std::sort(hits.begin(), hits.end());

				std::vector<double> uniqueHits;
				for (double hit : hits) {
					if (uniqueHits.empty() || std::abs(uniqueHits.back() - hit) > 1e-7) {
						uniqueHits.push_back(hit);
					}
				}

				if (uniqueHits.size() % 2 == 1) ++insideVotes;
			}

			return insideVotes >= 2;
		}
	};

	inline std::vector<Point> samplePoints(const Mesh& mesh) {
		std::vector<Point> points;
		const std::size_t sampleCount = (std::min<std::size_t>)(mesh.vertexCount(), 9);
		for (std::size_t i = 0; i < sampleCount; ++i) {
			std::size_t index = mesh.vertexCount() <= 1
				? 0
				: (i * (mesh.vertexCount() - 1)) / (sampleCount - 1);
			const Vertex* vertex = mesh.findByIndex(static_cast<int>(index));
			if (vertex) points.emplace_back(vertex->x, vertex->y, vertex->z);
		}
		return points;
	}

	inline bool isContainedIn(const Mesh& inner, const BVH& outerBvh) {
		std::vector<Point> points = samplePoints(inner);
		if (points.empty()) return false;

		int inside = 0;
		for (const Point& point : points) {
			if (outerBvh.containsPoint(point)) ++inside;
		}
		return inside == static_cast<int>(points.size());
	}

	inline std::vector<std::size_t> findOuterShellIndices(const std::vector<Mesh>& shells, double aabbEps = 1e-7) {
		std::vector<AABB> boxes;
		boxes.reserve(shells.size());
		for (const Mesh& shell : shells) boxes.push_back(computeAABB(shell));

		std::vector<BVH> bvhs;
		bvhs.reserve(shells.size());
		for (const Mesh& shell : shells) bvhs.emplace_back(shell);

		std::vector<bool> hasParent(shells.size(), false);
		for (std::size_t inner = 0; inner < shells.size(); ++inner) {
			for (std::size_t outer = 0; outer < shells.size(); ++outer) {
				if (inner == outer) continue;
				if (!overlapsAABB(boxes[inner], boxes[outer], aabbEps)) continue;
				if (!containsAABB(boxes[outer], boxes[inner], aabbEps)) continue;

				if (isContainedIn(shells[inner], bvhs[outer])) {
					hasParent[inner] = true;
					break;
				}
			}
		}

		std::vector<std::size_t> result;
		for (std::size_t i = 0; i < hasParent.size(); ++i) {
			if (!hasParent[i]) result.push_back(i);
		}
		return result;
	}

	inline Mesh mergeShells(const std::vector<Mesh>& shells, const std::vector<std::size_t>& keep, double weldTolerance = 1e-7) {
		Mesh result;
		std::unordered_map<QuantizedVertexKey, int, QuantizedVertexKeyHash> vertexMap;
		const double effectiveTolerance = weldTolerance > 0.0 ? weldTolerance : 1e-12;

		const auto mapVertex = [&](const Vertex& vertex) {
			QuantizedVertexKey key{
				static_cast<long long>(std::llround(vertex.x / effectiveTolerance)),
				static_cast<long long>(std::llround(vertex.y / effectiveTolerance)),
				static_cast<long long>(std::llround(vertex.z / effectiveTolerance))
			};

			auto found = vertexMap.find(key);
			if (found != vertexMap.end()) return found->second;

			int index = result.addVertex(vertex.x, vertex.y, vertex.z)->index;
			vertexMap.emplace(key, index);
			return index;
			};

		for (std::size_t shellIndex : keep) {
			const Mesh& shell = shells[shellIndex];
			for (std::size_t i = 0; i < shell.triangleCount(); ++i) {
				const Triangle* tri = shell.triangle(static_cast<int>(i));
				if (!tri) continue;

				const Vertex* v0 = tri->vertex(0);
				const Vertex* v1 = tri->vertex(1);
				const Vertex* v2 = tri->vertex(2);
				if (!v0 || !v1 || !v2) continue;

				result.addTriangle(mapVertex(*v0), mapVertex(*v1), mapVertex(*v2));
			}
		}

		return result;
	}

} // namespace mesh_outer
