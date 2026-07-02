#include<iostream>
#include<Windows.h>
#include<fstream>
#include<string>
#include<memory>
//#include "utils.hxx"
#include "mesh.hxx"

// 通用工厂函数：用 make_shared 构造任意类型
template <typename T, typename... Args>
std::shared_ptr<T> MakeShared(Args&&... args) {
	return std::make_shared<T>(std::forward<Args>(args)...);
}

int main() {
	std::string filePath = "D:/Code/c/ObjTmp/a.obj";
	std::string outputPath = "D:/Code/c/ObjTmp/exported.obj";
	std::string basePath = "D:/Code/c/ObjTmp/";
	auto mesh = MakeShared<Mesh>(filePath);
	try {
		std::cout << "load success" << std::endl;
		std::cout << "vertex count: " << mesh->vertexCount() << std::endl;
		std::cout << "edge count: " << mesh->edgeCount() << std::endl;
		std::cout << "triangle count: " << mesh->triangleCount() << std::endl;
		if (!mesh->parseErrors().empty()) {
			std::cout << "parse warnings: " << mesh->parseErrors().size() << std::endl;
			for (const auto& err : mesh->parseErrors()) {
				std::cout << "  " << err << std::endl;
			}
		}
	}
	catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		return 1;
	}
	//mesh->printEdgeUsageSummary();
	auto log = mesh->validateBasicTopology();
	std::cout << "\n=== Topology Validation ===" << std::endl;
	std::cout << "invalid vertices: " << log.invalidVertexIndices.size() << std::endl;
	std::cout << "invalid triangles: " << log.invalidTriangleIndices.size() << std::endl;
	std::cout << "degenerate triangles: " << log.degenerateTriangles.size() << std::endl;
	std::cout << "triangle-edge mismatches: " << log.triangleEdgeMismatch.size() << std::endl;
	std::cout << "broken opposite edges: " << log.brokenOppositeEdges.size() << std::endl;
	std::cout << "validation ok: " << (log.ok() ? "YES" : "NO") << std::endl;

	auto check = mesh->checkManifoldAndWatertight();
	std::cout << "\n=== Manifold & Watertight Check ===" << std::endl;
	std::cout << "boundary edges: " << check.boundaryEdgeCount << std::endl;
	std::cout << "non-manifold edges: " << check.nonManifoldEdgeCount << std::endl;
	std::cout << "inconsistent orientation edges: " << check.inconsistentOrientationEdgeCount << std::endl;
	std::cout << "non-manifold vertices: " << check.nonManifoldVertexCount << std::endl;
	std::cout << "isolated vertices: " << check.isolatedVertexCount << std::endl;
	std::cout << "degenerate triangles: " << check.degenerateTriangleCount << std::endl;

	// ===== 姿态变换演示 =====
	std::cout << "\n=== Transform Demo ===" << std::endl;

	// 记录第一个顶点变换前的坐标
	Vertex* v0 = mesh->findByIndex(0);
	if (v0) {
		std::cout << "v0 before: (" << v0->x << ", " << v0->y << ", " << v0->z << ")" << std::endl;
	}

	// 平移：沿 X 轴移动 10 单位
	mesh->translate(10.0, 0.0, 0.0);
	if (v0) {
		std::cout << "v0 after translate(10,0,0): (" << v0->x << ", " << v0->y << ", " << v0->z << ")" << std::endl;
	}

	// 用四元数绕 Y 轴旋转 90°
	Quat q = Quat::fromAxisAngle(Vec3(0, 1, 0), 3.14159265358979 / 2.0);
	mesh->rotate(q);
	if (v0) {
		std::cout << "v0 after rotate Y 90deg: (" << v0->x << ", " << v0->y << ", " << v0->z << ")" << std::endl;
	}

	// 导出变换后的网格
	mesh->exportObj(outputPath);
	std::cout << "exported transformed mesh to: " << outputPath << std::endl;
	/*std::vector<std::vector<Vertex*>> components = mesh->connectedVertexComponents();
	for (std::size_t i = 0; i < components.size(); ++i) {
		auto tris = mesh->collectTriangleFromVertexComponent(components[i]);

		std::cout << "component " << i
			<< " vertex count = " << components[i].size()
			<< ", triangle count = " << tris.size()
			<< std::endl;
	}
	std::vector<Mesh> meshs = mesh->splitComponents();
	int index = 0;
	for (std::vector<Mesh>::iterator it = meshs.begin(); it != meshs.end(); it++) {
		it->exportObj(basePath + std::to_string(index) + ".obj");
		index++;
	}
	std::vector<std::vector<Triangle*>> tricomps = mesh->connectedTriangleComponents();
	mesh->exportObj(outputPath);*/



	return 0;
}