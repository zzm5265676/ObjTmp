#include<iostream>
#include<Windows.h>
#include<fstream>
#include<string>
#include<memory>
//#include "utils.hxx"
#include "mesh.hxx"
#include "creation.hxx"
#include "repair.hxx"
#include "bvh.hxx"
#include "intersect.hxx"
#include "csg.hxx"

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
	// Structural validation
	auto log = mesh->validateBasicTopology();
	std::cout << "\n=== Topology Validation ===" << std::endl;
	std::cout << "invalid vertices: " << log.invalidVertexIndices.size() << std::endl;
	std::cout << "invalid triangles: " << log.invalidTriangleIndices.size() << std::endl;
	std::cout << "degenerate triangles: " << log.degenerateTriangles.size() << std::endl;
	std::cout << "triangle-edge mismatches: " << log.triangleEdgeMismatch.size() << std::endl;
	std::cout << "broken opposite edges: " << log.brokenOppositeEdges.size() << std::endl;
	std::cout << "validation ok: " << (log.ok() ? "YES" : "NO") << std::endl;

	// Full validation (watertight, manifold, oriented, degenerate, self-intersection)
	auto check = mesh->validateAll();
	std::cout << "\n=== Full Validation ===" << std::endl;
	std::cout << "boundary edges: " << check.boundaryEdgeCount << std::endl;
	std::cout << "non-manifold edges: " << check.nonManifoldEdgeCount << std::endl;
	std::cout << "inconsistent orientation edges: " << check.inconsistentOrientationEdgeCount << std::endl;
	std::cout << "non-manifold vertices: " << check.nonManifoldVertexCount << std::endl;
	std::cout << "isolated vertices: " << check.isolatedVertexCount << std::endl;
	std::cout << "degenerate triangles: " << check.degenerateTriangleCount << std::endl;
	std::cout << "self-intersecting pairs: " << check.selfIntersectingTriangleCount << std::endl;
	std::cout << "\n--- Flags ---" << std::endl;
	std::cout << "watertight: " << (check.isWatertight() ? "YES" : "NO") << std::endl;
	std::cout << "manifold: " << (check.isManifold() ? "YES" : "NO") << std::endl;
	std::cout << "oriented: " << (check.isOriented() ? "YES" : "NO") << std::endl;
	std::cout << "degenerate-free: " << (check.isDegenerateFree() ? "YES" : "NO") << std::endl;
	std::cout << "self-intersection-free: " << (!check.hasSelfIntersection() ? "YES" : "NO") << std::endl;
	std::cout << "all ok: " << (check.ok() ? "YES" : "NO") << std::endl;

	// ===== Normal Computation Demo =====
	std::cout << "\n=== Normal Computation ===" << std::endl;

	// Face normal of first triangle
	if (mesh->triangleCount() > 0) {
		Triangle* tri = mesh->findByIndex(0) ? nullptr : nullptr; // just use edges
		std::cout << "face normal of tri#0: " << mesh->triangleCount() << " triangles" << std::endl;
	}

	// Vertex normals with different methods
	auto vnSimple   = mesh->computeVertexNormals(NormalComputer::VertexNormalMethod::Simple);
	auto vnArea     = mesh->computeVertexNormals(NormalComputer::VertexNormalMethod::AreaWeighted);
	auto vnAngle    = mesh->computeVertexNormals(NormalComputer::VertexNormalMethod::AngleWeighted);
	auto vnEdges    = mesh->computeVertexNormals(NormalComputer::VertexNormalMethod::FromEdges);

	// Show first vertex normal with each method
	if (mesh->vertexCount() > 0) {
		std::cout << "v0 normal (simple):       (" << vnSimple[0].x << ", " << vnSimple[0].y << ", " << vnSimple[0].z << ")" << std::endl;
		std::cout << "v0 normal (area-weighted): (" << vnArea[0].x << ", " << vnArea[0].y << ", " << vnArea[0].z << ")" << std::endl;
		std::cout << "v0 normal (angle-weighted):(" << vnAngle[0].x << ", " << vnAngle[0].y << ", " << vnAngle[0].z << ")" << std::endl;
		std::cout << "v0 normal (from edges):    (" << vnEdges[0].x << ", " << vnEdges[0].y << ", " << vnEdges[0].z << ")" << std::endl;
	}

	// Edge normals
	auto en = mesh->computeEdgeNormals();
	std::cout << "edge normals computed: " << en.size() << std::endl;
	if (mesh->edgeCount() > 0) {
		std::cout << "edge#0 normal: (" << en[0].x << ", " << en[0].y << ", " << en[0].z << ")" << std::endl;
	}

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

	// 归一化到 [-1, 1]
	auto [bbMin, bbMax] = mesh->boundingBox();
	std::cout << "\n=== Normalize to Unit ===" << std::endl;
	std::cout << "bbox before: min(" << bbMin.x << ", " << bbMin.y << ", " << bbMin.z
		<< ") max(" << bbMax.x << ", " << bbMax.y << ", " << bbMax.z << ")" << std::endl;

	mesh->normalizeToUnit();

	auto [bbMin2, bbMax2] = mesh->boundingBox();
	std::cout << "bbox after:  min(" << bbMin2.x << ", " << bbMin2.y << ", " << bbMin2.z
		<< ") max(" << bbMax2.x << ", " << bbMax2.y << ", " << bbMax2.z << ")" << std::endl;

	mesh->exportObj(basePath + "normalized.obj");
	std::cout << "exported normalized mesh to: " << basePath << "normalized.obj" << std::endl;
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

	// ===== Creation API Demo =====
	std::cout << "\n=== Creation API Demo ===" << std::endl;

	// Box
	auto box = mesh_creation::api_make_box(2.0, 1.0, 1.5);
	std::cout << "box: " << box.vertexCount() << " verts, " << box.triangleCount() << " tris" << std::endl;
	box.exportObj(basePath + "prim_box.obj");

	// Box from two points
	auto box2 = mesh_creation::api_make_box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
	std::cout << "box2: " << box2.vertexCount() << " verts, " << box2.triangleCount() << " tris" << std::endl;

	// Sphere
	auto sphere = mesh_creation::api_make_sphere(1.0, 32);
	std::cout << "sphere(32): " << sphere.vertexCount() << " verts, " << sphere.triangleCount() << " tris" << std::endl;
	sphere.exportObj(basePath + "prim_sphere.obj");

	// Torus
	auto torus = mesh_creation::api_make_torus(1.0, 0.3, 32);
	std::cout << "torus(32): " << torus.vertexCount() << " verts, " << torus.triangleCount() << " tris" << std::endl;
	torus.exportObj(basePath + "prim_torus.obj");

	// Prism (hexagonal)
	auto prism = mesh_creation::api_make_prism(6, 2.0, 1.0, 1.0);
	std::cout << "prism(6): " << prism.vertexCount() << " verts, " << prism.triangleCount() << " tris" << std::endl;
	prism.exportObj(basePath + "prim_prism.obj");

	// Pyramid
	auto pyramid = mesh_creation::api_make_pyramid(5, 1.0, 2.0);
	std::cout << "pyramid(5): " << pyramid.vertexCount() << " verts, " << pyramid.triangleCount() << " tris" << std::endl;
	pyramid.exportObj(basePath + "prim_pyramid.obj");

	// Cone
	auto cone = mesh_creation::api_make_cone(1.0, 2.0, 32);
	std::cout << "cone(32): " << cone.vertexCount() << " verts, " << cone.triangleCount() << " tris" << std::endl;
	cone.exportObj(basePath + "prim_cone.obj");

	// Frustum
	auto frustum = mesh_creation::api_make_frustum(1.0, 0.5, 2.0, 32);
	std::cout << "frustum(32): " << frustum.vertexCount() << " verts, " << frustum.triangleCount() << " tris" << std::endl;
	frustum.exportObj(basePath + "prim_frustum.obj");

	// Verify normal direction: for sphere centered at origin,
	// vertex normal should point away from center (same direction as position)
	{
		auto vn = sphere.computeVertexNormals(NormalComputer::VertexNormalMethod::Simple);
		int outward = 0, inward = 0;
		for (std::size_t i = 0; i < sphere.vertexCount(); ++i) {
			Vertex* v = sphere.findByIndex(static_cast<int>(i));
			if (!v) continue;
			double dot = v->x * vn[i].x + v->y * vn[i].y + v->z * vn[i].z;
			if (dot > 0) outward++; else inward++;
		}
		std::cout << "\n--- Normal Direction Check (sphere) ---" << std::endl;
		std::cout << "outward: " << outward << ", inward: " << inward << std::endl;
		std::cout << "normals are " << (inward == 0 ? "CORRECT (all outward)" : "INVERTED!") << std::endl;
	}

	// Verify watertightness
	std::cout << "\n--- Primitive Validation ---" << std::endl;
	auto boxCheck = box.validateAll();
	std::cout << "box: watertight=" << boxCheck.isWatertight()
		<< " manifold=" << boxCheck.isManifold() << std::endl;
	auto sphereCheck = sphere.validateAll();
	std::cout << "sphere: watertight=" << sphereCheck.isWatertight()
		<< " manifold=" << sphereCheck.isManifold() << std::endl;
	auto torusCheck = torus.validateAll();
	std::cout << "torus: watertight=" << torusCheck.isWatertight()
		<< " manifold=" << torusCheck.isManifold() << std::endl;

	// ===== Geometry Utils Demo =====
	std::cout << "\n=== Geometry Utils ===" << std::endl;

	// orient3D
	Point pa(0,0,0), pb(1,0,0), pc(0,1,0), pd(0,0,1);
	double orient = geo::orient3D(pa, pb, pc, pd);
	std::cout << "orient3D(above): " << orient << " (positive = above plane)" << std::endl;

	// AABB
	auto boxAABB = geo::computeTriangleAABB(box.triangle(0));
	std::cout << "box tri#0 AABB: min(" << boxAABB.minX << "," << boxAABB.minY << "," << boxAABB.minZ
		<< ") max(" << boxAABB.maxX << "," << boxAABB.maxY << "," << boxAABB.maxZ << ")" << std::endl;

	// Point classification (sphere centered at origin, radius 1)
	{
		auto c1 = classifyPointInMesh(Point(0,0,0), sphere);
		auto c2 = classifyPointInMesh(Point(5,0,0), sphere);
		std::cout << "sphere: (0,0,0) = " << (c1 == geo::PointClass::Inside ? "Inside" : (c1 == geo::PointClass::OnBoundary ? "OnBoundary" : "Outside")) << std::endl;
		std::cout << "sphere: (5,0,0) = " << (c2 == geo::PointClass::Inside ? "Inside" : (c2 == geo::PointClass::OnBoundary ? "OnBoundary" : "Outside")) << std::endl;
	}

	// ===== Mesh Repair Demo =====
	std::cout << "\n=== Mesh Repair ===" << std::endl;
	{
		auto repairBox = mesh_creation::api_make_box(2, 1, 1);
		auto report = mesh_repair::repair(repairBox);
		std::cout << "repair box: merged=" << report.mergedVertices
			<< " degenerate=" << report.removedDegenerate
			<< " flipped=" << report.flippedFaces << std::endl;
	}

	// ===== BVH Demo =====
	std::cout << "\n=== BVH ===" << std::endl;
	{
		geo::BVH bvh(sphere);
		std::cout << "BVH built for sphere (" << sphere.triangleCount() << " tris)" << std::endl;

		// Ray query
		Point origin(0, 0, 0);
		Vec3 dir(1, 0, 0);
		Point hit;
		int triIdx;
		if (bvh.raycast(origin, dir, 10.0, hit, triIdx)) {
			std::cout << "ray hit at (" << hit.x << "," << hit.y << "," << hit.z
				<< ") tri#" << triIdx << std::endl;
		}

		// Closest point query
		Point query(2, 0, 0);
		int closestTri;
		Point closest = bvh.closestPoint(query, closestTri);
		std::cout << "closest to (2,0,0): (" << closest.x << "," << closest.y << "," << closest.z
			<< ") tri#" << closestTri << std::endl;
	}

	// ===== CSG Boolean Tests =====
	std::cout << "\n=== CSG Boolean Tests ===" << std::endl;

	// Helper lambda to run and export a boolean test
	auto runTest = [&](const std::string& name, Mesh& a, Mesh& b) {
		std::cout << "\n--- " << name << " ---" << std::endl;
		std::cout << "A: " << a.vertexCount() << " verts, " << a.triangleCount() << " tris" << std::endl;
		std::cout << "B: " << b.vertexCount() << " verts, " << b.triangleCount() << " tris" << std::endl;

		auto u = csg::meshUnion(a, b);
		auto i = csg::meshIntersection(a, b);
		auto d = csg::meshDifference(a, b);

		if (u.success) {
			u.mesh.exportObj(basePath + name + "_union.obj");
			std::cout << "union:        " << u.mesh.vertexCount() << " verts, "
				<< u.mesh.triangleCount() << " tris" << std::endl;
		}
		if (i.success) {
			i.mesh.exportObj(basePath + name + "_intersect.obj");
			std::cout << "intersection: " << i.mesh.vertexCount() << " verts, "
				<< i.mesh.triangleCount() << " tris" << std::endl;
		}
		if (d.success) {
			d.mesh.exportObj(basePath + name + "_diff.obj");
			std::cout << "difference:   " << d.mesh.vertexCount() << " verts, "
				<< d.mesh.triangleCount() << " tris" << std::endl;
		}
	};

	// Test 1: Two overlapping boxes (translate)
	{
		auto a = mesh_creation::api_make_box(2, 2, 2);
		auto b = mesh_creation::api_make_box(2, 2, 2);
		b.translate(1, 0, 0);
		runTest("box_box", a, b);
	}

	// Test 2: Box and sphere (sphere inside box)
	{
		auto a = mesh_creation::api_make_box(2, 2, 2);
		auto b = mesh_creation::api_make_sphere(0.8, 24);
		runTest("box_sphere", a, b);
	}

	// Test 3: Two spheres (overlapping)
	{
		auto a = mesh_creation::api_make_sphere(1.0, 24);
		auto b = mesh_creation::api_make_sphere(1.0, 24);
		b.translate(0.8, 0, 0);
		runTest("sphere_sphere", a, b);
	}

	// Test 4: Box and rotated prism
	{
		auto a = mesh_creation::api_make_box(2, 2, 2);
		auto b = mesh_creation::api_make_prism(6, 3, 0.8, 0.8);
		b.rotateEuler(0, 0, 3.14159 / 4.0);  // 45 degree rotation
		runTest("box_prism_rotated", a, b);
	}

	// Test 5: Torus and sphere (complex intersection)
	{
		auto a = mesh_creation::api_make_torus(1.5, 0.5, 24);
		auto b = mesh_creation::api_make_sphere(1.0, 24);
		runTest("torus_sphere", a, b);
	}

	// Test 6: Cone inside box
	{
		auto a = mesh_creation::api_make_box(2, 2, 2);
		auto b = mesh_creation::api_make_cone(0.8, 2.5, 24);
		b.translate(0, -0.25, 0);
		runTest("box_cone", a, b);
	}

	// Test 7: Frustum and sphere (offset and rotated)
	{
		auto a = mesh_creation::api_make_frustum(1.0, 0.5, 2.0, 24);
		auto b = mesh_creation::api_make_sphere(0.7, 24);
		b.translate(0.5, 0.5, 0);
		b.rotateEuler(0.3, 0.5, 0);
		runTest("frustum_sphere", a, b);
	}

	// Test 8: Two boxes at angle (edge intersection)
	{
		auto a = mesh_creation::api_make_box(3, 1, 1);
		auto b = mesh_creation::api_make_box(3, 1, 1);
		b.rotateEuler(0, 0, 3.14159 / 3.0);  // 60 degrees
		runTest("box_box_angle", a, b);
	}

	std::cout << "\n=== All tests complete ===" << std::endl;
	std::cout << "Exported OBJ files to: " << basePath << std::endl;

	return 0;
}