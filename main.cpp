#include<iostream>
#include<Windows.h>
#include<fstream>
#include<string>
#include<memory>
#include "config.hxx"
#include "mesh/mesh.hxx"
#include "operations/creation.hxx"
#include "operations/repair.hxx"
#include "geometry/bvh.hxx"
#include "operations/intersect.hxx"
#include "operations/csg.hxx"
#include "operations/smooth.hxx"
#include "operations/decimate.hxx"
#include "geometry/curvature.hxx"
#include "io/ply.hxx"

//  make_shared 
template <typename T, typename... Args>
std::shared_ptr<T> MakeShared(Args&&... args) {
	return std::make_shared<T>(std::forward<Args>(args)...);
}

int main() {
	std::string filePath = config::INPUT_DIR + "jiaban.obj";

	std::string outputPath = config::OUTPUT_DIR + "exported.obj";
	std::string basePath = config::OUTPUT_DIR;
	try {
		std::cout << "loading: " << filePath << std::endl;
		auto mesh = MakeShared<Mesh>(filePath);
		std::cout << "load success" << std::endl;
		std::cout << "vertex count: " << mesh->vertexCount() << std::endl;
		std::cout << "edge count: " << mesh->edgeCount() << std::endl;
		std::cout << "triangle count: " << mesh->triangleCount() << std::endl;
		if (!mesh->parseErrors().empty()) {
			std::cout << "parse warnings: " << mesh->parseErrors().size() << std::endl;
		}

		std::cout << "splitting components..." << std::endl;
		std::vector<Mesh> seshs = mesh->splitComponents();
		std::cout << "component count: " << seshs.size() << std::endl;
	}
	catch (const std::exception& e) {
		std::cerr << "error: " << e.what() << std::endl;
		return 1;
	}
	//try {
	//	std::cout << "load success" << std::endl;
	//	std::cout << "vertex count: " << mesh->vertexCount() << std::endl;
	//	std::cout << "edge count: " << mesh->edgeCount() << std::endl;
	//	std::cout << "triangle count: " << mesh->triangleCount() << std::endl;
	//	if (!mesh->parseErrors().empty()) {
	//		std::cout << "parse warnings: " << mesh->parseErrors().size() << std::endl;
	//		for (const auto& err : mesh->parseErrors()) {
	//			std::cout << "  " << err << std::endl;
	//		}
	//	}
	//}
	//catch (const std::exception& e) {
	//	std::cout << e.what() << std::endl;
	//	return 1;
	//}
	////mesh->printEdgeUsageSummary();
	//// Structural validation
	//auto log = mesh->validateBasicTopology();
	//std::cout << "\n=== Topology Validation ===" << std::endl;
	//std::cout << "invalid vertices: " << log.invalidVertexIndices.size() << std::endl;
	//std::cout << "invalid triangles: " << log.invalidTriangleIndices.size() << std::endl;
	//std::cout << "degenerate triangles: " << log.degenerateTriangles.size() << std::endl;
	//std::cout << "triangle-edge mismatches: " << log.triangleEdgeMismatch.size() << std::endl;
	//std::cout << "broken opposite edges: " << log.brokenOppositeEdges.size() << std::endl;
	//std::cout << "validation ok: " << (log.ok() ? "YES" : "NO") << std::endl;

	//// Full validation
	//auto check = mesh->validateAll();
	//std::cout << "\n=== Full Validation ===" << std::endl;
	//std::cout << "boundary edges: " << check.boundaryEdgeCount << std::endl;
	//std::cout << "non-manifold edges: " << check.nonManifoldEdgeCount << std::endl;
	//std::cout << "inconsistent orientation edges: " << check.inconsistentOrientationEdgeCount << std::endl;
	//std::cout << "non-manifold vertices: " << check.nonManifoldVertexCount << std::endl;
	//std::cout << "isolated vertices: " << check.isolatedVertexCount << std::endl;
	//std::cout << "degenerate triangles: " << check.degenerateTriangleCount << std::endl;
	//std::cout << "self-intersecting pairs: " << check.selfIntersectingTriangleCount << std::endl;
	//std::cout << "connected components: " << check.connectedComponentCount << std::endl;
	//std::cout << "Euler characteristic: " << check.eulerCharacteristic << std::endl;
	//std::cout << "sliver triangles: " << check.sliverTriangleCount << std::endl;
	//std::cout << "needle triangles: " << check.needleTriangleCount << std::endl;
	//std::cout << "cap triangles: " << check.capTriangleCount << std::endl;
	//std::cout << "\n--- Flags ---" << std::endl;
	//std::cout << "watertight: " << (check.isWatertight() ? "YES" : "NO") << std::endl;
	//std::cout << "manifold: " << (check.isManifold() ? "YES" : "NO") << std::endl;
	//std::cout << "oriented: " << (check.isOriented() ? "YES" : "NO") << std::endl;
	//std::cout << "degenerate-free: " << (check.isDegenerateFree() ? "YES" : "NO") << std::endl;
	//std::cout << "self-intersection-free: " << (!check.hasSelfIntersection() ? "YES" : "NO") << std::endl;
	//std::cout << "all ok: " << (check.ok() ? "YES" : "NO") << std::endl;

	//// ===== Normal Computation Demo =====
	//std::cout << "\n=== Normal Computation ===" << std::endl;

	//// Face normal of first triangle
	//if (mesh->triangleCount() > 0) {
	//	Triangle* tri = mesh->findByIndex(0) ? nullptr : nullptr; // just use edges
	//	std::cout << "face normal of tri#0: " << mesh->triangleCount() << " triangles" << std::endl;
	//}

	//// Vertex normals with different methods
	//auto vnSimple = mesh->computeVertexNormals(NormalComputer::VertexNormalMethod::Simple);
	//auto vnArea = mesh->computeVertexNormals(NormalComputer::VertexNormalMethod::AreaWeighted);
	//auto vnAngle = mesh->computeVertexNormals(NormalComputer::VertexNormalMethod::AngleWeighted);
	//auto vnEdges = mesh->computeVertexNormals(NormalComputer::VertexNormalMethod::FromEdges);

	//// Show first vertex normal with each method
	//if (mesh->vertexCount() > 0) {
	//	std::cout << "v0 normal (simple):       (" << vnSimple[0].x << ", " << vnSimple[0].y << ", " << vnSimple[0].z << ")" << std::endl;
	//	std::cout << "v0 normal (area-weighted): (" << vnArea[0].x << ", " << vnArea[0].y << ", " << vnArea[0].z << ")" << std::endl;
	//	std::cout << "v0 normal (angle-weighted):(" << vnAngle[0].x << ", " << vnAngle[0].y << ", " << vnAngle[0].z << ")" << std::endl;
	//	std::cout << "v0 normal (from edges):    (" << vnEdges[0].x << ", " << vnEdges[0].y << ", " << vnEdges[0].z << ")" << std::endl;
	//}

	//// Edge normals
	//auto en = mesh->computeEdgeNormals();
	//std::cout << "edge normals computed: " << en.size() << std::endl;
	//if (mesh->edgeCount() > 0) {
	//	std::cout << "edge#0 normal: (" << en[0].x << ", " << en[0].y << ", " << en[0].z << ")" << std::endl;
	//}

	//// =====  =====
	//std::cout << "\n=== Transform Demo ===" << std::endl;

	//// 
	//Vertex* v0 = mesh->findByIndex(0);
	//if (v0) {
	//	std::cout << "v0 before: (" << v0->x << ", " << v0->y << ", " << v0->z << ")" << std::endl;
	//}

	////  X  10 
	//mesh->translate(10.0, 0.0, 0.0);
	//if (v0) {
	//	std::cout << "v0 after translate(10,0,0): (" << v0->x << ", " << v0->y << ", " << v0->z << ")" << std::endl;
	//}

	////  Y  90
	//Quat q = Quat::fromAxisAngle(Vec3(0, 1, 0), 3.14159265358979 / 2.0);
	//mesh->rotate(q);
	//if (v0) {
	//	std::cout << "v0 after rotate Y 90deg: (" << v0->x << ", " << v0->y << ", " << v0->z << ")" << std::endl;
	//}

	//// 
	//mesh->exportObj(outputPath);
	//std::cout << "exported transformed mesh to: " << outputPath << std::endl;

	////  [-1, 1]
	//auto [bbMin, bbMax] = mesh->boundingBox();
	//std::cout << "\n=== Normalize to Unit ===" << std::endl;
	//std::cout << "bbox before: min(" << bbMin.x << ", " << bbMin.y << ", " << bbMin.z
	//	<< ") max(" << bbMax.x << ", " << bbMax.y << ", " << bbMax.z << ")" << std::endl;

	//mesh->normalizeToUnit();

	//auto [bbMin2, bbMax2] = mesh->boundingBox();
	//std::cout << "bbox after:  min(" << bbMin2.x << ", " << bbMin2.y << ", " << bbMin2.z
	//	<< ") max(" << bbMax2.x << ", " << bbMax2.y << ", " << bbMax2.z << ")" << std::endl;

	//mesh->exportObj(basePath + "normalized.obj");
	//std::cout << "exported normalized mesh to: " << basePath << "normalized.obj" << std::endl;
	///*std::vector<std::vector<Vertex*>> components = mesh->connectedVertexComponents();
	//for (std::size_t i = 0; i < components.size(); ++i) {
	//	auto tris = mesh->collectTriangleFromVertexComponent(components[i]);

	//	std::cout << "component " << i
	//		<< " vertex count = " << components[i].size()
	//		<< ", triangle count = " << tris.size()
	//		<< std::endl;
	//}
	//std::vector<Mesh> meshs = mesh->splitComponents();
	//int index = 0;
	//for (std::vector<Mesh>::iterator it = meshs.begin(); it != meshs.end(); it++) {
	//	it->exportObj(basePath + std::to_string(index) + ".obj");
	//	index++;
	//}
	//std::vector<std::vector<Triangle*>> tricomps = mesh->connectedTriangleComponents();
	//mesh->exportObj(outputPath);*/

	//// ===== Creation API Demo =====
	//std::cout << "\n=== Creation API Demo ===" << std::endl;

	//// Box
	//auto box = mesh_creation::api_make_box(2.0, 1.0, 1.5);
	//std::cout << "box: " << box.vertexCount() << " verts, " << box.triangleCount() << " tris" << std::endl;
	//box.exportObj(basePath + "prim_box.obj");

	//// Box from two points
	//auto box2 = mesh_creation::api_make_box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
	//std::cout << "box2: " << box2.vertexCount() << " verts, " << box2.triangleCount() << " tris" << std::endl;

	//// Sphere
	//auto sphere = mesh_creation::api_make_sphere(1.0, 32);
	//std::cout << "sphere(32): " << sphere.vertexCount() << " verts, " << sphere.triangleCount() << " tris" << std::endl;
	//sphere.exportObj(basePath + "prim_sphere.obj");

	//// Torus
	//auto torus = mesh_creation::api_make_torus(1.0, 0.3, 32);
	//std::cout << "torus(32): " << torus.vertexCount() << " verts, " << torus.triangleCount() << " tris" << std::endl;
	//torus.exportObj(basePath + "prim_torus.obj");

	//// Prism (hexagonal)
	//auto prism = mesh_creation::api_make_prism(6, 2.0, 1.0, 1.0);
	//std::cout << "prism(6): " << prism.vertexCount() << " verts, " << prism.triangleCount() << " tris" << std::endl;
	//prism.exportObj(basePath + "prim_prism.obj");

	//// Pyramid
	//auto pyramid = mesh_creation::api_make_pyramid(5, 1.0, 2.0);
	//std::cout << "pyramid(5): " << pyramid.vertexCount() << " verts, " << pyramid.triangleCount() << " tris" << std::endl;
	//pyramid.exportObj(basePath + "prim_pyramid.obj");

	//// Cone
	//auto cone = mesh_creation::api_make_cone(1.0, 2.0, 32);
	//std::cout << "cone(32): " << cone.vertexCount() << " verts, " << cone.triangleCount() << " tris" << std::endl;
	//cone.exportObj(basePath + "prim_cone.obj");

	//// Frustum
	//auto frustum = mesh_creation::api_make_frustum(1.0, 0.5, 2.0, 32);
	//std::cout << "frustum(32): " << frustum.vertexCount() << " verts, " << frustum.triangleCount() << " tris" << std::endl;
	//frustum.exportObj(basePath + "prim_frustum.obj");

	//// Verify normal direction: for sphere centered at origin,
	//// vertex normal should point away from center (same direction as position)
	//{
	//	auto vn = sphere.computeVertexNormals(NormalComputer::VertexNormalMethod::Simple);
	//	int outward = 0, inward = 0;
	//	for (std::size_t i = 0; i < sphere.vertexCount(); ++i) {
	//		Vertex* v = sphere.findByIndex(static_cast<int>(i));
	//		if (!v) continue;
	//		double dot = v->x * vn[i].x + v->y * vn[i].y + v->z * vn[i].z;
	//		if (dot > 0) outward++; else inward++;
	//	}
	//	std::cout << "\n--- Normal Direction Check (sphere) ---" << std::endl;
	//	std::cout << "outward: " << outward << ", inward: " << inward << std::endl;
	//	std::cout << "normals are " << (inward == 0 ? "CORRECT (all outward)" : "INVERTED!") << std::endl;
	//}

	//// Verify watertightness
	//std::cout << "\n--- Primitive Validation ---" << std::endl;
	//auto boxCheck = box.validateAll();
	//std::cout << "box: watertight=" << boxCheck.isWatertight()
	//	<< " manifold=" << boxCheck.isManifold()
	//	<< " euler=" << boxCheck.eulerCharacteristic
	//	<< " comp=" << boxCheck.connectedComponentCount << std::endl;
	//auto sphereCheck = sphere.validateAll();
	//std::cout << "sphere: watertight=" << sphereCheck.isWatertight()
	//	<< " manifold=" << sphereCheck.isManifold()
	//	<< " euler=" << sphereCheck.eulerCharacteristic
	//	<< " comp=" << sphereCheck.connectedComponentCount << std::endl;
	//auto torusCheck = torus.validateAll();
	//std::cout << "torus: watertight=" << torusCheck.isWatertight()
	//	<< " manifold=" << torusCheck.isManifold()
	//	<< " euler=" << torusCheck.eulerCharacteristic
	//	<< " comp=" << torusCheck.connectedComponentCount << std::endl;

	//// ===== Geometry Utils Demo =====
	//std::cout << "\n=== Geometry Utils ===" << std::endl;

	//// orient3D
	//Point pa(0, 0, 0), pb(1, 0, 0), pc(0, 1, 0), pd(0, 0, 1);
	//double orient = geo::orient3D(pa, pb, pc, pd);
	//std::cout << "orient3D(above): " << orient << " (positive = above plane)" << std::endl;

	//// AABB
	//auto boxAABB = geo::computeTriangleAABB(box.triangle(0));
	//std::cout << "box tri#0 AABB: min(" << boxAABB.minX << "," << boxAABB.minY << "," << boxAABB.minZ
	//	<< ") max(" << boxAABB.maxX << "," << boxAABB.maxY << "," << boxAABB.maxZ << ")" << std::endl;

	//// Point classification (sphere centered at origin, radius 1)
	//{
	//	auto c1 = classifyPointInMesh(Point(0, 0, 0), sphere);
	//	auto c2 = classifyPointInMesh(Point(5, 0, 0), sphere);
	//	std::cout << "sphere: (0,0,0) = " << (c1 == geo::PointClass::Inside ? "Inside" : (c1 == geo::PointClass::OnBoundary ? "OnBoundary" : "Outside")) << std::endl;
	//	std::cout << "sphere: (5,0,0) = " << (c2 == geo::PointClass::Inside ? "Inside" : (c2 == geo::PointClass::OnBoundary ? "OnBoundary" : "Outside")) << std::endl;
	//}

	//// ===== Mesh Repair Demo =====
	//std::cout << "\n=== Mesh Repair ===" << std::endl;
	//{
	//	auto repairBox = mesh_creation::api_make_box(2, 1, 1);
	//	auto report = mesh_repair::repair(repairBox);
	//	std::cout << "repair box: merged=" << report.mergedVertices
	//		<< " degenerate=" << report.removedDegenerate
	//		<< " flipped=" << report.flippedFaces << std::endl;
	//}

	//// ===== BVH Demo =====
	//std::cout << "\n=== BVH ===" << std::endl;
	//{
	//	geo::BVH bvh(sphere);
	//	std::cout << "BVH built for sphere (" << sphere.triangleCount() << " tris)" << std::endl;

	//	// Ray query
	//	Point origin(0, 0, 0);
	//	Vec3 dir(1, 0, 0);
	//	Point hit;
	//	int triIdx;
	//	if (bvh.raycast(origin, dir, 10.0, hit, triIdx)) {
	//		std::cout << "ray hit at (" << hit.x << "," << hit.y << "," << hit.z
	//			<< ") tri#" << triIdx << std::endl;
	//	}

	//	// Closest point query
	//	Point query(2, 0, 0);
	//	int closestTri;
	//	Point closest = bvh.closestPoint(query, closestTri);
	//	std::cout << "closest to (2,0,0): (" << closest.x << "," << closest.y << "," << closest.z
	//		<< ") tri#" << closestTri << std::endl;
	//}

	//// ===== CSG Boolean Tests =====
	//std::cout << "\n=== CSG Boolean Tests ===" << std::endl;

	//// Helper lambda to run and export a boolean test
	//auto runTest = [&](const std::string& name, Mesh& a, Mesh& b) {
	//	std::cout << "\n--- " << name << " ---" << std::endl;
	//	std::cout << "A: " << a.vertexCount() << " verts, " << a.triangleCount() << " tris" << std::endl;
	//	std::cout << "B: " << b.vertexCount() << " verts, " << b.triangleCount() << " tris" << std::endl;

	//	auto u = csg::meshUnion(a, b);
	//	auto i = csg::meshIntersection(a, b);
	//	auto d = csg::meshDifference(a, b);

	//	auto checkMesh = [&](const std::string& op, csg::CSGResult& r) {
	//		if (!r.success) return;
	//		r.mesh.exportObj(basePath + name + "_" + op + ".obj");
	//		auto chk = r.mesh.validateAll();
	//		std::cout << op << ": " << r.mesh.vertexCount() << "v " << r.mesh.triangleCount() << "t"
	//			<< " watertight=" << chk.isWatertight()
	//			<< " manifold=" << chk.isManifold() << std::endl;
	//		};

	//	checkMesh("union", u);
	//	checkMesh("intersect", i);
	//	checkMesh("diff", d);
	//	};

	//// Test 1: Two overlapping boxes (translate)
	//{
	//	auto a = mesh_creation::api_make_box(2, 2, 2);
	//	auto b = mesh_creation::api_make_box(2, 2, 2);
	//	b.translate(1, 0, 0);

	//	// Debug: check intersection
	//	geo::BVH bvhA(a), bvhB(b);
	//	auto intResult = mesh_intersect::findIntersection(a, b, bvhA, bvhB);
	//	std::cout << "intersection curves: " << intResult.curves.size() << std::endl;
	//	int segCount = 0;
	//	for (auto& c : intResult.curves) segCount += c.segments.size();
	//	std::cout << "intersection segments: " << segCount << std::endl;

	//	// Debug: check if intersection points are on triangle edges
	//	int onEdgeA = 0, onEdgeB = 0, neither = 0;
	//	for (auto& c : intResult.curves) {
	//		for (auto& seg : c.segments) {
	//			const Triangle* triA = a.triangle(seg.triA);
	//			const Triangle* triB = b.triangle(seg.triB);
	//			if (!triA || !triB) continue;

	//			Point va0(triA->vertex(0)->x, triA->vertex(0)->y, triA->vertex(0)->z);
	//			Point va1(triA->vertex(1)->x, triA->vertex(1)->y, triA->vertex(1)->z);
	//			Point va2(triA->vertex(2)->x, triA->vertex(2)->y, triA->vertex(2)->z);

	//			int eA0 = mesh_intersect::findEdge(seg.start, va0, va1, va2, 1e-4);
	//			int eA1 = mesh_intersect::findEdge(seg.end, va0, va1, va2, 1e-4);

	//			if (eA0 >= 0) onEdgeA++; else neither++;
	//			if (eA1 >= 0) onEdgeA++; else neither++;
	//		}
	//	}
	//	std::cout << "points on triA edges: " << onEdgeA << ", not on edge: " << neither << std::endl;

	//	runTest("box_box", a, b);

	//	// Debug: check vertex merging
	//	auto result = csg::meshUnion(a, b);
	//	if (result.success) {
	//		// Check for duplicate vertices
	//		int dupCount = 0;
	//		for (std::size_t i = 0; i < result.mesh.vertexCount(); ++i) {
	//			const Vertex* vi = result.mesh.findByIndex(static_cast<int>(i));
	//			if (!vi) continue;
	//			for (std::size_t j = i + 1; j < result.mesh.vertexCount(); ++j) {
	//				const Vertex* vj = result.mesh.findByIndex(static_cast<int>(j));
	//				if (!vj) continue;
	//				double d = Vec3(vi->x - vj->x, vi->y - vj->y, vi->z - vj->z).cachedLength();
	//				if (d < 1e-4) dupCount++;
	//			}
	//		}
	//		std::cout << "duplicate vertex pairs: " << dupCount << std::endl;
	//	}
	//}

	//// Test 2: Box and sphere (sphere inside box)
	//{
	//	auto a = mesh_creation::api_make_box(2, 2, 2);
	//	auto b = mesh_creation::api_make_sphere(0.8, 24);
	//	runTest("box_sphere", a, b);
	//}

	//// Test 3: Two spheres (overlapping)
	//{
	//	auto a = mesh_creation::api_make_sphere(1.0, 24);
	//	auto b = mesh_creation::api_make_sphere(1.0, 24);
	//	b.translate(0.8, 0, 0);
	//	runTest("sphere_sphere", a, b);
	//}

	//// Test 4: Box and rotated prism
	//{
	//	auto a = mesh_creation::api_make_box(2, 2, 2);
	//	auto b = mesh_creation::api_make_prism(6, 3, 0.8, 0.8);
	//	b.rotateEuler(0, 0, 3.14159 / 4.0);  // 45 degree rotation
	//	runTest("box_prism_rotated", a, b);
	//}

	//// Test 5: Torus and sphere (complex intersection)
	//{
	//	auto a = mesh_creation::api_make_torus(1.5, 0.5, 24);
	//	auto b = mesh_creation::api_make_sphere(1.0, 24);
	//	runTest("torus_sphere", a, b);
	//}

	//// Test 6: Cone inside box
	//{
	//	auto a = mesh_creation::api_make_box(2, 2, 2);
	//	auto b = mesh_creation::api_make_cone(0.8, 2.5, 24);
	//	b.translate(0, -0.25, 0);
	//	runTest("box_cone", a, b);
	//}

	//// Test 7: Frustum and sphere (offset and rotated)
	//{
	//	auto a = mesh_creation::api_make_frustum(1.0, 0.5, 2.0, 24);
	//	auto b = mesh_creation::api_make_sphere(0.7, 24);
	//	b.translate(0.5, 0.5, 0);
	//	b.rotateEuler(0.3, 0.5, 0);
	//	runTest("frustum_sphere", a, b);
	//}

	//// Test 8: Two boxes at angle (edge intersection)
	//{
	//	auto a = mesh_creation::api_make_box(3, 1, 1);
	//	auto b = mesh_creation::api_make_box(3, 1, 1);
	//	b.rotateEuler(0, 0, 3.14159 / 3.0);  // 60 degrees
	//	runTest("box_box_angle", a, b);
	//}

	//// ===== Smoothing Demo =====
	//std::cout << "\n=== Smoothing Demo ===" << std::endl;
	//{
	//	auto smoothSphere = mesh_creation::api_make_sphere(1.0, 16);
	//	std::cout << "sphere before smooth: " << smoothSphere.vertexCount() << " verts" << std::endl;
	//	mesh_smooth::smoothLaplacian(smoothSphere, 5, 0.3);
	//	std::cout << "sphere after smooth: " << smoothSphere.vertexCount() << " verts" << std::endl;
	//	smoothSphere.exportObj(basePath + "smoothed_sphere.obj");
	//}

	//// ===== Decimation Demo =====
	//std::cout << "\n=== Decimation Demo ===" << std::endl;
	//{
	//	auto highRes = mesh_creation::api_make_sphere(1.0, 32);
	//	std::cout << "sphere before decimate: " << highRes.triangleCount() << " tris" << std::endl;
	//	mesh_decimate::decimateQEM(highRes, 0.5);
	//	std::cout << "sphere after decimate: " << highRes.triangleCount() << " tris" << std::endl;
	//	highRes.exportObj(basePath + "decimated_sphere.obj");
	//}

	//// ===== Curvature Demo =====
	//std::cout << "\n=== Curvature Demo ===" << std::endl;
	//{
	//	auto curvSphere = mesh_creation::api_make_sphere(1.0, 16);
	//	auto curv = mesh_curvature::computeCurvature(curvSphere);
	//	// For a unit sphere, Gaussian curvature should be 1.0
	//	double avgK = 0;
	//	for (double k : curv.gaussian) avgK += k;
	//	avgK /= curv.gaussian.size();
	//	std::cout << "sphere avg Gaussian curvature: " << avgK
	//		<< " (expected ~1.0)" << std::endl;
	//}

	//// ===== PLY/STL Export Demo =====
	//std::cout << "\n=== PLY/STL Export ===" << std::endl;
	//{
	//	auto exportMesh = mesh_creation::api_make_box(2, 1, 1);
	//	mesh_io::exportPly(exportMesh, basePath + "exported.ply");
	//	mesh_io::exportStl(exportMesh, basePath + "exported.stl");
	//	std::cout << "exported PLY and STL files" << std::endl;
	//}

	//std::cout << "\n=== All tests complete ===" << std::endl;
	//std::cout << "Exported OBJ files to: " << basePath << std::endl;

	return 0;
}
