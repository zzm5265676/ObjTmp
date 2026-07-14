#include "config.hxx"
#include "mesh/mesh.hxx"
#include "mesh/outer_shell.hxx"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv) {
	std::string inputPath = argc > 1 ? argv[1] : config::INPUT_DIR + "fdx.obj";
	bool countOnly = false;
	bool outerCountOnly = false;
	bool splitOnly = false;
	std::string outputPath = config::OUTPUT_DIR + "outer_shells.obj";
	int toleranceArg = 3;

	if (argc > 2) {
		std::string secondArg = argv[2];
		if (secondArg == "--count-only") {
			countOnly = true;
			toleranceArg = 3;
		}
		else if (secondArg == "--outer-count-only") {
			outerCountOnly = true;
			toleranceArg = 3;
		}
		else if (secondArg == "--split-components") {
			splitOnly = true;
			toleranceArg = 3;
		}
		else {
			outputPath = secondArg;
			toleranceArg = 3;
		}
	}

	try {
		double connectTolerance = argc > toleranceArg ? std::stod(argv[toleranceArg]) : 1.0e-7;
		if (connectTolerance < 0.0) {
			throw std::runtime_error("connectTolerance must be non-negative");
		}

		std::cout << "loading: " << inputPath << "\n";
		std::cout << "connect tolerance: " << connectTolerance << "\n";
		Mesh mesh(inputPath, connectTolerance);

		std::cout << "vertices: " << mesh.vertexCount() << "\n";
		std::cout << "directed edges: " << mesh.edgeCount() << "\n";
		std::cout << "triangles: " << mesh.triangleCount() << "\n";
		if (!mesh.parseErrors().empty()) {
			std::cout << "parse warnings: " << mesh.parseErrors().size() << "\n";
		}

		std::cout << "splitting components...\n";
		std::vector<Mesh> components = mesh.splitComponents();
		std::cout << "components: " << components.size() << "\n";

		if (countOnly) {
			return 0;
		}
		if (splitOnly) {
			for (std::size_t i = 0; i < components.size(); ++i) {
				std::string componentPath = config::OUTPUT_DIR + "component_" + std::to_string(i) + ".obj";
				if (!components[i].exportObj(componentPath)) {
					std::cerr << "failed to export: " << componentPath << "\n";
					return 1;
				}

				std::cout << "component " << i
					<< ": vertices=" << components[i].vertexCount()
					<< ", directed_edges=" << components[i].edgeCount()
					<< ", triangles=" << components[i].triangleCount()
					<< ", output=" << componentPath << "\n";
			}
			return 0;
		}

		std::cout << "filtering enclosed shells...\n";
		std::vector<std::size_t> keep = mesh_outer::findOuterShellIndices(components, connectTolerance);
		std::cout << "outer shells: " << keep.size() << "\n";
		std::cout << "removed enclosed shells: " << (components.size() - keep.size()) << "\n";

		if (outerCountOnly) {
			return 0;
		}

		Mesh outerMesh = mesh_outer::mergeShells(components, keep, connectTolerance);
		if (!outerMesh.exportObj(outputPath)) {
			std::cerr << "failed to export: " << outputPath << "\n";
			return 1;
		}

		std::cout << "output: " << outputPath << "\n";
		std::cout << "output vertices: " << outerMesh.vertexCount() << "\n";
		std::cout << "output triangles: " << outerMesh.triangleCount() << "\n";
	}
	catch (const std::exception& e) {
		std::cerr << "error: " << e.what() << "\n";
		return 1;
	}

	return 0;
}
