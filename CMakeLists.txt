cmake_minimum_required(VERSION 3.16)
project(ObjTmp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include_directories(${PROJECT_SOURCE_DIR}/include)

set(MESH_SRC
    src/core/point.cpp
    src/core/vec.cpp
    src/core/vertex.cpp
    src/mesh/mesh.cpp
)

set(MESH_HDR
    include/config.hxx
    include/core/edge.hxx
    include/core/point.hxx
    include/core/triangle.hxx
    include/core/vec3.hxx
    include/core/vertex.hxx
    include/mesh/mesh.hxx
)

add_executable(main main.cpp ${MESH_SRC} ${MESH_HDR})
target_compile_definitions(main PRIVATE OBJTMP_PROJECT_ROOT="${PROJECT_SOURCE_DIR}")

# Set working directory for VS debugger
set_target_properties(main PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)

# Create output directory
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/data/output)

# Source groups for IDE organization
source_group("src\\core" FILES
    src/core/point.cpp
    src/core/vec.cpp
    src/core/vertex.cpp
)
source_group("src\\mesh" FILES
    src/mesh/mesh.cpp
)
source_group("src\\operations" FILES
    src/operations/creation.cpp
    src/operations/repair.cpp
)
