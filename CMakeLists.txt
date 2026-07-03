cmake_minimum_required(VERSION 3.16)
project(ObjTmp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB_RECURSE ALL_SRC CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)
file(GLOB_RECURSE ALL_HDR CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/include/*.hxx
    ${PROJECT_SOURCE_DIR}/include/*.h
    ${PROJECT_SOURCE_DIR}/include/*.hpp
)

add_executable(main main.cpp ${ALL_SRC} ${ALL_HDR})

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
