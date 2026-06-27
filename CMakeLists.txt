cmake_minimum_required(VERSION 3.16)
project(MyProject CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB_RECURSE ALL_SRC CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)
file(GLOB_RECURSE ALL_HDR CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/include/*.h
    ${PROJECT_SOURCE_DIR}/include/*.hpp
    ${PROJECT_SOURCE_DIR}/include/*.hxx

)

add_executable(main main.cpp ${ALL_SRC} ${ALL_HDR})

# 只在有文件时才创建 source_group
if(ALL_SRC)
    source_group("src" FILES ${ALL_SRC})
endif()
if(ALL_HDR)
    source_group("include" FILES ${ALL_HDR})
endif()