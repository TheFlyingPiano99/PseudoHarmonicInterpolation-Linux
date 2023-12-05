#include "cmake_minimum_required(VERSION 3.20)
project(PseudoHarmonicSurface LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=C++20 -lstdc++")

add_executable(PseudoHarmonicSurface
	main.cpp
	modifiedgordonwixomsurface.cpp
    	vector.cc
	matrix3x3.cc
)
target_link_libraries(
    PseudoHarmonicSurface 
    ${CMAKE_CURRENT_SOURCE_DIR}/triangle/triangle.o
)

