cmake_minimum_required(VERSION 3.5.1)

project(MarchingCube)


set(CMAKE_CXX_STANDARD 14)

include_directories(include/header/)


FILE(GLOB_RECURSE HeadFILE include/header/*.h)
FILE(GLOB_RECURSE SRCFILE include/src/*.cpp)


add_executable(MarchingCube MarchingCubeSample.cpp ${HeadFILE}  ${SRCFILE})