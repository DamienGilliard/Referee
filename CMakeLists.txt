cmake_minimum_required(VERSION 3.0)

project(Referee)

set(CMAKE_CXX_STANDARD 17)

include(cmake/functions.cmake)

add_submodule(pcl)

set(SHARED_LIB_NAME referee)

file(GLOB_RECURSE SRC_FILES src/*.cc
                            src/*.hh)

add_library(${SHARED_LIB_NAME} SHARED ${SRC_FILES})

target_include_directories(${SHARED_LIB_NAME} PUBLIC src)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(Referee src/referee.cc)
