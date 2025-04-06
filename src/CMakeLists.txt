cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0079)
    cmake_policy(SET CMP0079 NEW)
endif()
project(Referee)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE GIT_SUBMOD_RESULT
    )
if(NOT GIT_SUBMOD_RESULT EQUAL "0")
    message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
endif()

################################
# Dynamic library of Referee
################################
set(SHARED_LIB_NAME referee)
add_subdirectory(Referee)

###################################
# 3rd party libraries paths
###################################
# GlobalMatch
set(GLOBAL_MATCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/GlobalMatch/code/global_match")

file(GLOB_RECURSE GLOBAL_MATCH_SOURCES
    ${GLOBAL_MATCH_DIR}/*.cpp
    ${GLOBAL_MATCH_DIR}/*.h)

add_library(global_match STATIC ${GLOBAL_MATCH_SOURCES})
target_include_directories(global_match
    PUBLIC ${GLOBAL_MATCH_DIR})

target_link_libraries(${SHARED_LIB_NAME} global_match)

# CSF
set(CSF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/GlobalMatch/code/3rd_party/csf")
include_directories(${CSF_DIR})
file(GLOB_RECURSE CSF_SOURCES
    ${CSF_DIR}/*.cpp
    ${CSF_DIR}/*.h)

add_library(csf STATIC ${CSF_SOURCES})
target_include_directories(csf
    PUBLIC ${CSF_DIR})

target_link_libraries(global_match csf)

# pcl
find_package(PCL 1.14 REQUIRED COMPONENTS common io visualization filters segmentation features surface kdtree octree search sample_consensus registration keypoints tracking recognition)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${SHARED_LIB_NAME} ${PCL_LIBRARIES})

# vtk
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

target_include_directories(${SHARED_LIB_NAME} PUBLIC ${VTK_INCLUDE_DIRS})
target_include_directories(global_match PUBLIC ${VTK_INCLUDE_DIRS})  # Add this line
target_link_libraries(${SHARED_LIB_NAME} ${VTK_LIBRARIES})

############
# Executable
############

set(EXECUTABLE_NAME_1 referee_exec)
add_executable(${EXECUTABLE_NAME_1}
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)       

target_link_libraries(${EXECUTABLE_NAME_1} ${SHARED_LIB_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES})