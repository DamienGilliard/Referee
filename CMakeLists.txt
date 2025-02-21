cmake_minimum_required(VERSION 3.6)

project(Referee)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

execute_process(COMMAND git submodule update --init --recursive
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

file(GLOB_RECURSE SOURCES_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Referee.hh
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Referee/*.cc)

add_library(${SHARED_LIB_NAME} SHARED ${SOURCES_LIB})
target_include_directories(${SHARED_LIB_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Referee)   

###################################
# 3rd party libraries paths
###################################
# pcl
find_package(PCL 1.14 REQUIRED COMPONENTS common io visualization filters segmentation features surface kdtree octree search sample_consensus registration keypoints tracking recognition)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${SHARED_LIB_NAME} ${PCL_LIBRARIES})

############
# Executable
############

set(EXECUTABLE_NAME_1 referee_exec)
add_executable(${EXECUTABLE_NAME_1}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc)       

target_link_libraries(${EXECUTABLE_NAME_1} ${SHARED_LIB_NAME} ${PCL_LIBRARIES})
