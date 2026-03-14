cmake_minimum_required(VERSION 3.12)
include(ExternalProject)

if(POLICY CMP0079)
    cmake_policy(SET CMP0079 NEW)
endif()
project(Referee)

set(CMAKE_CXX_STANDARD 20)
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

set(CURL_LIBRARY /lib/x86_64-linux-gnu/libcurl.so CACHE FILEPATH "" FORCE)
set(CURL_INCLUDE_DIR /usr/include CACHE PATH "" FORCE)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

################################
# Dynamic library of Referee
################################
set(SHARED_LIB_NAME referee)
add_subdirectory(Referee)

###################################
# 3rd party libraries paths
###################################

# # eigen
# include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/eigen)

# OpenMPI
include_directories(/usr/lib/x86_64-linux-gnu/openmpi/include)

# GlobalMatch
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/GlobalMatch)

# Graaf
include_directories(${CMAKE_SOURCE_DIR}/3rd_party/graaf/include)
target_include_directories(${SHARED_LIB_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/3rd_party/graaf/include)

# pcl
find_package(PCL 1.14 REQUIRED COMPONENTS common io visualization filters segmentation features surface kdtree octree search sample_consensus registration keypoints tracking recognition)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# GDAL
## ↓↓↓ Avoid running GDAL tests in my project ↓↓↓
set(SAVED_BUILD_TESTING ${BUILD_TESTING})
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
## ^^^ Avoid running GDAL tests in my project ^^^
set(GDAL_USE_TIFF_INTERNAL ON CACHE BOOL "" FORCE)
set(GDAL_USE_GEOTIFF_INTERNAL ON CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/gdal)
set(BUILD_TESTING ${SAVED_BUILD_TESTING})
# # to make sure GDAL is built before referee is linked (had some issues with this)
# add_dependencies(${SHARED_LIB_NAME} GDAL)
target_link_libraries(${SHARED_LIB_NAME} ${PCL_LIBRARIES} GlobalMatch GDAL)

# PDAL 
find_package(PDAL REQUIRED)
target_link_libraries(${SHARED_LIB_NAME} ${PDAL_LIBRARIES})

#json
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/json/single_include)

############
# Executable
############

set(EXECUTABLE_NAME_1 referee_exec)
add_executable(${EXECUTABLE_NAME_1}
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)       

target_link_libraries(${EXECUTABLE_NAME_1} ${SHARED_LIB_NAME})

#############
# Unit tests
#############

# Get all test files in the ./UnitTests/ directory
file(GLOB UNIT_TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/tests/UnitTests/*.cc")

enable_testing()
include(CTest)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)

set(TEST_EXECUTABLES)
foreach(TEST_FILE ${UNIT_TEST_FILES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    list(APPEND TEST_EXECUTABLES ${TEST_NAME})
endforeach()

add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS ${TEST_EXECUTABLES})

add_dependencies(${EXECUTABLE_NAME_1} run_tests) 