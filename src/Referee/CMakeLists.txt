################################
# Dynamic library of Referee
################################

file(GLOB_RECURSE SOURCES_LIB
    "${CMAKE_CURRENT_SOURCE_DIR}/../Referee.hh"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")

add_library(${SHARED_LIB_NAME} ${SOURCES_LIB})