# Iterate over each test file and create the corresponding executable, link libraries, and add tests
foreach(TEST_FILE ${UNIT_TEST_FILES})
    message(STATUS "Processing test file: ${TEST_FILE}")
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    message(STATUS "Test name: ${TEST_NAME}")
    add_executable(${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${TEST_NAME} ${SHARED_LIB_NAME})
    add_test(NAME ${TEST_NAME}
             COMMAND ${CMAKE_BINARY_DIR}/tests/${TEST_NAME})
    add_dependencies(${TEST_NAME} ${SHARED_LIB_NAME})
endforeach()


# # test on plane intersection
# add_executable(${UNIT_TEST_EXECUTABLE_NAME_PLANE_INTERSECTION} ${CMAKE_CURRENT_SOURCE_DIR}/UnitTests/${UNIT_TEST_EXECUTABLE_NAME_PLANE_INTERSECTION}.cc)
# target_link_libraries(${UNIT_TEST_EXECUTABLE_NAME_PLANE_INTERSECTION} ${SHARED_LIB_NAME})
# add_test(NAME ${UNIT_TEST_EXECUTABLE_NAME_PLANE_INTERSECTION} 
#         COMMAND ${CMAKE_BINARY_DIR}/tests/${UNIT_TEST_EXECUTABLE_NAME_PLANE_INTERSECTION})
# add_dependencies(${UNIT_TEST_EXECUTABLE_NAME_PLANE_INTERSECTION} ${SHARED_LIB_NAME})