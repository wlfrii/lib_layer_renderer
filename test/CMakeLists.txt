include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../export/)

set(TEST_NAME "${PROJECT_NAME}_test")
message("==== Build Test: ${TEST_NAME} ====")
add_executable(${TEST_NAME}
    lib_layer_renderer_test.cpp
)
target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
)


set(TEST_NAME "${PROJECT_NAME}_test_multiview")
message("==== Build Test: ${TEST_NAME} ====")
add_executable(${TEST_NAME}
    lib_layer_renderer_test_multiview.cpp
)
target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
)