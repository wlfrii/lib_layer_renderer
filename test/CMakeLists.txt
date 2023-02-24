set(TEST_NAME "test_${PROJECT_NAME}")
message("==== Build Test: ${TEST_NAME} ====")
add_executable(${TEST_NAME}
    test_lib_layer_renderer.cpp
)
target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
)


set(TEST_NAME "test_${PROJECT_NAME}_multiview")
message("==== Build Test: ${TEST_NAME} ====")
add_executable(${TEST_NAME}
    test_lib_layer_renderer_multiview.cpp
)
target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
)