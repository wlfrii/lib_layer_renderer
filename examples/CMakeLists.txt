set(EXAMPLE_NAME "${PROJECT_NAME}_examples")
message("==== Build Example: ${EXAMPLE_NAME} ====")
add_executable(${EXAMPLE_NAME}
    lib_layer_renderer_examples.cpp
)
target_link_libraries(${EXAMPLE_NAME}
    ${PROJECT_NAME}
)


set(EXAMPLE_NAME "${PROJECT_NAME}_examples_multiview")
message("==== Build Example: ${EXAMPLE_NAME} ====")
add_executable(${EXAMPLE_NAME}
    lib_layer_renderer_examples_multiview.cpp
)
target_link_libraries(${EXAMPLE_NAME}
    ${PROJECT_NAME}
)