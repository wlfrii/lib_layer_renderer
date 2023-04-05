set(PROJ_NAME "${PROJECT_NAME}_example")
message("==== Build Test: ${PROJ_NAME} ====")
add_executable(${PROJ_NAME}
    lib_layer_renderer_example.cpp
)
target_include_directories(${PROJ_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(${PROJ_NAME}
    PUBLIC    
        ${PROJECT_NAME}
)


set(PROJ_NAME "${PROJECT_NAME}_multiview_example")
message("==== Build Test: ${PROJ_NAME} ====")
add_executable(${PROJ_NAME}
    lib_layer_renderer_multiview_example.cpp
)
target_include_directories(${PROJ_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(${PROJ_NAME}
    ${PROJECT_NAME}
)
