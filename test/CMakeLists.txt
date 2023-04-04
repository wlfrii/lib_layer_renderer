set(TEST_NAME "${PROJECT_NAME}_test")
message("==== Build Test: ${TEST_NAME} ====")
add_executable(${TEST_NAME}
    lib_layer_renderer_test.cpp
)
target_include_directories(${TEST_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../export>
)
target_link_libraries(${TEST_NAME}
    PUBLIC    
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


# # Add OpenCV
# find_package(OpenCV REQUIRED)
# if(${OpenCV_FOUND})
#     message(STATUS "OpenCV library status:")
#     message(STATUS "  version: ${OpenCV_VERSION}")
#     message(STATUS "  libraries: ${OpenCV_LIBS}")
#     message(STATUS "  include path: ${OpenCV_INCLUDE_DIRS}")
# else(${OpenCV_FOUND})
#     message(WARNING "Cannot found OpenCV.")
# endif(${OpenCV_FOUND})

# set(TEST_NAME "${PROJECT_NAME}_sr")
# message("==== Build Test: ${TEST_NAME} ====")
# add_executable(${TEST_NAME}
#     lib_layer_renderer_sr_test.cpp
# )
# target_include_directories(${TEST_NAME}
#     PUBLIC
#         $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
#         $<BUILD_INTERFACE:${PATH_3RDPARTY}/glm>
# )
# target_link_libraries(${TEST_NAME}
#     lib_layer_renderer
#     ${OpenCV_LIBS}
# )

# set(TEST_NAME "${PROJECT_NAME}_line_pair")
# message("==== Build Test: ${TEST_NAME} ====")
# add_executable(${TEST_NAME}
#     lib_layer_renderer_line_pair.cpp
# )
# target_include_directories(${TEST_NAME}
#     PUBLIC
#         $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
#         $<BUILD_INTERFACE:${PATH_3RDPARTY}/glm>
# )
# target_link_libraries(${TEST_NAME}
#     lib_layer_renderer
#     ${OpenCV_LIBS}
# )