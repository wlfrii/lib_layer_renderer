set(TEST_NAME "test_${PROJECT_NAME}_run")
message("==== Build Test: ${TEST_NAME} ====")

add_executable(${TEST_NAME}
    test_run.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
)

target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)


set(TEST_NAME "test_${PROJECT_NAME}_run_texture3d")
message("==== Build Test: ${TEST_NAME} ====")

add_executable(${TEST_NAME}
    test_run_texture3d.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
)

target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

set(TEST_NAME "test_${PROJECT_NAME}_run_gripper")
message("==== Build Test: ${TEST_NAME} ====")

add_executable(${TEST_NAME}
    test_run_gripper.cpp
)

target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
)