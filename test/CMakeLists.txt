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
