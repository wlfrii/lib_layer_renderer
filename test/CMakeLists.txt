set(TEST_NAME "${PROJECT_NAME}_run")
message("==== Build Test: ${TEST_NAME} ====")

add_executable(${TEST_NAME}
    test_run.cpp
)
target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
)
