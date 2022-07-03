cmake_minimum_required(VERSION 3.10)

project(render_layer
    VERSION 1.1.0
    DESCRIPTION "A project for rendering surgical tool and its background"
)
message("==== Build Library: ${PROJECT_NAME} ====")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --------------------------------------------------------------------
#                   Create target and set properties
# --------------------------------------------------------------------
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} STATIC
    ${SRCS}
    ${PATH_3RDPARTY}/glad/src/glad.c
    ${GL_UTIL_SRC}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PATH_3RDPARTY}>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}/glm>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}/glad/include>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}/stb_image>
        $<BUILD_INTERFACE:${GL_UTIL_PATH}>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${PROJECT_NAME}
    glfw
    ${OpenCV_LIBS}
)

file(GLOB MODEL_STL ${CMAKE_CURRENT_SOURCE_DIR}/models/*.STL)
file(COPY
    ${MODEL_STL}
    DESTINATION ${CMAKE_BINARY_DIR}/models/
)
file(GLOB SHADERS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.*s)
file(COPY
    ${SHADERS}
    DESTINATION ${CMAKE_BINARY_DIR}/shaders/
)

# Build the library firstc
# add_subdirectory(src)
# Build the examples
# add_subdirectory(examples)


# Build the test
option(BUILD_TEST "select to build the test case" ON)
if(BUILD_TEST)
    add_subdirectory(test)
endif()
