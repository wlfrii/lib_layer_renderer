cmake_minimum_required(VERSION 3.10)

project(lib_layer_renderer
    VERSION 1.2.0
    DESCRIPTION "A library for rendering shapes, model, vertices, and graph based on OpenGL."
)
message("==== Build Library: ${PROJECT_NAME} ====")

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/app/)


# --------------------------------------------------------------------
#                      Find and Set Dependency
# --------------------------------------------------------------------
# # gl_util{} path
set(GL_UTIL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../mylearn/learn_OpenGL/gl_util")
# GLM, GLAD path
set(PATH_3RDPARTY "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty")
# Add GLFW
find_package(glfw3 REQUIRED)
if(${glfw3_FOUND})
    message(STATUS "GLFW3 found")
else()
    message(ERROR "Cannot find GLFW3")
endif()

# Add lib_math
set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../buildtarget/)
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package(lib_math REQUIRED)
if(${lib_math_FOUND})
    message(STATUS "lib_math found")
    message(STATUS "  include path: ${lib_math_INCLUDE_DIRS}")
else()
    message(ERROR "Cannot find lib_math")
endif()

# --------------------------------------------------------------------
#                   Create target and set properties
# --------------------------------------------------------------------
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(APPEND SRCS ${PATH_3RDPARTY}/glad/src/glad.c)
file(GLOB GL_UTIL_SRC ${GL_UTIL_PATH}/gl_util/*.cpp)
list(APPEND SRCS ${GL_UTIL_SRC})
add_library(${PROJECT_NAME} STATIC
    ${SRCS}
)

list(APPEND GL_UTIL_PATH ${GL_UTIL_PATH})
list(APPEND GL_UTIL_PATH ${PATH_3RDPARTY}/glm)
list(APPEND GL_UTIL_PATH ${PATH_3RDPARTY}/glad/include)
list(APPEND GL_UTIL_PATH ${PATH_3RDPARTY}/stb_image)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/export>
        $<BUILD_INTERFACE:${GL_UTIL_PATH}>
        $<BUILD_INTERFACE:${lib_math_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${gl_util_INCLUDE_DIRS}>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        glfw
        ${lib_math_LIBRARIES}
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

# Build the example
add_subdirectory(examples)

# Build the test
option(BUILD_TEST "select to build the test case" OFF)
if(BUILD_TEST)
    add_subdirectory(test)
endif()