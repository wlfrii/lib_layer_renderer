cmake_minimum_required(VERSION 3.10)

project(nonrigid_reconstruction
    VERSION 1.0.0
    DESCRIPTION "A project for reconstruction of nonrigid scene")
message("==== Build Project: ${PROJECT_NAME} ====")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/app/)

# Setting
set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


# Add Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
    message(STATUS "Eigen found: ${EIGEN3_INCLUDE_DIRS}")
else(${Eigen3_FOUND})
    message(ERROR "Cannot find Eigen")
endif(${Eigen3_FOUND})


# Add lib_math
set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../buildtarget/)
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package(lib_math REQUIRED)
if(${lib_math_FOUND})
    message(STATUS "lib_math found: ${lib_math_INCLUDE_DIRS}")
else(${lib_math_FOUND})
    message(ERROR "Cannot find lib_math")
endif(${lib_math_FOUND})


# Add OpenCV
find_package(OpenCV REQUIRED)
if(${OpenCV_FOUND})
    message(STATUS "OpenCV library status:")
    message(STATUS "  version: ${OpenCV_VERSION}")
    message(STATUS "  libraries: ${OpenCV_LIBS}")
    message(STATUS "  include path: ${OpenCV_INCLUDE_DIRS}")
else(${OpenCV_FOUND})
    message(WARNING "Cannot found OpenCV.")
endif(${OpenCV_FOUND})


# Add GLFW
find_package(glfw3 REQUIRED)
if(${glfw3_FOUND})
    message(STATUS "GLFW3 found")
else()
    message(ERROR "Cannot find GLFW3")
endif()


# gl_util{} path
set(GL_UTIL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../mylearn/learn_OpenGL/gl_util")
file(GLOB GL_UTIL_SRC ${GL_UTIL_PATH}/gl_util/*.cpp)
# include_directories(${GL_UTIL_PATH})


file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(${PROJECT_NAME}
    main.cpp
    ${SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib_layer_renderer/3rdparty/glad/src/glad.c
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${lib_math_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${GL_UTIL_PATH}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib_layer_renderer/export
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib_layer_renderer/3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib_layer_renderer/3rdparty/glad/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib_layer_renderer/3rdparty/glm
)
link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib_layer_renderer/build)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${lib_math_LIBRARIES}
    lib_layer_renderer
    glfw
)
