cmake_minimum_required(VERSION 3.10)

project(lib_render_layer
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
# PCL requires Qt5
set(CMAKE_PREFIX_PATH /usr/local/Cellar/qt@5/5.15.3/)
find_package(PCL REQUIRED)
if(PCL_FOUND)
    message(STATUS "Found PCL")
    message(STATUS "  Include: ${PCL_INCLUDE_DIRS}")
    message(STATUS "  Library path: ${PCL_LIBRARY_DIRS}")
    message(STATUS "  Libraries: ${PCL_LIBRARIES}")
    # message(STATUS "  Definitions: ${PCL_DEFINITIONS}")
endif()


file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} STATIC
    ${SRCS}
    ${PATH_3RDPARTY}/glad/src/glad.c
    ${GL_UTIL_SRC}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}/glm>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}/glad/include>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}/stb_image>
        $<BUILD_INTERFACE:${GL_UTIL_PATH}>
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${lib_math_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${PROJECT_NAME}
    glfw
    ${lib_math_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
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


# Build the test
option(BUILD_TEST "select to build the test case" ON)
if(BUILD_TEST)
    add_subdirectory(test)
endif()
