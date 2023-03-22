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

# Add Ceres
find_package(Ceres REQUIRED)
if(${Ceres_FOUND})
    message(STATUS "Ceres found: ${CERES_INCLUDE_DIRS}")
else()
    message(ERROR "Cannot find Ceres")
endif()


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
)

file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(${PROJECT_NAME}
    main.cpp
    ${SRCS}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${lib_math_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${CERES_INCLUDE_DIRS}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../export
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${lib_math_LIBRARIES}
    lib_layer_renderer
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
)
