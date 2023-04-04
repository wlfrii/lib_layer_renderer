cmake_minimum_required(VERSION 3.16)

project(lib_layer_renderer
    VERSION 1.2.0
    DESCRIPTION "A library for rendering shapes, model, vertices, and graph based on OpenGL."
    LANGUAGES CXX
)
message("==== Build Library: ${PROJECT_NAME} ====")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")


# option(ENABLE_INSTALL "Flag for installing" ON)

    # Set the install path and prefix
    option(SPECIFY_INSTALL_PATH "Specify the install path" ON)
    if(SPECIFY_INSTALL_PATH)
    set(MODULE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
    message(STATUS "Module root: ${MODULE_ROOT}")
    set(REPO_ROOT ${MODULE_ROOT}/../)
    message(STATUS "Repo root: ${REPO_ROOT}")
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX
            ${REPO_ROOT}/buildtarget/ CACHE PATH "repo root" FORCE
            )
    endif()
    endif()
    message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

    include(GNUInstallDirs)

# --------------------------------------------------------------------
#                      Find and Set Dependency
# --------------------------------------------------------------------

# gl_util{} path
set(GL_UTIL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../mylearn/learn_OpenGL/gl_util")
file(GLOB GL_UTIL_SRC ${GL_UTIL_PATH}/gl_util/*.cpp)

# GLM, GLAD path
set(PATH_3RDPARTY "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty")

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

# Add GLFW
find_package(glfw3 REQUIRED)
if(${glfw3_FOUND})
    message(STATUS "GLFW3 found")
else()
    message(ERROR "Cannot find GLFW3")
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
        $<BUILD_INTERFACE:${lib_math_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${GL_UTIL_PATH}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/export>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}/glm>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}/glad/include>
        $<BUILD_INTERFACE:${PATH_3RDPARTY}/stb_image>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        glfw
        ${lib_math_LIBRARIES}
)

set(${PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/export
                                 ${PATH_3RDPARTY}/glm
                                 ${PATH_3RDPARTY}/glad/include
                                 ${Boost_INCLUDE_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

# --------------------------------------------------------------------
#                           Installation
# --------------------------------------------------------------------
    set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
    message(STATUS "INSTALL_CONFIGDIR: ${INSTALL_CONFIGDIR}")
    install(TARGETS ${PROJECT_NAME} 
        EXPORT ${PROJECT_NAME}-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    )

    file(GLOB MODEL_STL ${CMAKE_CURRENT_SOURCE_DIR}/models/*.STL)
    file(COPY ${MODEL_STL}
        DESTINATION ${CMAKE_BINARY_DIR}/models/
    )
    file(GLOB SHADERS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.*s)
    file(COPY ${SHADERS}
        DESTINATION ${CMAKE_BINARY_DIR}/shaders/
    )
    file(GLOB EXPORT_H export/*.h)
    install(FILES ${EXPORT_H}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    )
    install(DIRECTORY export/${PROJECT_NAME}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    )
    file(GLOB EXPORT_H ${GL_UTIL_PATH}/*.h)
    install(FILES ${EXPORT_H}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    )
    install(DIRECTORY ${GL_UTIL_PATH}/gl_util
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    )
    file(GLOB EXPORT_H ${PATH_3RDPARTY}/glad/include/glad/*.h)
    install(FILES ${EXPORT_H}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/glad/
    )
    file(GLOB EXPORT_H ${PATH_3RDPARTY}/glad/include/KHR/*.h)
    install(FILES ${EXPORT_H}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/KHR/
    )
    file(GLOB EXPORT_H /usr/local/include/GLFW/*.h)
    install(FILES ${EXPORT_H}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/GLFW/
    )
    install(DIRECTORY ${PATH_3RDPARTY}/glm/glm
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    )

    install(EXPORT ${PROJECT_NAME}-targets
        FILE ${PROJECT_NAME}Targets.cmake
        DESTINATION ${INSTALL_CONFIGDIR}
    )

# --------------------------------------------------------------------
#                            Packaging
# --------------------------------------------------------------------
    include(CMakePackageConfigHelpers)
    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
    )
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )
    # Install the package infomation
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake  
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake 
        DESTINATION ${INSTALL_CONFIGDIR}
    )


# Build the test
# option(BUILD_TEST "select to build the test case" ON)
# if(BUILD_TEST)
#     add_subdirectory(test)
# endif()


# add_subdirectory(NonRigidReconstruction)
