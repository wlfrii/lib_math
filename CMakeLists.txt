cmake_minimum_required(VERSION 3.10)

project(lib_math 
    VERSION 1.2.0
    DESCRIPTION "A extended library for matrix mathematics"
    LANGUAGES CXX)
message(STATUS "Project: ${PROJECT_NAME}")

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# Set the install path and prefix
option(SPECIFY_INSTALL_PATH "Specify the install path" OFF)
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

option(LIB_MATH_KINE_DOUBLE "Use double precision in kinematics" OFF)
if(LIB_MATH_KINE_DOUBLE)
    add_definitions(-DLIB_MATH_KINE_DOUBLE)
    message(STATUS "Use double precision in kinematics")
endif()

# make cache variables for install destinations
include(GNUInstallDirs)

# --------------------------------------------------------------------
#                   Create target and set properties
# --------------------------------------------------------------------
file(GLOB_RECURSE SRCS src/*.cpp)
add_library(${PROJECT_NAME}
    ${SRCS}
)

# Specifies include directories to use when compiling a given target.  
# The target name must have been created by a command such as  
# add_excutable() or add_library().
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
    message(STATUS "Found Eigen: ${EIGEN3_INCLUDE_DIRS}")
else(${Eigen3_FOUND})
    message(ERROR "Cannot find Eigen")
endif(${Eigen3_FOUND})

# We need to tell CMake that we want to use different include 
# directories depending on if we are building the library or using it 
# from an installed location. If we don't do this, when CMake creates 
# the export information it will export a path that is specific to the 
# current build directory and will not be valid for other projects.
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/export>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# --------------------------------------------------------------------
#                           Installation
# --------------------------------------------------------------------
# The install(TARGETS) and install(EXPORT) commands work together to 
# install both targets and a CMake file designed to make it easy to 
# import the targets into another CMake project.

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
message(STATUS "INSTALL_CONFIGDIR: ${INSTALL_CONFIGDIR}")

# 1. Install library
# Set installation path 
install(TARGETS ${PROJECT_NAME} 
    EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
# 2. Install header files
file(GLOB EXPORT_H export/*.h)
install(FILES ${EXPORT_H}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
file(GLOB EXPORT_H export/kine/*.h)
install(FILES ${EXPORT_H}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/kine
)
file(GLOB EXPORT_H export/matrix/*.h)
install(FILES ${EXPORT_H}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/matrix
)
file(GLOB EXPORT_H export/util/*.h)
install(FILES ${EXPORT_H}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/util
)
file(GLOB EXPORT_H export/curve/*.h)
install(FILES ${EXPORT_H}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/curve
)
file(GLOB EXPORT_H export/timer/*.h)
install(FILES ${EXPORT_H}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/timer
)
# 3. Explicitly install the ${PROJECT_NAME}Targets export details.
install(EXPORT ${PROJECT_NAME}-targets
    # NAMESPACE mylib::
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

# --------------------------------------------------------------------
#                         Creating Packages
# --------------------------------------------------------------------
# Create a library that can be found by find_package()
# i.e., creat a ConfigVersion.cmake file
# Install the Config, ConfigVersion and Custom find moduls
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
