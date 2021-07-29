cmake_minimum_required(VERSION 3.10)

project(lib_math 
    VERSION 1.1.0 
    DESCRIPTION "A extended library for matrix mathematics"
    LANGUAGES CXX)
set(BINARY ${CMAKE_PROJECT_NAME})
message(STATUS "Project: ${BINARY}")

set(CMAKE_CXX_STANDARD 14)

# Set the install path and prefix
set(MODULE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "Module root: ${MODULE_ROOT}")
set(REPO_ROOT ${MODULE_ROOT}/../../)
message(STATUS "Repo root: ${REPO_ROOT}")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX 
        ${REPO_ROOT}/buildtarget/ CACHE PATH "repo root" FORCE
    )
endif()
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

# make cache variables for install destinations
include(GNUInstallDirs)

# --------------------------------------------------------------------
#                   Create target and set properties
# --------------------------------------------------------------------
file(GLOB_RECURSE SRCS src/*.cpp)
add_library(${BINARY} STATIC 
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
target_include_directories(${BINARY}
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

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${BINARY})
message(STATUS "INSTALL_CONFIGDIR: ${INSTALL_CONFIGDIR}")

# 1. Install library
# Set installation path 
install(TARGETS ${BINARY} 
    EXPORT ${BINARY}-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${BINARY}
)
# 2. Install header files
file(GLOB EXPORT_H export/*.h)
install(FILES ${EXPORT_H}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${BINARY}
)
file(GLOB EXPORT_H export/kine/*.h)
install(FILES ${EXPORT_H}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${BINARY}/kine
)
# 3. Explicitly install the ${BINARY}Targets export details.
install(EXPORT ${BINARY}-targets
    # NAMESPACE mylib::
    FILE ${BINARY}Targets.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

# --------------------------------------------------------------------
#                         Creating Packages
# --------------------------------------------------------------------
# Create a library that can be found be find_package()
# i.e., creat a ConfigVersion.cmake file
# Install the Config, ConfigVersion and Custom find moduls
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${BINARY}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${BINARY}Config.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${BINARY}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
# Install the package infomation
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${BINARY}Config.cmake  
        ${CMAKE_CURRENT_BINARY_DIR}/${BINARY}ConfigVersion.cmake 
    DESTINATION ${INSTALL_CONFIGDIR}
)
