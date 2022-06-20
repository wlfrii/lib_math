cmake_minimum_required(VERSION 3.10)
project(lib_math_test)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../buildtarget/)
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package(lib_math REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_SRC)
add_executable(${PROJECT_NAME}
    # main.cpp
    ${SRC_SRC}
)

# Use catch2 as the test library
set(CATCH_INCLUDE_DIR /usr/local/Cellar/catch2/2.13.9/include/)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CATCH_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${lib_math_LIBRARIES}
)
