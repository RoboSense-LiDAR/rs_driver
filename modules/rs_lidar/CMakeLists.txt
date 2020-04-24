cmake_minimum_required(VERSION 3.5)
include(CheckSymbolExists)
#========================
# debug
#========================
add_definitions(-DDEBUG_LEVEL=0)
#========================
# Project details / setup
#========================
set(PROJECT_NAME rs_lidar)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

cmake_policy(SET CMP0048 NEW)
project(${PROJECT_NAME})

set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_definitions(-std=c++11)
add_compile_options(-W)

#========================
# Dependencies Setup
#========================
set(EXTERNAL_LIBS "")

find_package(PCL QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
FIND_PACKAGE( OpenMP REQUIRED) 
if(OPENMP_FOUND) 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") 
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}") 
set(CMAKE_SHARE_LINKER_FLAGS "${CMAKE_SHARE_LINKER_FLAGS} ${OpenMP_SHARE_LINKER_FLAGS}") 
endif()
#========================
# Build Setup
#========================

add_library(rs_lidar SHARED
          src/input.cpp
          src/lidar_driver.cpp
          )
add_dependencies(rs_lidar rs_yaml)

target_link_libraries(rs_lidar
                      rs_yaml
                    ${PCL_LIBRARIES}
                      pcap
                     )

target_include_directories(rs_lidar PUBLIC 
                      ${PROJECT_SOURCE_DIR}/include
                            ${PCL_INCLUDE_DIRS})
