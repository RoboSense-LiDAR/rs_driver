cmake_minimum_required(VERSION 3.5)
project(lidar_driver)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

#========================
# Subdirectory
#========================
include(CheckSymbolExists)
#========================
# Project details / setup
#========================
set(PROJECT_NAME lidar_driver)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
#========================
# debug
#========================

option(RS_POINT_COMPLEX "Select point type of complex" OFF)
message("RS_POINT_COMPLEX is ${RS_POINT_COMPLEX}")

if(RS_POINT_COMPLEX)
message("add RS_POINT_COMPLEX macro!!")
add_definitions(-DRS_POINT_COMPLEX)
endif(RS_POINT_COMPLEX)

add_definitions(-DDEBUG_LEVEL=0)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

cmake_policy(SET CMP0048 NEW)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_definitions(-std=c++14)
add_compile_options(-W)

#========================
# Dependencies Setup
#========================
set(EXTERNAL_LIBS "")
find_package(roscpp 1.12 QUIET)
if(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Found, Ros Support is turned On!")
  message(=============================================================)
  add_definitions(-DROS_FOUND)
  include_directories(${roscpp_INCLUDE_DIRS})
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${roscpp_LIBRARIES})
else(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif(roscpp_FOUND)


#========================
# Build Setup
#========================
include_directories(${PROJECT_SOURCE_DIR})

add_executable(demo
            demo/demo.cpp
            utility/thread_pool.cpp
            )

target_link_libraries(demo
                    ${roscpp_LIBRARIES}
                    pthread
                    pcap
                       )


