cmake_minimum_required(VERSION 3.5)
project(lidar_driver)


#========================
# Subdirectory
#========================
add_subdirectory(modules/rs_common)
add_subdirectory(modules/rs_lidar)
add_subdirectory(modules/rs_manager)
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

find_package(Protobuf QUIET)
if(Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Found, Protobuf Support is turned On!")
  message(=============================================================)
  add_definitions(-DPROTO_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${PROTOBUF_LIBRARY})
else(Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
  message(=============================================================)
endif(Protobuf_FOUND)

#========================
# Build Setup
#========================
add_executable(demo
            demo/demo.cpp
            )
add_dependencies(demo rs_sensormanager)

target_link_libraries(demo
                    rs_sensormanager
                    ${roscpp_LIBRARIES}
                    ${PROTOBUF_LIBRARY}  
                       )

