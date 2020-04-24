cmake_minimum_required(VERSION 3.5)
include(CheckSymbolExists)
#========================
# debug
#========================
add_definitions(-DDEBUG_LEVEL=0)
#========================
# Project details / setup
#========================
set(PROJECT_NAME rs_sensor)
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
#========================
# Ros part
#========================
find_package(roscpp 1.12 QUIET COMPONENTS
            sensor_msgs
            pcl_conversions
            pcl_ros
            std_msgs
            )

if(roscpp_FOUND)
  message("-- ROS Found, Ros Support is turned On!")
  add_definitions(-DROS_FOUND)
  include_directories(${roscpp_INCLUDE_DIRS})
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${roscpp_LIBRARIES})
else(roscpp_FOUND)
  message("-- ROS Not Found, Ros Support is turned Off!")
endif(roscpp_FOUND)
#========================
# Proto part
#========================
find_package(Protobuf QUIET)
if(Protobuf_FOUND )
  message("-- Protobuf Found, Protobuf Support is turned On!")
  add_definitions(-DPROTO_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${PROTOBUF_LIBRARY})
else(Protobuf_FOUND)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
endif(Protobuf_FOUND)

#=============================
#sensor manager
add_library(rs_sensormanager SHARED
              src/sensor_manager/sensor_manager.cpp
              src/ros/lidar_points_ros_adapter.cpp
              src/ros/lidar_packtes_ros_adapter.cpp      
              src/proto/lidar/lidar_points_proto_adapter.cpp
              src/proto/lidar/lidar_packets_proto_adapter.cpp
)
add_dependencies(rs_sensormanager rs_yaml rs_lidar)
if(Protobuf_FOUND)
add_dependencies(rs_sensormanager proto_msg)
endif(Protobuf_FOUND)
target_link_libraries(rs_sensormanager
                      rs_yaml
                      rs_lidar
                      ${roscpp_LIBRARIES}
                      ${PROTOBUF_LIBRARY}
                      pthread
                      )
if(Protobuf_FOUND)
target_link_libraries(rs_sensormanager proto_msg)
endif(Protobuf_FOUND)
target_include_directories(rs_sensormanager PUBLIC ${PROJECT_SOURCE_DIR}/include)


                
       