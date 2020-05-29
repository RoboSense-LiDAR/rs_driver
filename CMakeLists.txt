
#############################################################################
#  Copyright 2020 RoboSense All rights reserved.
#  Suteng Innovation Technology Co., Ltd. www.robosense.ai
#
#  This software is provided to you directly by RoboSense and might
#  only be used to access RoboSense LiDAR. Any compilation,
#  modification, exploration, reproduction and redistribution are
#  restricted without RoboSense's prior consent.
#
#  THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
#  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
#  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
#  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
#  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
###############################################################################
cmake_minimum_required(VERSION 3.5)
project(lidar_driver)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#========================
# Project details / setup
#========================
set(PROJECT_NAME lidar_driver)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
cmake_policy(SET CMP0048 NEW)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_definitions(-std=c++11)
add_compile_options(-W)

#========================
# Installation Setup
#========================
SET(CMAKE_INSTALL_PREFIX /usr/include/rs_lidar_driver)
set(INSTALL_COMMON_DIR ${CMAKE_INSTALL_PREFIX}/common)
set(INSTALL_DRIVER_DIR ${CMAKE_INSTALL_PREFIX}/driver)
set(INSTALL_INTERFACE_DIR ${CMAKE_INSTALL_PREFIX}/interface)
set(INSTALL_MACRO_DIR ${CMAKE_INSTALL_PREFIX}/macro)
set(INSTALL_MSG_DIR ${CMAKE_INSTALL_PREFIX}/msg)
set(INSTALL_UTILITY_DIR ${CMAKE_INSTALL_PREFIX}/utility)



find_package(Boost REQUIRED)

find_package(PCL QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



configure_file (
  "${PROJECT_SOURCE_DIR}/macro/macro.h.in"
  "${PROJECT_SOURCE_DIR}/macro/macro.h"
)

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
add_executable(demo_linux_pcap
              demo/demo_linux_pcap.cpp
               )
target_link_libraries(demo_linux_pcap
                      ${Boost_LIBRARIES}                      
                      pcap
                      pthread
                      )

add_executable(demo_linux_online
              demo/demo_linux_online.cpp
              )
target_link_libraries(demo_linux_online
                    ${Boost_LIBRARIES}                       
                          pcap
                          pthread
)

add_executable(demo_ros_recv_ros_send
              demo/ros/demo_ros_recv_ros_send.cpp
              )
target_link_libraries(demo_ros_recv_ros_send
                    ${Boost_LIBRARIES}    
                    ${roscpp_LIBRARIES}
                    ${PCL_LIBRARIES}         
                          pcap
                          pthread
)

add_executable(demo_pcap_recv_ros_send
              demo/ros/demo_pcap_recv_ros_send.cpp
              )
target_link_libraries(demo_pcap_recv_ros_send
                    ${Boost_LIBRARIES}    
                    ${roscpp_LIBRARIES}
                    ${PCL_LIBRARIES}         
                          pcap
                          pthread
)


#========================
# Installation
#========================        

install(DIRECTORY common/
       DESTINATION ${INSTALL_COMMON_DIR}
       FILES_MATCHING PATTERN "*.h")

install(DIRECTORY driver/
        DESTINATION ${INSTALL_DRIVER_DIR}
        FILES_MATCHING PATTERN "*.h" )

install(DIRECTORY driver/
        DESTINATION ${INSTALL_DRIVER_DIR}
        FILES_MATCHING PATTERN "*.hpp" )

install(DIRECTORY interface/
        DESTINATION ${INSTALL_INTERFACE_DIR}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY macro/
        DESTINATION ${INSTALL_MACRO_DIR}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY msg/
        DESTINATION ${INSTALL_MSG_DIR}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY utility/
        DESTINATION ${INSTALL_UTILITY_DIR}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY utility/
        DESTINATION ${INSTALL_UTILITY_DIR}
        FILES_MATCHING PATTERN "*.hpp")

#========================
# Allow find_package()
#======================== 

# # Change the include location for the case of an install location
# set(CONF_INCLUDE_DIRS ${INSTALL_INCLUDE_DIR} ${CONF_INCLUDE_DIRS} )


# #FIXME: this will only be applied to installed files. Confirm that we want that.
# # gather all the includes but remove ones in the source tree
# # because we added an include for the local yaml-cpp-pm we should also remove it
# list(REMOVE_ITEM CONF_INCLUDE_DIRS ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/include)

# # We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
# configure_file(
#   "cmake/${PROJECT_NAME}Config.cmake.in"
#   "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY
# )

# # The same versioning file can be used for both cases
# configure_file(
#   "cmake/${PROJECT_NAME}ConfigVersion.cmake.in"
#   "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY
# )

# # Install Config.cmake
# install(
#   FILES
#     "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
#     "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#   DESTINATION 
#     "${INSTALL_CMAKE_DIR}/${PROJECT_NAME}"
# )

# #Handle pkg-config file
# set(LIBRARY_CC_ARGS "")
# foreach(_LIB IN LISTS PACKAGE_LIBS EXTERNAL_LIBS) 
#   get_filename_component(_FILE_NAME ${_LIB} NAME)
#   if(${_FILE_NAME} STREQUAL ${_LIB}) # not an absolute path
#     set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} -l${_LIB}")
#   else()
#     set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} ${_LIB}")
#   endif()
# endforeach()
# unset(_LIB)
# unset(_FILE_NAME)

# configure_file(cmake/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
# install(FILES
#   ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc
#   DESTINATION ${INSTALL_PKGCONFIG_DIR}
# )
# unset(LIBRARY_CC_ARGS)


