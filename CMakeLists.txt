
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
project(rs_driver)
set (PROJECT_VERSION_MAJOR 1)
set (PROJECT_VERSION_MINOR 1)
set (PROJECT_VERSION_PATCH 0)
#========================
# Project details / setup
#========================
set(PROJECT_NAME rs_driver)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
cmake_policy(SET CMP0048 NEW)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_definitions(-std=c++11)
add_compile_options(-W)
configure_file (
  "${PROJECT_SOURCE_DIR}/rs_driver/macro/version.h.in"
  "${PROJECT_SOURCE_DIR}/rs_driver/macro/version.h"
)

#========================
# Installation Setup
#========================
SET(CMAKE_INSTALL_PREFIX /usr/include)
set(INSTALL_DRIVER_DIR ${CMAKE_INSTALL_PREFIX}/rs_driver)

#========================
# Dependencies Setup
#========================
find_package(Boost COMPONENTS system REQUIRED)

#========================
# Build Setup
#========================
include_directories(${PROJECT_SOURCE_DIR})
add_executable(demo
              demo/demo.cpp
              )
target_link_libraries(demo
                    ${Boost_LIBRARIES}       
                      pcap
                      pthread
)

#========================
# Installation
#========================        
install(DIRECTORY rs_driver/
        DESTINATION ${INSTALL_DRIVER_DIR}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY rs_driver/
        DESTINATION ${INSTALL_DRIVER_DIR}
        FILES_MATCHING PATTERN "*.hpp")

