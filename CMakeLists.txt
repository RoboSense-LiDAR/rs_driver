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
  "${PROJECT_SOURCE_DIR}/src/rs_driver/macro/version.h.in"
  "${PROJECT_SOURCE_DIR}/src/rs_driver/macro/version.h"
)

#========================
# Installation Setup
#========================
SET(CMAKE_INSTALL_PREFIX /usr/include)
set(INSTALL_DRIVER_DIR ${CMAKE_INSTALL_PREFIX})

#========================
# Dependencies Setup
#========================
find_package(Boost COMPONENTS system REQUIRED)

#========================
# Build Setup
#========================
include_directories(${PROJECT_SOURCE_DIR}/src)
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
install(DIRECTORY src/rs_driver
        DESTINATION ${INSTALL_DRIVER_DIR}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY src/rs_driver
        DESTINATION ${INSTALL_DRIVER_DIR}
        FILES_MATCHING PATTERN "*.hpp")

