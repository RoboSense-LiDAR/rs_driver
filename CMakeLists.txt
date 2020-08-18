cmake_minimum_required(VERSION 3.5)
project(rs_driver)
set (PROJECT_VERSION_MAJOR 1)
set (PROJECT_VERSION_MINOR 2)
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
add_compile_options(-Wall)
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
find_package( OpenMP QUIET)
if(OPENMP_FOUND)
message(=============================================================)
message("-- OpenMP Found, OpenMP support is turned On!")
message(=============================================================)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
endif()
#========================
# Build Setup
#========================
include_directories(${PROJECT_SOURCE_DIR}/src)
add_executable(demo_online
              demo/demo_online.cpp
              )
target_link_libraries(demo_online
                    ${Boost_LIBRARIES}       
                      pcap
                      pthread
)
add_executable(demo_pcap
              demo/demo_pcap.cpp
              )
target_link_libraries(demo_pcap
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

