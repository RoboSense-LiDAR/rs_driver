cmake_minimum_required(VERSION 3.5)
project(rs_driver)

#=============================
#  Compile Demos (TRUE/FALSE)
#=============================
set(COMPILE_DEMOS FASLE)

#========================
#  Setup Version Number
#========================
set (PROJECT_VERSION_MAJOR 1)
set (PROJECT_VERSION_MINOR 2)
set (PROJECT_VERSION_PATCH 1)

#========================
#  Project details / setup
#========================
set(PROJECT_NAME rs_driver)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
cmake_policy(SET CMP0048 NEW)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_definitions(-std=c++14)
add_compile_options(-Wall)
configure_file (
  "${PROJECT_SOURCE_DIR}/src/rs_driver/macro/version.h.in"
  "${PROJECT_SOURCE_DIR}/src/rs_driver/macro/version.h"
)

#========================
#  Installation Setup
#========================
SET(CMAKE_INSTALL_PREFIX /usr/local)
set(INSTALL_DRIVER_DIR ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/include)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake)
set(INSTALL_PKGCONFIG_DIR ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

#========================
#  Dependencies Setup
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
list(APPEND EXTERNAL_LIBS ${Boost_LIBRARIES} pcap pthread)

#========================
#  Build Demos
#========================
if(${COMPILE_DEMOS})
add_subdirectory(${PROJECT_SOURCE_DIR}/demo)
endif(${COMPILE_DEMOS})

#========================
#  Installation
#========================        
set(CONF_INCLUDE_DIRS ${INSTALL_DRIVER_DIR})

configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake @ONLY
)

install(FILES ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake
     DESTINATION ${INSTALL_CMAKE_DIR}/${PROJECT_NAME})

install(DIRECTORY src/
        DESTINATION ${INSTALL_DRIVER_DIR}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY src/
        DESTINATION ${INSTALL_DRIVER_DIR}
        FILES_MATCHING PATTERN "*.hpp")

#========================
#  Uninstall
#======================== 
if(NOT TARGET uninstall)
   configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/cmake_uninstall.cmake.in
  ${PROJECT_BINARY_DIR}/cmake_uninstall.cmake @ONLY
)
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

        