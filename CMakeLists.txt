cmake_minimum_required(VERSION 3.5)
project(rs_driver)

#=============================
#  Compile Demos (TRUE/FALSE)
#=============================
set(COMPILE_DEMOS TRUE)

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
if (POLICY CMP0074) 
  cmake_policy(SET CMP0074 NEW)
endif()
set(CMAKE_BUILD_TYPE Release)
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  message(=============================================================)
  message("-- Cmake run for msvc")
  message(=============================================================)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Zi")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
  set(CompilerFlags
    CMAKE_CXX_FLAGS                       CMAKE_C_FLAGS
    CMAKE_CXX_FLAGS_DEBUG                 CMAKE_C_FLAGS_DEBUG
    CMAKE_CXX_FLAGS_RELEASE               CMAKE_C_FLAGS_RELEASE
	CMAKE_CXX_FLAGS_MINSIZEREL            CMAKE_C_FLAGS_MINSIZEREL
	CMAKE_CXX_FLAGS_RELWITHDEBINFO        CMAKE_C_FLAGS_RELWITHDEBINFO
  )
  foreach(CompilerFlag ${CompilerFlags})
    string(REPLACE "/MT" "/MD" ${CompilerFlag} "${${CompilerFlag}}")
  endforeach()
  add_compile_definitions(_DISABLE_EXTENDED_ALIGNED_STORAGE) # to disable a msvc error
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:100000000")
elseif((CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  message(=============================================================)
  message("-- Cmake run for GNU or Clang")
  message(=============================================================)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
else()
  message(FATAL_ERROR "Unsupported compiler.")
endif()

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
if(COMPILE_DEMOS)
  if (WIN32)
    set(OPENNI_LIBRARY "C:\\Program Files\\OpenNI2\\Lib\\")
    set(OPENNI_INCLUDE_DIRS "C:\\Program Files\\OpenNI2\\Include\\")
  endif()
  set(Boost_USE_STATIC_LIBS ON)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
  find_package(PCL COMPONENTS common visualization io REQUIRED)
  if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
  endif()
else()
# otherwise use pcl's boost
# set (BOOST_ROOT "c:\\boost_1_55_0")
  find_package(Boost COMPONENTS system date_time regex REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

find_package( OpenMP QUIET)
if(OPENMP_FOUND)
  message(=============================================================)
  message("-- OpenMP Found, OpenMP support is turned On!")
  message(=============================================================)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(LIBPCAP_ROOT C:\\local\\libpcap-master\\install\\x64-Release)
  include_directories(${LIBPCAP_ROOT}\\include)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    message("-- link x64 version of libpcap")
    link_directories(${LIBPCAP_ROOT}\\lib\\x64)
  else(CMAKE_SIZEOF_VOID_P EQUAL 8)
    message("-- link x86 version of libpcap")
	link_directories(${LIBPCAP_ROOT}\\lib)
  endif(CMAKE_SIZEOF_VOID_P EQUAL 8)
elseif((CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
endif()

if(PCL_FOUND)
  list(APPEND EXTERNAL_LIBS ${Boost_LIBRARIES} ${PCL_LIBRARIES} pcap)
else()
  list(APPEND EXTERNAL_LIBS ${Boost_LIBRARIES} pcap)
endif()

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

        
