cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(rs_driver VERSION 1.2.1)

#=============================
#  Compile Demos&Tools (TRUE/FALSE)
#=============================
set(COMPILE_DEMOS TRUE)
set(COMPILE_TOOLS TRUE)

#========================
#  Project setup
#========================
string(TOUPPER ${CMAKE_PROJECT_NAME} CMAKE_PROJECT_NAME_UPPER)
string(TOLOWER ${CMAKE_PROJECT_NAME} CMAKE_PROJECT_NAME_LOWER)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
set(CMAKE_BUILD_TYPE Release)
configure_file (
  "${PROJECT_SOURCE_DIR}/src/rs_driver/macro/version.h.in"
  "${PROJECT_SOURCE_DIR}/src/rs_driver/macro/version.h"
)

#========================
#  Platform cross setup
#========================
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

#========================
#  Installation Setup
#========================
SET(CMAKE_INSTALL_PREFIX /usr/local)
set(INSTALL_DRIVER_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_PROJECT_NAME}/include)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake)
set(INSTALL_PKGCONFIG_DIR ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

#========================
#  OpenMP
#========================
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
#  Boost
#========================
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_ARCHITECTURE "-x64")
if(WIN32)
  set (BOOST_ROOT "c:\\boost_1_55_0")
endif()
find_package(Boost COMPONENTS system date_time regex REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND EXTERNAL_LIBS ${Boost_LIBRARIES})

#========================
#  PCAP
#========================
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
endif()
list(APPEND EXTERNAL_LIBS pcap)

if(WIN32)
include_directories(Q:/Downloads/WpdPack_4_1_2/WpdPack/Include)
link_directories(Q:/Downloads/WpdPack_4_1_2/WpdPack/Lib/x64)
list(APPEND EXTERNAL_LIBS wpcap ws2_32 wsock32)
endif()

#========================
#  Build Demos
#========================
if(${COMPILE_DEMOS})
  add_subdirectory(${PROJECT_SOURCE_DIR}/demo)
endif(${COMPILE_DEMOS})

if(${COMPILE_TOOLS})
  add_subdirectory(${PROJECT_SOURCE_DIR}/tool)
endif(${COMPILE_TOOLS})

#========================
#  Installation
#========================        
configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/${CMAKE_PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${CMAKE_PROJECT_NAME}Config.cmake @ONLY
)

install(FILES ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${CMAKE_PROJECT_NAME}Config.cmake
     DESTINATION ${INSTALL_CMAKE_DIR}/${CMAKE_PROJECT_NAME})

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

        
