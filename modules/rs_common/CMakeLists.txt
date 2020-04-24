cmake_minimum_required(VERSION 3.5)
include(CheckSymbolExists)
#========================
# Project details / setup~
#========================
set(PROJECT_NAME rs_common)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
project(${PROJECT_NAME})
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_definitions(-std=c++11)
add_compile_options(-W)
#========================
# Complie Proto
#========================
find_package(Protobuf QUIET)
if(Protobuf_FOUND)
  SET(PROTO_FILE_PATH ${PROJECT_SOURCE_DIR}/include/rs_common/msg/proto_msg)
  file(GLOB PROTOBUF_FILELIST ${PROTO_FILE_PATH}/*.proto)
  message("-- Protobuf Found, Protobuf Support is turned On!")
  foreach(proto_file ${PROTOBUF_FILELIST})
    message(STATUS "COMPILING ${proto_file} USING ${PROTOBUF_PROTOC_EXECUTABLE}")
    execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
                    --proto_path=${PROTO_FILE_PATH}
                    --cpp_out=${PROTO_FILE_PATH}
                    ${proto_file})
  endforeach()
else(Protobuf_FOUND)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
endif(Protobuf_FOUND)

#========================
# Dependencies Setup
#========================

#========================
# Build Setup
#========================
add_library(yaml SHARED IMPORTED)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set_target_properties(yaml PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libyaml-cpp.so)
endif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64") 


add_library(encryptor SHARED IMPORTED)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set_target_properties(encryptor PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libencryptor_x86_64_v_2.so)
endif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64") 

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set_target_properties(encryptor PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libencryptor_aarch64_v_2.so)
endif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")  
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} encryptor)

add_library(rs_yaml SHARED
            src/yaml/yaml_parser.cpp
            src/utility/thread_pool.cpp
            )
target_link_libraries(rs_yaml
                      encryptor
                      stdc++fs
                      yaml
                      )  
target_include_directories(rs_yaml PUBLIC
                  ${PROJECT_SOURCE_DIR}/include
                  ${PROJECT_SOURCE_DIR}/include/rs_common/yaml
                          )
if(Protobuf_FOUND)
add_library(proto_msg SHARED
            ${PROTO_FILE_PATH}/Proto_msg.LidarPoints.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.LidarScan.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.LidarPacket.pb.cc
            )
endif(Protobuf_FOUND)
