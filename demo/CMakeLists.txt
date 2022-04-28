cmake_minimum_required(VERSION 3.5)

project(rs_driver_demos)

message(=============================================================)
message("-- Ready to compile demos")
message(=============================================================)

if (${ENABLE_PCL_POINTCLOUD})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

endif (${ENABLE_PCL_POINTCLOUD})

include_directories(${DRIVER_INCLUDE_DIRS})

add_executable(demo_online
              demo_online.cpp)

target_link_libraries(demo_online
                    pthread
                    ${EXTERNAL_LIBS})

if(${ENABLE_PCAP_PARSE})

add_executable(demo_pcap
              demo_pcap.cpp)

target_link_libraries(demo_pcap
                    pthread
                ${EXTERNAL_LIBS})

endif(${ENABLE_PCAP_PARSE})

        
