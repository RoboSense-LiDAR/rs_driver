cmake_minimum_required(VERSION 3.5)
project(rs_driver_demos)
message(=============================================================)
message("-- Ready to compile demos")
message(=============================================================)

find_package(PCL QUIET REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${DRIVER_INCLUDE_DIRS})
add_executable(demo_online
              demo_online.cpp
              )
target_link_libraries(demo_online
                    pthread
                    ${EXTERNAL_LIBS}       
)
add_executable(demo_pcap
              demo_pcap.cpp
              )
target_link_libraries(demo_pcap
                    pthread
                ${EXTERNAL_LIBS}       
)

        
