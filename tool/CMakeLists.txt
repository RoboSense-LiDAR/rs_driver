cmake_minimum_required(VERSION 3.5)
project(rs_driver_viewer)
message(=============================================================)
message("-- Start to compile tools")
message(=============================================================)
include_directories(${PROJECT_SOURCE_DIR}/../src)
if(WIN32)
    cmake_policy(SET CMP0074 NEW)
    set(OPENNI_LIBRARY "C:\\Program Files\\OpenNI2\\Lib\\")
    set(OPENNI_INCLUDE_DIRS "C:\\Program Files\\OpenNI2\\Include\\")
endif()
find_package(PCL COMPONENTS common visualization io QUIET REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
if(PCL_FOUND)
add_executable(viewer
               rs_driver_viewer.cpp
              )
target_link_libraries(viewer
                    ${EXTERNAL_LIBS}    
                    ${PCL_LIBRARIES}   
)
else()
message("PCL Not found! Can not compile rs_driver viewer!")
endif()
