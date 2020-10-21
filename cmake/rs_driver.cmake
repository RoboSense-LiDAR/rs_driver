# - Config file for the rs_driver package
# It defines the following variables
#  rs_driver_INCLUDE_DIRS - include directories for rs_driver
#  rs_driver_LIBRARIES    - libraries to link against
#  rs_driver_FOUND        - found flag

# Compute paths
set(rs_driver_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../src)
set(RS_DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../src)

find_package(Boost COMPONENTS system REQUIRED)
list(APPEND rs_driver_LIBRARIES ${Boost_LIBRARIES} pcap pthread)
list(APPEND RS_DRIVER_LIBRARIES ${Boost_LIBRARIES} pcap pthread)

find_package( OpenMP QUIET)
    if(OPENMP_FOUND)
    message(=============================================================)
    message("-- OpenMP Found, OpenMP support is turned On!")
    message(=============================================================)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
endif()
