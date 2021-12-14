
cmake_minimum_required(VERSION 3.5)

project(rs_driver_test)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${DRIVER_INCLUDE_DIRS})

add_definitions("-DUNIT_TEST")

add_executable(rs_driver_test
              rs_driver_test.cpp
              packet_test.cpp
              sync_queue_test.cpp
              single_return_block_diff_test.cpp
              dual_return_block_diff_test.cpp
              decoder_opt_test.cpp
              decoder_test.cpp
              decoder_rs32_test.cpp)
              

target_link_libraries(rs_driver_test
                      ${GTEST_LIBRARIES}
                      ${EXTERNAL_LIBS})
        
