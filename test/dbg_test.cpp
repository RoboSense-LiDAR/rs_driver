#include <gtest/gtest.h>

#include <rs_driver/utility/dbg.hpp>

using namespace robosense::lidar;


TEST(HexdumpTest, Basic) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    hexdump(data, sizeof(data), "Test Data");
    SUCCEED(); // if no exception thrown
}

