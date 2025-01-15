#include <gtest/gtest.h>

#include <rs_driver/common/rs_common.hpp>

using namespace robosense::lidar;

// Test RS_SWAP_INT16
TEST(RS_SWAP_INT16Test, Basic) {
    // Test little endian to big endian conversion
    EXPECT_EQ(RS_SWAP_INT16(0x1234), 0x3412);
    EXPECT_EQ(RS_SWAP_INT16(0x0001), 0x0100);

    // Test big endian to little endian conversion
    EXPECT_EQ(RS_SWAP_INT16(0x3412), 0x1234);
    EXPECT_EQ(RS_SWAP_INT16(0x0100), 0x0001);

    // Test zero and max values
    EXPECT_EQ(RS_SWAP_INT16(0x0000), 0x0000);
    EXPECT_EQ(RS_SWAP_INT16(0xFFFF), -1);
}


// Test u8ArrayToInt32
TEST(U8ArrayToInt32Test, ValidInput) {
    uint8_t data[] = {0x00, 0x00, 0x00, 0x7B}; // 123 in decimal
    int32_t result = u8ArrayToInt32(data, sizeof(data));
    EXPECT_EQ(result, 123);
}

TEST(U8ArrayToInt32Test, InvalidLength) {
    uint8_t data[] = {0x00, 0x00, 0x00}; // Not enough bytes for a 32-bit integer
    int32_t result = u8ArrayToInt32(data, sizeof(data));
    EXPECT_EQ(result, 0); // 0 is returned for invalid input
}

// Test convertUint32ToFloat
TEST(ConvertUint32ToFloatTest, ValidInput) {
    uint32_t byteArray = 0x4048F5C3; // 3.14 in decimal
    float result = convertUint32ToFloat(byteArray);
    EXPECT_FLOAT_EQ(result, 3.14f); // using EXPECT_FLOAT_EQ for float comparison
}

TEST(ConvertUint32ToFloatTest, ZeroInput) {
    uint32_t byteArray = 0x00000000;
    float result = convertUint32ToFloat(byteArray);
    EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST(ConvertUint32ToFloatTest, NegativeInput) {
    uint32_t byteArray = 0xC0000000; // 对应浮点数 -2.0
    float result = convertUint32ToFloat(byteArray);
    EXPECT_FLOAT_EQ(result, -2.0f);
}
