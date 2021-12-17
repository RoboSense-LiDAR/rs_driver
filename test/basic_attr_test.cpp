
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/basic_attr.hpp>

using namespace robosense::lidar;

TEST(TestParseTime, calcTimeUTC)
{
  RSTimestampUTC2 ts = 
  {{0x01, 0x02, 0x03, 0x04, 0x05, 0x06}, {0x11, 0x22, 0x33, 0x44}};

  ASSERT_EQ(calcTimeUTCWithNs(&ts), 0x010203040506 * 1000000 + 0x11223344/1000);
  ASSERT_EQ(calcTimeUTCWithUs(&ts), 0x010203040506 * 1000000 + 0x11223344);
  ASSERT_EQ(calcTimeUTCWithMs(&ts), 0x010203040506 * 1000000 + 0x1122 * 1000 + 0x3344);
}

TEST(TestParseTime, calcTimeYMD)
{
  uint8_t ts[] = {0x15, 0x0a, 0x01, 0x01, 0x02, 0x03, 0x11, 0x22, 0x33, 0x44};

  ASSERT_EQ(calcTimeYMD((RSTimestampYMD*)&ts), 1633021327399124);
}

TEST(TestParseTime, calcTimeHost)
{
  calcTimeHost();
}

TEST(TestParseTemp, calcTemp)
{
  {
    uint8_t temp[] = {0x18, 0x01};
    ASSERT_EQ(calcTemp((RSTemprature*)&temp), 35);
  }

  {
    uint8_t temp[] = {0x18, 0x81};
    ASSERT_EQ(calcTemp((RSTemprature*)&temp), -35);
  }
}

