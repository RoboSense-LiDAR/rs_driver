
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/basic_attr.hpp>
#include <rs_driver/utility/dbg.hpp>

using namespace robosense::lidar;

TEST(TestParseTime, parseTimeYMD)
{
  uint8_t ts1[] = {0x15, 0x0a, 0x01, 0x01, 0x02, 0x03, 0x01, 0x11, 0x02, 0x22};
  uint8_t ts2[10];

  ASSERT_EQ(parseTimeYMD((RSTimestampYMD*)ts1), 1633021323273546);

  createTimeYMD(1633021323273546, (RSTimestampYMD*)ts2);
  ASSERT_EQ(memcmp(ts2, ts1, 10), 0);
}

TEST(TestParseTime, parseTimeUTC)
{
  RSTimestampUTC ts1 = 
  {{0x01, 0x02, 0x03, 0x04, 0x05, 0x06}, {0x00, 0x02, 0x33, 0x44}};
  RSTimestampUTC ts2;

  ASSERT_EQ(parseTimeUTCWithNs(&ts1), 0x010203040506 * 1000000 + 0x00023344/1000);

  {
    ASSERT_EQ(parseTimeUTCWithUs(&ts1), 0x010203040506 * 1000000 + 0x00023344);

    createTimeUTCWithUs(0x010203040506 * 1000000 + 0x00023344, &ts2);
    hexdump ((uint8_t*)&ts2, sizeof(ts2), "ts2");
    ASSERT_EQ(memcmp(&ts2, &ts1, sizeof(ts1)), 0);
  }

  ASSERT_EQ(parseTimeUTCWithMs(&ts1), 0x010203040506 * 1000000 + 0x0002 * 1000 + 0x3344);
}

TEST(TestParseTime, getTimeHost)
{
  getTimeHost();
}

TEST(TestParseTemp, parseTemp)
{
  {
    uint8_t temp[] = {0x18, 0x01};
    ASSERT_EQ(parseTemp((RSTemperature*)&temp), 35);
  }

  {
    uint8_t temp[] = {0x18, 0x81};
    ASSERT_EQ(parseTemp((RSTemperature*)&temp), -35);
  }
}

