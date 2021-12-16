
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder_base_opt.hpp>

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

TEST(TestScanBlock, ctor)
{
  ScanBlock blk(10, 20);
  ASSERT_EQ(blk.start_, 10);
  ASSERT_EQ(blk.end_, 20);

  ASSERT_FALSE(blk.in(5));
  ASSERT_TRUE(blk.in(10));
  ASSERT_TRUE(blk.in(15));
  ASSERT_FALSE(blk.in(20));
  ASSERT_FALSE(blk.in(25));
}

TEST(TestScanBlock, ctorCrossZero)
{
  ScanBlock blk(35000, 10);
  ASSERT_EQ(blk.start_, 35000);
  ASSERT_EQ(blk.end_, 10);

  ASSERT_FALSE(blk.in(34999));
  ASSERT_TRUE(blk.in(35000));
  ASSERT_TRUE(blk.in(0));
  ASSERT_FALSE(blk.in(10));
  ASSERT_FALSE(blk.in(15));
}

TEST(TestScanBlock, ctorBeyondRound)
{
  ScanBlock blk(36100, 36200);
  ASSERT_EQ(blk.start_, 100);
  ASSERT_EQ(blk.end_, 200);
}

TEST(TestDistanceBlock, ctor)
{
  DistanceBlock blk(0.5, 200, 0.75, 150);
  ASSERT_EQ(blk.min_, 0.75);
  ASSERT_EQ(blk.max_, 150);

  ASSERT_FALSE(blk.in(0.45));
  ASSERT_TRUE(blk.in(0.75));
  ASSERT_TRUE(blk.in(0.8));
  ASSERT_TRUE(blk.in(150));
  ASSERT_FALSE(blk.in(150.5));
}

TEST(TestDistanceBlock, ctorNoUseBlock)
{
  DistanceBlock blk(0.5, 200, 0.0, 200.5);
  ASSERT_EQ(blk.min_, 0.5);
  ASSERT_EQ(blk.max_, 200);
}

TEST(TestSplitAngle, toSplit)
{
  {
    SplitAngle sa(10);
    ASSERT_FALSE(sa.toSplit(5));
    ASSERT_TRUE(sa.toSplit(15));
  }

  {
    SplitAngle sa(10);
    ASSERT_FALSE(sa.toSplit(5));
    ASSERT_TRUE(sa.toSplit(10));
    ASSERT_FALSE(sa.toSplit(15));
  }

  {
    SplitAngle sa(10);
    ASSERT_FALSE(sa.toSplit(10));
    ASSERT_FALSE(sa.toSplit(15));
  }
}

TEST(TestSplitAngle, toSplit_Zero)
{
  {
    SplitAngle sa(0);
    ASSERT_FALSE(sa.toSplit(35999));
    ASSERT_TRUE(sa.toSplit(1));
    ASSERT_FALSE(sa.toSplit(2));
  }

  {
    SplitAngle sa(0);
    ASSERT_FALSE(sa.toSplit(35999));
    ASSERT_TRUE(sa.toSplit(0));
    ASSERT_FALSE(sa.toSplit(2));
  }

  {
    SplitAngle sa(0);
    ASSERT_FALSE(sa.toSplit(0));
    ASSERT_FALSE(sa.toSplit(2));
  }
}

