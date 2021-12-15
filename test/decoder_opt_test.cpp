
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
    ASSERT_EQ(calcTemp((RsTemprature*)&temp), 35);
  }

  {
    uint8_t temp[] = {0x18, 0x81};
    ASSERT_EQ(calcTemp((RsTemprature*)&temp), -35);
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

TEST(TestChanAngles, genUserChan)
{
  std::vector<int32_t> vert_angles;
  std::vector<uint16_t> user_chans;

  vert_angles.push_back(100);
  vert_angles.push_back(0);
  vert_angles.push_back(-100);
  vert_angles.push_back(200);

  ChanAngles::genUserChan (vert_angles, user_chans);
  ASSERT_EQ(user_chans.size(), 4);
  ASSERT_EQ(user_chans[0], 2);
  ASSERT_EQ(user_chans[1], 1);
  ASSERT_EQ(user_chans[2], 0);
  ASSERT_EQ(user_chans[3], 3);
}

TEST(TestChanAngles, loadFromFile)
{
  std::vector<int32_t> vert_angles, horiz_angles;

  ASSERT_EQ(ChanAngles::loadFromFile ("../rs_driver/test/res/angle.csv", vert_angles, horiz_angles), 0);
  ASSERT_EQ(vert_angles.size(), 4);
  ASSERT_EQ(horiz_angles.size(), 4);
  ASSERT_EQ(vert_angles[0], 500);
  ASSERT_EQ(vert_angles[1], 250);
  ASSERT_EQ(vert_angles[2], 0);
  ASSERT_EQ(vert_angles[3], -250);

  ASSERT_EQ(horiz_angles[0], 10);
  ASSERT_EQ(horiz_angles[1], -20);
  ASSERT_EQ(horiz_angles[2], 0);
  ASSERT_EQ(horiz_angles[3], -100);

  ASSERT_EQ(ChanAngles::loadFromFile ("../rs_driver/test/res/angle.csv", vert_angles, horiz_angles), 0);
  ASSERT_EQ(vert_angles.size(), 4);
  ASSERT_EQ(horiz_angles.size(), 4);

  ASSERT_LT(ChanAngles::loadFromFile ("../rs_driver/test/res/non_exist.csv", vert_angles, horiz_angles), 0);
  ASSERT_EQ(vert_angles.size(), 0);
  ASSERT_EQ(horiz_angles.size(), 0);

}

TEST(TestChanAngles, loadFromDifop)
{
  uint8_t vert_angle_arr[] = {0x00, 0x01, 0x02, 
                              0x01, 0x03, 0x04,
                              0x01, 0x05, 0x06,
                              0x00, 0x07, 0x08};
  uint8_t horiz_angle_arr[] = {0x00, 0x11, 0x22,
                               0x01, 0x33, 0x44,
                               0x00, 0x55, 0x66,
                               0x01, 0x77, 0x88};

  std::vector<int32_t> vert_angles, horiz_angles;

  ASSERT_EQ(ChanAngles::loadFromDifop(
        (const RSCalibrationAngle*)vert_angle_arr, 
        (const RSCalibrationAngle*)horiz_angle_arr, 
        4,
        vert_angles, 
        horiz_angles), 0);

  ASSERT_EQ(vert_angles.size(), 4);
  ASSERT_EQ(horiz_angles.size(), 4);
  ASSERT_EQ(vert_angles[0], 258);
  ASSERT_EQ(vert_angles[1], -772);
  ASSERT_EQ(vert_angles[2], -1286);
  ASSERT_EQ(vert_angles[3], 1800);

  ASSERT_EQ(horiz_angles[0], 4386);
  ASSERT_EQ(horiz_angles[1], -13124);
  ASSERT_EQ(horiz_angles[2], 21862);
  ASSERT_EQ(horiz_angles[3], -30600);

  ASSERT_EQ(ChanAngles::loadFromDifop(
        (const RSCalibrationAngle*)vert_angle_arr, 
        (const RSCalibrationAngle*)horiz_angle_arr, 
        4,
        vert_angles, 
        horiz_angles), 0);
  ASSERT_EQ(vert_angles.size(), 4);
  ASSERT_EQ(horiz_angles.size(), 4);
}

TEST(TestChanAngles, memberLoadFromFile)
{
  ChanAngles angles(4);
  ASSERT_EQ(angles.chan_num_, 4);
  ASSERT_EQ(angles.vert_angles_.size(), 4);
  ASSERT_EQ(angles.horiz_angles_.size(), 4);
  ASSERT_EQ(angles.user_chans_.size(), 4);

  ASSERT_EQ(angles.loadFromFile ("../rs_driver/test/res/angle.csv"), 0);
  ASSERT_EQ(angles.user_chans_.size(), 4);
  ASSERT_EQ(angles.toUserChan(0), 3);
  ASSERT_EQ(angles.toUserChan(1), 2);
  ASSERT_EQ(angles.toUserChan(2), 1);
  ASSERT_EQ(angles.toUserChan(3), 0);
}

TEST(TestChanAngles, memberLoadFromFile_fail)
{
  ChanAngles angles(4);
  ASSERT_EQ(angles.chan_num_, 4);

  ASSERT_LT(angles.loadFromFile ("../rs_driver/test/res/non_exist.csv"), 0);
  ASSERT_EQ(angles.vert_angles_.size(), 4);
  ASSERT_EQ(angles.vert_angles_[0], 0);
}

TEST(TestChanAngles, memberLoadFromDifop)
{
  uint8_t vert_angle_arr[] = {0x00, 0x01, 0x02, 
                              0x01, 0x03, 0x04,
                              0x01, 0x05, 0x06,
                              0x00, 0x07, 0x08};
  uint8_t horiz_angle_arr[] = {0x00, 0x11, 0x22,
                               0x01, 0x33, 0x44,
                               0x00, 0x55, 0x66,
                               0x01, 0x77, 0x88};

  ChanAngles angles(4);
  ASSERT_EQ(angles.chan_num_, 4);
  ASSERT_EQ(angles.loadFromDifop(
        (const RSCalibrationAngle*)vert_angle_arr, 
        (const RSCalibrationAngle*)horiz_angle_arr, 
        4), 0);

  ASSERT_EQ(angles.user_chans_.size(), 4);
  ASSERT_EQ(angles.toUserChan(0), 2);
  ASSERT_EQ(angles.toUserChan(1), 1);
  ASSERT_EQ(angles.toUserChan(2), 0);
  ASSERT_EQ(angles.toUserChan(3), 3);
}

TEST(TestChanAngles, memberLoadFromDifop_fail)
{
  uint8_t vert_angle_arr[] = {0x00, 0x01, 0x02, 
                              0x01, 0x03, 0x04,
                              0xFF, 0x05, 0x06,
                              0xFF, 0x07, 0x08};
  uint8_t horiz_angle_arr[] = {0x00, 0x11, 0x22,
                               0x01, 0x33, 0x44,
                               0xFF, 0x55, 0x66,
                               0xFF, 0x77, 0x88};

  ChanAngles angles(4);
  ASSERT_EQ(angles.chan_num_, 4);

  ASSERT_LT(angles.loadFromDifop(
        (const RSCalibrationAngle*)vert_angle_arr, 
        (const RSCalibrationAngle*)horiz_angle_arr, 
        4), 0);
}

TEST(TestTrigon, ctor)
{
  Trigon trigon;
  ASSERT_EQ(trigon.cos(6000), 0.5);
  ASSERT_EQ(trigon.sin(3000), 0.5);
#if 0
  trigon.print();
#endif
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

