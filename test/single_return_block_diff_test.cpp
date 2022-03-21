
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder_mech.hpp>
#include <rs_driver/driver/decoder/block_diff.hpp>

using namespace robosense::lidar;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} MyChannel;

typedef struct
{
  uint16_t azimuth;
  MyChannel channels[2];
} MyBlock;

typedef struct
{
  MyBlock blocks[3];
} MyPacket;

TEST(TestSingleReturnBlockDiff, ctor)
{
  MyPacket pkt = 
  {
        htons(1), 0x00, 0x00, 0x00, 0x00 
    ,  htons(21), 0x00, 0x00, 0x00, 0x00 
    ,  htons(51), 0x00, 0x00, 0x00, 0x00
  };

  SingleReturnBlockDiff<MyPacket> diff(pkt, 
      3,     // blocks per packet
      0.5f,  // block_duration
      25,    // block_az_duraton
      2.0f); // fov_blind_duration

  int32_t az_diff;
  float ts;

  // first block
  diff.getDiff (0, az_diff, ts);
  ASSERT_EQ(az_diff, 20);
  ASSERT_EQ(ts, 0.0f);

  diff.getDiff (1, az_diff, ts);
  ASSERT_EQ(az_diff, 30);
  ASSERT_EQ(ts, 0.5f);

  diff.getDiff (2, az_diff, ts);
  ASSERT_EQ(az_diff, 25);
  ASSERT_EQ(ts, 1.0f);
}

TEST(TestSingleReturnBlockDiff, ctor_fov)
{
  MyPacket pkt = 
  {
        htons(1), 0x00, 0x00, 0x00, 0x00 
    ,  htons(21), 0x00, 0x00, 0x00, 0x00 
    ,  htons(141), 0x00, 0x00, 0x00, 0x00
  };

  SingleReturnBlockDiff<MyPacket> diff(pkt, 
      3,     // blocks per packet
      0.5f,  // block_duration
      25,    // block_az_duraton
      2.0f); // fov_blind_duration

  int32_t az_diff;
  float ts;

  // first block
  diff.getDiff (0, az_diff, ts);
  ASSERT_EQ(az_diff, 20);
  ASSERT_EQ(ts, 0.0f);

  diff.getDiff (1, az_diff, ts);
  ASSERT_EQ(az_diff, 25);
  ASSERT_EQ(ts, 0.5f);

  diff.getDiff (2, az_diff, ts);
  ASSERT_EQ(az_diff, 25);
  ASSERT_EQ(ts, 2.5f);
}

