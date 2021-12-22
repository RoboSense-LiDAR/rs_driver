
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder.hpp>
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
  MyBlock blocks[6];
} MyPacket;

TEST(TestDualPacketTraverser, toNext)
{
  RSDecoderConstParam const_param = 
  {
    0
    , 0
    , 0
    , 0
    , {0x00} // msop id
    , {0x00} // difop id
    , {0x00} // block id
    , 6 // blocks per packet
    , 2 // channels per block
    , 0.0f // distance min
    , 0.0f // distance max
    , 0.25 // distance resolution
    , 0.0 // temperature resolution

    // lens center
    , 0 // RX
    , 0 // RY
    , 0 // RZ

    // firing_ts
    , 0.50 // block_duration
    , {0.0,  0.25} // chan_tss
    , {0.0} // chan_azis
  };

  MyPacket pkt = 
  {
          htons(1), 0x00, 0x00, 0x00, 0x00 
      ,   htons(1), 0x00, 0x00, 0x00, 0x00 
      ,  htons(21), 0x00, 0x00, 0x00, 0x00
      ,  htons(21), 0x00, 0x00, 0x00, 0x00
      ,  htons(51), 0x00, 0x00, 0x00, 0x00
      ,  htons(51), 0x00, 0x00, 0x00, 0x00
  };

  DualReturnBlockDiff<MyPacket> diff(pkt, 
      const_param.BLOCKS_PER_PKT, const_param.BLOCK_DURATION);

  // first block
  ASSERT_EQ(diff.ts(0), 0.0f);
  // still first block
  ASSERT_EQ(diff.ts(1), 0.0f);
  // second block
  ASSERT_EQ(diff.ts(2), 0.5f);

  // first block
  ASSERT_EQ(diff.azimuth(0), 20);
  // still frist block
  ASSERT_EQ(diff.azimuth(1), 20);
  // last block
  ASSERT_EQ(diff.azimuth(4), 30);
  // last block
  ASSERT_EQ(diff.azimuth(5), 30);
}

