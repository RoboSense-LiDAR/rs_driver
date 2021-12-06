
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder.hpp>
#include <rs_driver/driver/decoder/packet_traverser.hpp>

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
    {0x00} // msop id
    , {0x00} // difop id
    , {0x00} // block id
    , 6 // blocks per packet
    , 2 // channels per block
    , 0.25 // distance resolution

    // firing_ts
    , {0.0,  0.25} // chan_tss
    , 0.50 // block_duration

    // lens center
    , 0 // RX
    , 0 // RY
    , 0 // RZ
  };

  MyPacket pkt = 
  {
          1, 0x00, 0x00, 0x00, 0x00 
      ,   0, 0x00, 0x00, 0x00, 0x00 
      ,  21, 0x00, 0x00, 0x00, 0x00
      ,   0, 0x00, 0x00, 0x00, 0x00
      ,  51, 0x00, 0x00, 0x00, 0x00
      ,   0, 0x00, 0x00, 0x00, 0x00
  };

  int16_t azi;
  double ts;
  uint16_t blk, chan;
  DualReturnPacketTraverser<MyPacket> traverser(const_param, pkt, 0.25);

  // blk 0, chan 0
  ASSERT_FALSE(traverser.isLast());
  ASSERT_TRUE(traverser.get(blk, chan, azi, ts));
  ASSERT_EQ(blk, 0);
  ASSERT_EQ(chan, 0);
  ASSERT_EQ(azi, 1);
  ASSERT_EQ(ts, 0.25);

  // blk 0, chan 1
  traverser.toNext();
  ASSERT_FALSE(traverser.isLast());
  ASSERT_TRUE(traverser.get(blk, chan, azi, ts));
  ASSERT_EQ(blk, 0);
  ASSERT_EQ(chan, 1);
  ASSERT_EQ(azi, 11);
  ASSERT_EQ(ts, 0.50);

  // blk 1, chan 0. 2nd return
  traverser.toNext();
  ASSERT_FALSE(traverser.isLast());
  ASSERT_TRUE(traverser.get(blk, chan, azi, ts));
  ASSERT_EQ(blk, 1);
  ASSERT_EQ(chan, 0);
  ASSERT_EQ(azi, 1);
  ASSERT_EQ(ts, 0.25);

  // blk 1, chan 1. 2nd return
  traverser.toNext();
  ASSERT_FALSE(traverser.isLast());
  ASSERT_TRUE(traverser.get(blk, chan, azi, ts));
  ASSERT_EQ(blk, 1);
  ASSERT_EQ(chan, 1);
  ASSERT_EQ(azi, 11);
  ASSERT_EQ(ts, 0.50);

  // blk 2, chan 0
  traverser.toNext();
  ASSERT_FALSE(traverser.isLast());
  ASSERT_TRUE(traverser.get(blk, chan, azi, ts));
  ASSERT_EQ(blk, 2);
  ASSERT_EQ(chan, 0);
  ASSERT_EQ(azi, 21);
  ASSERT_EQ(ts, 0.75);

  // blk 2, chan 1
  traverser.toNext();
  ASSERT_FALSE(traverser.isLast());
  ASSERT_TRUE(traverser.get(blk, chan, azi, ts));
  ASSERT_EQ(blk, 2);
  ASSERT_EQ(chan, 1);
  ASSERT_EQ(azi, 36);
  ASSERT_EQ(ts, 1.0);

  // blk 3, chan 0. 2nd return.
  traverser.toNext();
  ASSERT_FALSE(traverser.isLast());
  ASSERT_TRUE(traverser.get(blk, chan, azi, ts));
  ASSERT_EQ(blk, 3);
  ASSERT_EQ(chan, 0);
  ASSERT_EQ(azi, 21);
  ASSERT_EQ(ts, 0.75);

  // blk 3, chan 1. 2nd return
  traverser.toNext();
  ASSERT_FALSE(traverser.isLast());

  // blk 4, chan 0. 
  traverser.toNext();
  ASSERT_FALSE(traverser.isLast());
  ASSERT_TRUE(traverser.get(blk, chan, azi, ts));
  ASSERT_EQ(blk, 4);
  ASSERT_EQ(chan, 0);
  ASSERT_EQ(azi, 51);
  ASSERT_EQ(ts, 1.25);

  // blk 4, chan 1. 
  traverser.toNext();
  ASSERT_FALSE(traverser.isLast());
  ASSERT_TRUE(traverser.get(blk, chan, azi, ts));
  ASSERT_EQ(blk, 4);
  ASSERT_EQ(chan, 1);
  ASSERT_EQ(azi, 66);
  ASSERT_EQ(ts, 1.5);
}

