
#include <gtest/gtest.h>

#include "rs_driver/msg/point_cloud_msg.h"
#include <rs_driver/driver/decoder/decoder_RS32.hpp>

using namespace robosense::lidar;

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloud;

static ErrCode errCode = ERRCODE_SUCCESS;
static void errCallback(const Error& err)
{
  errCode = err.error_code;
}

TEST(TestDecoderRS32, getEchoMode)
{
  RSDecoderParam param;
  DecoderRS32<PointCloud> decoder(param, errCallback);

  ASSERT_TRUE(decoder.getEchoMode(0) == RSEchoMode::ECHO_DUAL);
  ASSERT_TRUE(decoder.getEchoMode(1) == RSEchoMode::ECHO_SINGLE);
  ASSERT_TRUE(decoder.getEchoMode(2) == RSEchoMode::ECHO_SINGLE);
}

TEST(TestDecoderRS32, decodeDifopPkt)
{
  RSDecoderParam param;
  DecoderRS32<PointCloud> decoder(param, errCallback);

  RS32DifopPkt pkt;

  pkt.rpm = htons(1200);
  pkt.return_mode = 0;
  decoder.decodeDifopPkt((uint8_t*)&pkt, sizeof(pkt));
  ASSERT_EQ(decoder.echo_mode_, RSEchoMode::ECHO_DUAL);
  ASSERT_EQ(decoder.rps_, 20);
}

TEST(TestDecoderRS32, decodeMsopPkt)
{
  RSDecoderParam param;
  DecoderRS32<PointCloud> decoder(param, errCallback);

  RS32MsopPkt pkt = 
  {
    0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0, // msop id
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved_1
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ts_YMD
    0x00, // lidar type
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved_2
    0x00, 0x00, // temprature
    0x00, 0x00 // reserved_3
  };

  decoder.decodeMsopPkt((uint8_t*)&pkt, sizeof(pkt));
  ASSERT_EQ(decoder.getTemperature(), 0);
}
