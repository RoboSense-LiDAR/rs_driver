
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

