
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder_RS32.hpp>

using namespace robosense::lidar;

typedef PointCloudT<PointXYZIRT> PointCloud;

ErrCode errCode = ERRCODE_SUCCESS;

void errCallback(const Error& err)
{
  errCode = err.error_code;
}

TEST(TestDecoderRS32, decodeDifopPkt)
{
  RSDecoderParam param;
  DecoderRS32<PointCloud> decoder(param, errCallback);

  RS32DifopPkt pkt;
}

