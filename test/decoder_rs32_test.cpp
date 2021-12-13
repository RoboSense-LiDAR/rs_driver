
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder_RS32.hpp>

using namespace robosense::lidar;

struct PointXYZIRT
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint16_t ring;
  double timestamp;
};

template <typename T_Point>
class PointCloudT
{
public:
  typedef T_Point PointT;
  typedef std::vector<PointT> VectorT;

  uint32_t height = 0;    ///< Height of point cloud
  uint32_t width = 0;     ///< Width of point cloud
  bool is_dense = false;  ///< If is_dense=true, the point cloud does not contain NAN points
  double timestamp = 0.0;
  uint32_t seq = 0;           ///< Sequence number of message
  VectorT points;
};

typedef PointCloudT<PointXYZIRT> PointCloud;

ErrCode errCode = ERRCODE_SUCCESS;

void errCallback(const Error& err)
{
  errCode = err.error_code;
}

TEST(TestDecoder, processDifopPkt_Error)
{
  RSDecoderParam param;
  DecoderRS32<PointCloud> decoder(param, errCallback);

  RS32DifopPkt pkt;
  decoder.processDifopPkt((const uint8_t*)&pkt, 2);
  ASSERT_EQ(errCode, ERRCODE_WRONGPKTLENGTH);

  decoder.processDifopPkt((const uint8_t*)&pkt, sizeof(pkt));
  ASSERT_EQ(errCode, ERRCODE_WRONGPKTHEADER);
}

TEST(TestDecoderRS32, decodeDifopPkt)
{
  RSDecoderParam param;
  DecoderRS32<PointCloud> decoder(param, errCallback);

  RS32DifopPkt pkt;
}

