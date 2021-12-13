
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder_RS32.hpp>
#include <rs_driver/msg/point_cloud_msg.h>

using namespace robosense::lidar;

template <typename T_PointCloud>
class MyDecoder : public Decoder<T_PointCloud>
{
public:
  MyDecoder(const RSDecoderParam& param, 
    const std::function<void(const Error&)>& excb,
    const RSDecoderConstParam& const_param)
  : Decoder<T_PointCloud>(param, excb, const_param)
  {
  }

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size)
  {
  }

  virtual void decodeMsopPkt(const uint8_t* pkt, size_t size)
  {
  }

};

struct MyDifopPkt
{
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
  RSDecoderConstParam const_param;
  MyDecoder<PointCloud> decoder(param, errCallback, const_param);

  MyDifopPkt pkt;
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

