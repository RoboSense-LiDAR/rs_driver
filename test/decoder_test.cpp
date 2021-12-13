
#include <gtest/gtest.h>

#include "rs_driver/msg/point_cloud_msg.h"
#include <rs_driver/driver/decoder/decoder.hpp>
#include <rs_driver/utility/dbg.h>

using namespace robosense::lidar;

typedef PointCloudT<PointXYZIRT> PointCloud;

#pragma pack(push, 1)
struct MyMsopPkt
{
  uint8_t id[8];
};

struct MyDifopPkt
{
  uint8_t id[8];
  uint16_t rpm;
  RSFOV fov;
  RSCalibrationAngle ver_angle_cali[2];
  RSCalibrationAngle hori_angle_cali[2];
};
#pragma pack(pop)

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

  virtual void decodeDifopPkt(const uint8_t* packet, size_t size)
  {
    const MyDifopPkt& pkt = *(const MyDifopPkt*)(packet);
    this->template decodeDifopCommon<MyDifopPkt>(pkt);
  }

  virtual void decodeMsopPkt(const uint8_t* pkt, size_t size)
  {
  }

};

ErrCode errCode = ERRCODE_SUCCESS;

void errCallback(const Error& err)
{
  errCode = err.error_code;
}

TEST(TestDecoder, processDifopPkt_fail)
{
    RSDecoderConstParam const_param = 
    {
        sizeof(MyMsopPkt) // msop len
      , sizeof(MyDifopPkt) // difop len
      , 8 // msop id len
      , 8 // difop id len
      , {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0} // msop id
      , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
      , {0xFF, 0xEE} // block id
      , 12 // blocks per packet
      , 32 // channels per block
    };

  RSDecoderParam param;
  MyDecoder<PointCloud> decoder(param, errCallback, const_param);

  MyDifopPkt pkt;
  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt((const uint8_t*)&pkt, 10);
  ASSERT_EQ(errCode, ERRCODE_WRONGPKTLENGTH);

  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt((const uint8_t*)&pkt, sizeof(pkt));
  ASSERT_EQ(errCode, ERRCODE_WRONGPKTHEADER);
}

TEST(TestDecoder, processDifopPkt)
{
    RSDecoderConstParam const_param = 
    {
        sizeof(MyMsopPkt) // msop len
      , sizeof(MyDifopPkt) // difop len
      , 8 // msop id len
      , 8 // difop id len
      , {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0} // msop id
      , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
      , {0xFF, 0xEE} // block id
      , 12 // blocks per packet
      , 32 // channels per block
    };

  RSDecoderParam param;
  MyDecoder<PointCloud> decoder(param, errCallback, const_param);

  uint8_t pkt[] = 
  {
     0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55 // msop len
    , 0x02, 0x58 // rpm
  };

  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt(pkt, sizeof(MyDifopPkt));
  ASSERT_EQ(errCode, ERRCODE_SUCCESS);

  ASSERT_EQ(decoder.rps(), 10);
}

