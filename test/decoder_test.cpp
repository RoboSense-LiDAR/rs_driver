
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
    , 0 // blocks per packet
    , 2 // channels per block
  };

  const_param.BLOCK_DURATION = 55.52 / 1000000;

  RSDecoderParam param;
  MyDecoder<PointCloud> decoder(param, errCallback, const_param);

  uint8_t pkt[] = 
  {
    0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55 // msop len
    , 0x02, 0x58 // rpm
    , 0x23, 0x28 // start angle = 9000
    , 0x46, 0x50 // end angle = 18000 
    , 0x00, 0x00, 0x10 // vert angles
    , 0x01, 0x00, 0x20
    , 0x00, 0x00, 0x01 // horiz angles
    , 0x01, 0x00, 0x02
  };

  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt(pkt, sizeof(MyDifopPkt));
  ASSERT_EQ(errCode, ERRCODE_SUCCESS);

  ASSERT_EQ(decoder.rps_, 10);
  ASSERT_EQ(decoder.blks_per_frame_, 1801);
  ASSERT_EQ(decoder.fov_blind_ts_diff_, 0.075f);
  ASSERT_EQ(decoder.chan_angles_.vert_angles_.size(), 2);
  ASSERT_EQ(decoder.chan_angles_.vert_angles_[0], 16);
  ASSERT_EQ(decoder.chan_angles_.horiz_angles_.size(), 2);
  ASSERT_EQ(decoder.chan_angles_.horiz_angles_[0], 1);
}

TEST(TestDecoder, processDifopPkt_invalid_rpm)
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
    , 0x00, 0x00 // rpm = 0
  };

  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt(pkt, sizeof(MyDifopPkt));
  ASSERT_EQ(errCode, ERRCODE_SUCCESS);

  ASSERT_EQ(decoder.rps_, 10);
}



