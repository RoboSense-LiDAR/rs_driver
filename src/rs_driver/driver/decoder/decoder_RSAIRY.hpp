/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <rs_driver/driver/decoder/decoder_mech.hpp>
#include <iomanip>

namespace robosense
{
namespace lidar
{
#pragma pack(push, 1)

typedef struct
{
  uint8_t id[4];
  uint8_t reserved_0[4];
  uint8_t pkt_cnt_top2bot[4];
  uint8_t pkt_cnt_bot2top[4];
  uint8_t data_type[2];
  uint8_t reserved_1[2];
  RSTimestampUTC timestamp;
  uint8_t reserved_2;
  uint8_t lidar_type;
  uint8_t lidar_mode;
  uint8_t reserved_3[5];
  RSTemperature temp;
  RSTemperature topboard_temp;
} RSAIRYHeader;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} RSAIRYChannel;
typedef struct
{
  uint8_t id[2];
  uint16_t azimuth;
  RSAIRYChannel channels[48];
} RSAIRYMsopBlock;

typedef struct
{
  RSAIRYHeader header;
  RSAIRYMsopBlock blocks[8];
  uint8_t tail[6];
  uint8_t reserved_4[16];
} RSAIRYMsopPkt;

typedef struct
{
  uint16_t vol_main;
  uint16_t vol_12v;
  uint16_t vol_mcu;
  uint16_t vol_1v;
  uint16_t current_main;
  uint8_t reserved[16];
} RSAIRYStatus;

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV2 eth;
  RSFOV fov;
  uint8_t reserved_1[4];
  RSVersionV1 version;
  uint8_t reserved_2[239];
  uint8_t install_mode;
  uint8_t reserved_3[2];
  RSSN sn;
  uint16_t zero_cal;
  uint8_t return_mode;
  uint8_t reserved_4[167];
  RSCalibrationAngle vert_angle_cali[96];
  RSCalibrationAngle horiz_angle_cali[96];
  uint8_t reserved_5[22];
  RSAIRYStatus status;
  uint32_t qx;
  uint32_t qy;
  uint32_t qz;
  uint32_t qw;
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint8_t ptp_err[4];
  uint32_t pitch;
  uint32_t yaw;
  uint32_t roll;
  uint8_t reserved_6[110];
  uint16_t tail;
} RSAIRYDifopPkt;

typedef struct
{
  uint8_t id[4];
  RSTimestampUTC timestamp;
  uint8_t acceIx[4];
  uint8_t acceIy[4];
  uint8_t acceIz[4];
  uint8_t gyrox[4];
  uint8_t gyroy[4];
  uint8_t gyroz[4];
  int32_t temperature;
  uint8_t odr;       // output data rate
  uint8_t acceIFsr;  // accel full scale range
                     // 0: +/- 2g
                     // 1: +/- 4g
                     // 2: +/- 8g
                     // 3: +/- 16g

  uint8_t gyroIFsr;  // gyro full scale range
                     // 0: +/- 250 dps
                     // 1: +/- 500 dps
                     // 2: +/- 1000 dps
                     // 3: +/- 2000 dps
  uint32_t cnt;
  uint16_t tail;
} RSAIRYImuPkt;
#pragma pack(pop)

enum RSAIRYLidarModel
{
  RSAIRY_CHANNEL_48 = 0,
  RSAIRY_CHANNEL_96 = 1,
  RSAIRY_CHANNEL_192 = 2,
};
template <typename T_PointCloud>
class DecoderRSAIRY : public DecoderMech<T_PointCloud>
{
public:
  constexpr static float FSR_BASE = 32768.0;
  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  void decodeImuPkt(const uint8_t* pkt, size_t size) override;
  virtual ~DecoderRSAIRY() = default;

  explicit DecoderRSAIRY(const RSDecoderParam& param);
  virtual bool isNewFrame(const uint8_t* packet) override;
#ifndef UNIT_TEST
protected:
#endif

  static RSDecoderMechConstParam& getConstParam();
  static RSEchoMode getEchoMode(uint8_t mode);
  RSAIRYLidarModel getLidarModel(uint8_t mode);
  template <typename T_BlockIterator>
  bool internDecodeMsopPkt(const uint8_t* pkt, size_t size);

  RSAIRYLidarModel lidarModel_{ RSAIRY_CHANNEL_96 };
  uint16_t u16ChannelNum_{ 96 };
  bool bInit_{ false };
  bool loaded_install_info_{false};
  uint16_t install_mode_{0xFF};

};

template <typename T_PointCloud>
inline RSDecoderMechConstParam& DecoderRSAIRY<T_PointCloud>::getConstParam()
{
  static RSDecoderMechConstParam param = {
    {
        1248  // msop len
        ,
        1248  // difop len
        ,
        4  // msop id len
        ,
        8  // difop id len
        ,
        { 0x55, 0xAA, 0x05, 0x5A }  // msop id
        ,
        { 0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55 }  // difop id
        ,
        { 0xFF, 0xEE }  // block id
        ,
        96  // laser number
        ,
        8  // blocks per packet
        ,
        48  // channels per block
        ,
        0.1f  // distance min
        ,
        60.0f  // distance max
        ,
        0.005f  // distance resolution
        ,
        0.0625f  // temperature resolution
        ,
        51  // imu len
        ,
        4  // imu id len
        ,
        { 0xAA, 0x55, 0x5A, 0x05 }  // imu id
    }                               // lens center
    ,
    0.0075f  // RX
    ,
    0.00664f  // RY
    ,
    0.04532f  // RZ
  };

  INIT_ONLY_ONCE();

  float blk_ts = 111.080f;

  float firing_tss_airy_m[] = 
  {
    0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
    7.616f, 7.616f, 7.616f, 7.616f, 7.616f, 7.616f, 7.616f, 7.616f,
    16.184f, 16.184f, 16.184f, 16.184f, 16.184f, 16.184f, 16.184f, 16.184f,
    24.752f, 24.752f, 24.752f, 24.752f, 24.752f, 24.752f, 24.752f, 24.752f,
    33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f,
    42.840f, 42.840f, 42.840f, 42.840f, 42.840f, 42.840f, 42.840f, 42.840f,
    52.360f, 52.360f, 52.360f, 52.360f, 52.360f, 52.360f, 52.360f, 52.360f,
    61.880f, 61.880f, 61.880f, 61.880f, 61.880f, 61.880f, 61.880f, 61.880f,
    71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f,
    79.968f, 79.968f, 79.968f, 79.968f, 79.968f, 79.968f, 79.968f, 79.968f,
    88.536f, 88.536f, 88.536f, 88.536f, 88.536f, 88.536f, 88.536f, 88.536f,
    97.104f, 97.104f, 97.104f, 97.104f, 97.104f, 97.104f, 97.104f, 97.104f,
  };

  param.BLOCK_DURATION = blk_ts / 1000000;
  for (uint16_t i = 0; i < sizeof(firing_tss_airy_m)/sizeof(firing_tss_airy_m[0]); i++)
  {
    param.CHAN_TSS[i] = (double)firing_tss_airy_m[i] / 1000000;
    param.CHAN_AZIS[i] = firing_tss_airy_m[i] / blk_ts;
  }

  return param;
}

template <typename T_PointCloud>
inline RSEchoMode DecoderRSAIRY<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode)
  {
    case 0x03:  // dual return
      return RSEchoMode::ECHO_DUAL;
    default:
      return RSEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline RSAIRYLidarModel DecoderRSAIRY<T_PointCloud>::getLidarModel(uint8_t mode)
{
  switch (mode)
  {
    case 0x01:
      this->u16ChannelNum_ = 48;
      return RSAIRYLidarModel::RSAIRY_CHANNEL_48;
    case 0x02:
      this->u16ChannelNum_ = 96;
      return RSAIRYLidarModel::RSAIRY_CHANNEL_96;
    case 0x03:
      this->u16ChannelNum_ = 192;
      return RSAIRYLidarModel::RSAIRY_CHANNEL_192;
    default:
      this->u16ChannelNum_ = 96;
      return RSAIRYLidarModel::RSAIRY_CHANNEL_96;
  }
}
template <typename T_PointCloud>
inline DecoderRSAIRY<T_PointCloud>::DecoderRSAIRY(const RSDecoderParam& param)
  : DecoderMech<T_PointCloud>(getConstParam(), param)
{}

template <typename T_PointCloud>
inline void DecoderRSAIRY<T_PointCloud>::decodeImuPkt(const uint8_t* packet, size_t size)
{
  const RSAIRYImuPkt& pkt = *(const RSAIRYImuPkt*)(packet);
  if (this->imuDataPtr_ && this->cb_imu_data_ && !this->imuDataPtr_->state)
  {
    if (this->param_.use_lidar_clock)
    {
      this->imuDataPtr_->timestamp = parseTimeUTCWithUs(&pkt.timestamp) * 1e-6;
    }
    else
    {
      this->imuDataPtr_->timestamp = getTimeHost() * 1e-6;
    }
    uint8_t acclFsr = 1 << (pkt.acceIFsr + 1);
    float acceUnit = acclFsr / FSR_BASE;

    this->imuDataPtr_->linear_acceleration_x = u8ArrayToInt32(pkt.acceIx, 4) * acceUnit;
    this->imuDataPtr_->linear_acceleration_y = u8ArrayToInt32(pkt.acceIy, 4) * acceUnit;
    this->imuDataPtr_->linear_acceleration_z = u8ArrayToInt32(pkt.acceIz, 4) * acceUnit;

    uint16_t gyroFsr = 250 * (1 << pkt.gyroIFsr);
    float gyroUnit = gyroFsr / FSR_BASE * M_PI / 180;
    this->imuDataPtr_->angular_velocity_x = u8ArrayToInt32(pkt.gyrox, 4) * gyroUnit;
    this->imuDataPtr_->angular_velocity_y = u8ArrayToInt32(pkt.gyroy, 4) * gyroUnit;
    this->imuDataPtr_->angular_velocity_z = u8ArrayToInt32(pkt.gyroz, 4) * gyroUnit;

    this->imuDataPtr_->state = true;

    this->cb_imu_data_();
  }
}
template <typename T_PointCloud>
inline void DecoderRSAIRY<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RSAIRYDifopPkt& pkt = *(const RSAIRYDifopPkt*)(packet);
  this->template decodeDifopCommon<RSAIRYDifopPkt>(pkt);

  this->device_info_.qx = convertUint32ToFloat(ntohl(pkt.qx));
  this->device_info_.qy = convertUint32ToFloat(ntohl(pkt.qy));
  this->device_info_.qz = convertUint32ToFloat(ntohl(pkt.qz));
  this->device_info_.qw = convertUint32ToFloat(ntohl(pkt.qw));

  this->device_info_.x = convertUint32ToFloat(ntohl(pkt.x));
  this->device_info_.y = convertUint32ToFloat(ntohl(pkt.y));
  this->device_info_.z = convertUint32ToFloat(ntohl(pkt.z));
  this->device_info_.state = true;

#ifdef ENABLE_TRANSFORM
  double difop_roll = DEGREE_TO_RADIAN(convertUint32ToFloat(ntohl(pkt.roll)));
  double difop_pitch = DEGREE_TO_RADIAN(convertUint32ToFloat(ntohl(pkt.pitch)));
  double difop_yaw = DEGREE_TO_RADIAN(convertUint32ToFloat(ntohl(pkt.yaw)));
 
  Eigen::AngleAxisd current_rotation_x(difop_roll + this->param_.transform_param.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd current_rotation_y(difop_pitch + this->param_.transform_param.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd current_rotation_z(difop_yaw + this->param_.transform_param.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d current_translation(this->param_.transform_param.x, 
                                           this->param_.transform_param.y, 
                                           this->param_.transform_param.z);
  this->trans_ = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
#endif
  if ((uint16_t)(pkt.install_mode) != 0xFF)
  {
    this->install_mode_ = (uint16_t)(pkt.install_mode);
    this->loaded_install_info_ = true;
  }
}

template <typename T_PointCloud>
inline bool DecoderRSAIRY<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
{
  const RSAIRYMsopPkt& msopPkt = *(const RSAIRYMsopPkt*)(pkt);
  if (msopPkt.header.data_type[0] != 0)
  {
    return false;
  }

  if(!bInit_)
  {
    this->echo_mode_ = getEchoMode (msopPkt.header.data_type[1]);
    this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
      (this->blks_per_frame_ << 1) : this->blks_per_frame_;
    
    lidarModel_ = getLidarModel(msopPkt.header.lidar_mode);
    
    float blk_ts = 111.080f;
    this->mech_const_param_.BLOCK_DURATION = blk_ts / 1000000;

    float firing_tss_48L[] = 
    {
      0.00f, 0.00f, 0.00f, 0.00f, 7.616f, 7.616f, 7.616f, 7.616f, 
      16.184f, 16.184f, 16.184f, 16.184f, 24.752f, 24.752f, 24.752f, 24.752f, 
      33.320f, 33.320f, 33.320f, 33.320f, 42.840f, 42.840f, 42.840f, 42.840f,
      52.360f, 52.360f, 52.360f, 52.360f, 61.880f, 61.880f, 61.880f, 61.880f,
      71.400f, 71.400f, 71.400f, 71.400f, 79.968f, 79.968f, 79.968f, 79.968f,
      88.536f, 88.536f, 88.536f, 88.536f, 97.104f, 97.104f, 97.104f, 97.104f,
    };

    float firing_tss_side[] = 
    {
      0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,   
      11.424f, 11.424f, 11.424f, 11.424f, 11.424f, 11.424f, 11.424f, 11.424f, 
      22.848f, 22.848f, 22.848f, 22.848f, 22.848f, 22.848f, 22.848f, 22.848f, 
      34.272f, 34.272f, 34.272f, 34.272f, 34.272f, 34.272f, 34.272f, 34.272f, 
      45.696f, 45.696f, 45.696f, 45.696f, 45.696f, 45.696f, 45.696f, 45.696f, 
      54.264f, 54.264f, 54.264f, 54.264f, 54.264f, 54.264f, 54.264f, 54.264f, 
      62.832f, 62.832f, 62.832f, 62.832f, 62.832f, 62.832f, 62.832f, 62.832f, 
      71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 
      79.016f, 79.016f, 79.016f, 79.016f, 79.016f, 79.016f, 79.016f, 79.016f, 
      85.680f, 85.680f, 85.680f, 85.680f, 85.680f, 85.680f, 85.680f, 85.680f, 
      92.344f, 92.344f, 92.344f, 92.344f, 92.344f, 92.344f, 92.344f, 92.344f, 
      99.008f, 99.008f, 99.008f, 99.008f, 99.008f, 99.008f, 99.008f, 99.008f, 
    };

    float firing_tss_normal[] = 
    {
      0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
      5.712f, 5.712f, 5.712f, 5.712f, 5.712f, 5.712f, 5.712f, 5.712f, 
      12.376f, 12.376f, 12.376f, 12.376f, 12.376f, 12.376f, 12.376f, 12.376f,
      19.040f, 19.040f, 19.040f, 19.040f, 19.040f, 19.040f, 19.040f, 19.040f,
      25.704f, 25.704f, 25.704f, 25.704f, 25.704f, 25.704f, 25.704f, 25.704f,
      33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f,
      41.888f, 41.888f, 41.888f, 41.888f, 41.888f, 41.888f, 41.888f, 41.888f,
      50.456f, 50.456f, 50.456f, 50.456f, 50.456f, 50.456f, 50.456f, 50.456f,
      59.024f, 59.024f, 59.024f, 59.024f, 59.024f, 59.024f, 59.024f, 59.024f,
      70.448f, 70.448f, 70.448f, 70.448f, 70.448f, 70.448f, 70.448f, 70.448f,
      81.872f, 81.872f, 81.872f, 81.872f, 81.872f, 81.872f, 81.872f, 81.872f,
      93.296f, 93.296f, 93.296f, 93.296f, 93.296f, 93.296f, 93.296f, 93.296f
    };

    if(lidarModel_ ==  RSAIRYLidarModel::RSAIRY_CHANNEL_48)
    {
      for (uint16_t i = 0; i < sizeof(firing_tss_48L)/sizeof(firing_tss_48L[0]); i++)
      {
        this->mech_const_param_.CHAN_TSS[i] = (double)firing_tss_48L[i] / 1000000;
        this->mech_const_param_.CHAN_AZIS[i] = firing_tss_48L[i] / blk_ts;
      }
      bInit_ = true;
    }

    if(lidarModel_ ==  RSAIRYLidarModel::RSAIRY_CHANNEL_96)
    {
      // AIRY: side
      if ((loaded_install_info_) && (this->install_mode_ == 0x1))
      {
        for (uint16_t i = 0; i < sizeof(firing_tss_side)/sizeof(firing_tss_side[0]); i++)
        {
          this->mech_const_param_.CHAN_TSS[i] = (double)firing_tss_side[i] / 1000000;
          this->mech_const_param_.CHAN_AZIS[i] = firing_tss_side[i] / blk_ts;
        }
        bInit_ = true;
      }
      // AIRY: normal
      else if((loaded_install_info_) && (this->install_mode_ == 0x0))
      {
        for (uint16_t i = 0; i < sizeof(firing_tss_normal)/sizeof(firing_tss_normal[0]); i++)
        {
          this->mech_const_param_.CHAN_TSS[i] = (double)firing_tss_normal[i] / 1000000;
          this->mech_const_param_.CHAN_AZIS[i] = firing_tss_normal[i] / blk_ts;
        }
        bInit_ = true;
      }
      // Airy M
      else if((loaded_install_info_) && (this->install_mode_ == 0x2))
      {
        bInit_ = true;
      }
    }
  }
  if (lidarModel_ == RSAIRYLidarModel::RSAIRY_CHANNEL_48)
  {
    if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
    {
      return internDecodeMsopPkt<SingleReturnBlockIterator<RSAIRYMsopPkt>>(pkt, size);
    }
    else
    {
      return internDecodeMsopPkt<DualReturnBlockIterator<RSAIRYMsopPkt>>(pkt, size);
    }
  }
  else if (lidarModel_ == RSAIRYLidarModel::RSAIRY_CHANNEL_96)
  {
    if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
    {
      return internDecodeMsopPkt<TwoInOneBlockIterator<RSAIRYMsopPkt>>(pkt, size);
    }
    else
    {
      return internDecodeMsopPkt<FourInOneBlockIterator<RSAIRYMsopPkt>>(pkt, size);
    }
  }
  else
  {
    RS_ERROR << "Unsupported lidar model:" << lidarModel_ << RS_REND;
    return false;
  }
}

template <typename T_PointCloud>
template <typename T_BlockIterator>
inline bool DecoderRSAIRY<T_PointCloud>::internDecodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RSAIRYMsopPkt& pkt = *(const RSAIRYMsopPkt*)(packet);
  bool ret = false;
  this->temperature_ = parseTempInLe(&(pkt.header.temp)) * this->const_param_.TEMPERATURE_RES;
  this->is_get_temperature_ = true;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs((RSTimestampUTC*)&pkt.header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = ts * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs(ts, (RSTimestampUTC*)&pkt.header.timestamp);
    }
  }
  T_BlockIterator iter(pkt, this->const_param_.BLOCKS_PER_PKT, this->mech_const_param_.BLOCK_DURATION, this->block_az_diff_,
                       this->fov_blind_ts_diff_);
  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RSAIRYMsopBlock& block = pkt.blocks[blk];

    if (memcmp(this->const_param_.BLOCK_ID, block.id, 2) != 0)
    {
      this->cb_excep_(Error(ERRCODE_WRONGMSOPBLKID));
      break;
    }

    int32_t block_az_diff;
    double block_ts_off;
    iter.get(blk, block_az_diff, block_ts_off);

    double block_ts = pkt_ts + block_ts_off;
    int32_t block_az = ntohs(block.azimuth);

    if (this->split_strategy_->newBlock(block_az))
    {
      this->cb_split_frame_(this->u16ChannelNum_, this->cloudTs());
      this->first_point_ts_ = block_ts;
      ret = true;
    }

    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      const RSAIRYChannel& channel = block.channels[chan];
      uint16_t chan_id = chan;
      if (lidarModel_ == RSAIRYLidarModel::RSAIRY_CHANNEL_96 && (blk % 2) == 1)
      {
        chan_id = chan + 48;
      }
      double chan_ts = block_ts + this->mech_const_param_.CHAN_TSS[chan_id];
      int32_t angle_horiz = block_az + (int32_t)((float)block_az_diff * this->mech_const_param_.CHAN_AZIS[chan_id]);

      int32_t angle_vert = this->chan_angles_.vertAdjust(chan_id);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan_id, angle_horiz);
      uint16_t u16RawDistance = ntohs(channel.distance);
      uint16_t u16Distance = u16RawDistance & 0x3FFF;
      uint8_t feature = (u16RawDistance >> 14) & 0x03;
      float distance = u16Distance * this->const_param_.DISTANCE_RES;

      if (this->distance_section_.in(distance) && this->scan_section_.in(angle_horiz_final))
      {
        float x = distance * COS(angle_vert) * COS(angle_horiz_final) + this->lidar_lens_center_Rxy_ * COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - this->lidar_lens_center_Rxy_ * SIN(angle_horiz);
        float z = distance * SIN(angle_vert) + this->mech_const_param_.RZ;
        this->transformPoint(x, y, z);
        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.intensity);
        setRing(point, this->chan_angles_.toUserChan(chan_id));
        setTimestamp(point, chan_ts);
        setFeature(point, feature);
        this->point_cloud_->points.emplace_back(point);
      }
      else if (!this->param_.dense_points)
      {
        typename T_PointCloud::PointT point;
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        setRing(point, this->chan_angles_.toUserChan(chan_id));
        setTimestamp(point, chan_ts);
        setFeature(point, feature);
        this->point_cloud_->points.emplace_back(point);
      }

      this->prev_point_ts_ = chan_ts;
    }
  }

  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
inline bool DecoderRSAIRY<T_PointCloud>::isNewFrame(const uint8_t* packet)
{
  const RSAIRYMsopPkt& pkt = *(const RSAIRYMsopPkt*)(packet);

  int data_type_ = (int)(pkt.header.data_type[0]);
  if (data_type_ > 0)
  {
    return false;
  }

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RSAIRYMsopBlock& block = pkt.blocks[blk];

    if (memcmp(this->const_param_.BLOCK_ID, block.id, 1) != 0)
    {
      break;
    }

    int32_t block_az = ntohs(block.azimuth);
    if (this->pre_split_strategy_->newBlock(block_az))
    {
      return true;
    }
  }

  return false;
}

}  // namespace lidar
}  // namespace robosense
