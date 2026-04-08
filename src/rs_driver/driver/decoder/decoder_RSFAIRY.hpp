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
  uint16_t temp;
  RSTemperature topboard_temp;
} RSFAIRYHeader;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} RSFAIRYChannel;

typedef struct
{
  uint8_t id[2];
  uint16_t azimuth;
  RSFAIRYChannel channels[48];
} RSFAIRYMsopBlock;

typedef struct
{
  RSFAIRYHeader header;
  RSFAIRYMsopBlock blocks[8];
  uint8_t  tail[6];
  uint8_t reserved_4[16];
} RSFAIRYMsopPkt;

typedef struct
{
  uint16_t vol_main;
  uint16_t vol_12v;
  uint16_t vol_mcu;
  uint16_t vol_1v;
  uint16_t current_main;
  uint8_t reserved[16];
} RSFAIRYStatus;

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV2 eth;
  RSFOV fov;
  uint8_t reserved_1[4];
  RSVersionV1 version;
  uint8_t reserved_2[242];
  RSSN sn;
  uint16_t zero_cal;
  uint8_t return_mode;
  uint8_t reserved_3[167];
  RSCalibrationAngle vert_angle_cali[96];
  RSCalibrationAngle horiz_angle_cali[96];
  uint8_t reserved_4[22];
  RSFAIRYStatus status;
  uint32_t qx;
  uint32_t qy;
  uint32_t qz;
  uint32_t qw;
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint8_t reserved_5[126];
  uint16_t tail;
} RSFAIRYDifopPkt;

typedef struct
{
  uint8_t id[4];
  RSTimestampUTC timestamp;
  uint32_t acceIx;
  uint32_t acceIy;
  uint32_t acceIz;
  uint32_t gyrox;
  uint32_t gyroy;
  uint32_t gyroz;
  int32_t temperature;
  uint8_t reserved_1[3];
  uint32_t cnt;
  uint16_t tail;
} RSFAIRYImuPkt;
#pragma pack(pop)

enum RSFAIRYModel
{
  RSFAIRY_CHANNEL_48 = 0,
  RSFAIRY_CHANNEL_96 = 1,
};
template <typename T_PointCloud>
class DecoderRSFAIRY : public DecoderMech<T_PointCloud>
{
public:

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  void decodeImuPkt(const uint8_t* pkt, size_t size) override;
  virtual ~DecoderRSFAIRY() = default;

  explicit DecoderRSFAIRY(const RSDecoderParam& param);
  virtual bool isNewFrame(const uint8_t* packet) override;
#ifndef UNIT_TEST
protected:
#endif

  static RSDecoderMechConstParam& getConstParam();
  static RSEchoMode getEchoMode(uint8_t mode);
  RSFAIRYModel getLidarModel(uint8_t mode);
  template <typename T_BlockIterator>
  bool internDecodeMsopPkt(const uint8_t* pkt, size_t size);

  RSFAIRYModel lidarModel_{RSFAIRY_CHANNEL_96};
  uint16_t u16ChannelNum_{96};
  bool bInit_{false};

};

template <typename T_PointCloud>
inline RSDecoderMechConstParam& DecoderRSFAIRY<T_PointCloud>::getConstParam()
{
  static RSDecoderMechConstParam param = 
  {
    {
      1248 // msop len
      , 1248 // difop len
      , 4 // msop id len
      , 8 // difop id len
      , {0x55, 0xAA, 0x05, 0x5A} // msop id
      , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
      , {0xFF, 0xEE} // block id
      , 96 // laser number 
      , 8 // blocks per packet
      , 48 // channels per block
      , 0.1f // distance min
      , 150.0f // distance max
      , 0.005f // distance resolution
      , 0.0625f // temperature resolution
      , 51  // imu len
      , 4  // imu id len
      , { 0xAA, 0x55, 0x5A, 0x05 }  // imu id
    }
      // lens center
      , 0.01952f // RX
      , -0.00950f // RY
      , 0.045f // RZ
  };

  INIT_ONLY_ONCE();

  float blk_ts = 69.462f;
  float firing_tss[] = 
  {
    0.00f,  0.00f,  0.00f,  0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
    0.00f,  0.00f,  0.00f,  0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
    0.00f,  0.00f,  0.00f,  0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
    0.00f,  0.00f,  0.00f,  0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
    8.160f, 8.160f, 8.160f, 8.160f, 8.160f, 8.160f, 8.160f, 8.160f,
    8.160f, 8.160f, 8.160f, 8.160f, 8.160f, 8.160f, 8.160f, 8.160f,
    31.008f, 31.008f, 31.008f, 31.008f, 31.008f, 31.008f, 31.008f, 31.008f,
    31.008f, 31.008f, 31.008f, 31.008f, 31.008f, 31.008f, 31.008f, 31.008f,
    53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f,
    53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f,
    53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f,
    53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f, 53.856f,
  };


  param.BLOCK_DURATION = blk_ts / 1000000;
  for (uint16_t i = 0; i < sizeof(firing_tss)/sizeof(firing_tss[0]); i++)
  {
    param.CHAN_TSS[i] = (double)firing_tss[i] / 1000000;
    param.CHAN_AZIS[i] = firing_tss[i] / blk_ts;
  }

  return param;
}

template <typename T_PointCloud>
inline RSEchoMode DecoderRSFAIRY<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode)
  {
    case 0x03: // dual return
      return RSEchoMode::ECHO_DUAL;
    default:
      return RSEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline RSFAIRYModel DecoderRSFAIRY<T_PointCloud>::getLidarModel(uint8_t mode)
{
  switch (mode)
  {
    case 0x01: 
      this->u16ChannelNum_ = 48;
      return RSFAIRYModel::RSFAIRY_CHANNEL_48;
    case 0x02:
      this->u16ChannelNum_ = 96;
      return RSFAIRYModel::RSFAIRY_CHANNEL_96;
    default:
      this->u16ChannelNum_ = 96;
      return RSFAIRYModel::RSFAIRY_CHANNEL_96;
  }
}
template <typename T_PointCloud>
inline DecoderRSFAIRY<T_PointCloud>::DecoderRSFAIRY(const RSDecoderParam& param)
  : DecoderMech<T_PointCloud>(getConstParam(), param)
{
}

template <typename T_PointCloud>
inline void DecoderRSFAIRY<T_PointCloud>::decodeImuPkt(const uint8_t* packet, size_t size)
{
  const RSFAIRYImuPkt& pkt = *(const RSFAIRYImuPkt*)(packet);
  if(this->imuDataPtr_ && this->cb_imu_data_ && !this->imuDataPtr_->state)
  {
    if (this->param_.use_lidar_clock)
    {
      this->imuDataPtr_->timestamp = parseTimeUTCWithUs(&pkt.timestamp) * 1e-6;
    }
    else
    {
      this->imuDataPtr_->timestamp = getTimeHost() * 1e-6;
    }

    this->imuDataPtr_->linear_acceleration_x = convertUint32ToFloat(ntohl(pkt.acceIx));
    this->imuDataPtr_->linear_acceleration_y = convertUint32ToFloat(ntohl(pkt.acceIy));
    this->imuDataPtr_->linear_acceleration_z = convertUint32ToFloat(ntohl(pkt.acceIz));

    this->imuDataPtr_->angular_velocity_x = convertUint32ToFloat(ntohl(pkt.gyrox));
    this->imuDataPtr_->angular_velocity_y = convertUint32ToFloat(ntohl(pkt.gyroy));
    this->imuDataPtr_->angular_velocity_z = convertUint32ToFloat(ntohl(pkt.gyroz));
    
    this->imuDataPtr_->state = true;

    this->cb_imu_data_();
  }
}
template <typename T_PointCloud>
inline void DecoderRSFAIRY<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RSFAIRYDifopPkt& pkt = *(const RSFAIRYDifopPkt*)(packet);
  this->template decodeDifopCommon<RSFAIRYDifopPkt>(pkt);

  this->device_info_.qx = convertUint32ToFloat(ntohl(pkt.qx)) ;
  this->device_info_.qy = convertUint32ToFloat(ntohl(pkt.qy)) ;
  this->device_info_.qz = convertUint32ToFloat(ntohl(pkt.qz)) ;
  this->device_info_.qw = convertUint32ToFloat(ntohl(pkt.qw)) ;

  this->device_info_.x = convertUint32ToFloat(ntohl(pkt.x)) ;
  this->device_info_.y = convertUint32ToFloat(ntohl(pkt.y)) ;
  this->device_info_.z = convertUint32ToFloat(ntohl(pkt.z)) ;
  this->device_info_.state = true;
}

template <typename T_PointCloud>
inline bool DecoderRSFAIRY<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
{
  const RSFAIRYMsopPkt& msopPkt = *(const RSFAIRYMsopPkt*)(pkt);
  if(msopPkt.header.data_type[0]  != 0)
  {
    return false;
  }

  if(!bInit_)
  {
    this->echo_mode_ = getEchoMode (msopPkt.header.data_type[1]);
    this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
      (this->blks_per_frame_ << 1) : this->blks_per_frame_;
    
    lidarModel_ = getLidarModel(msopPkt.header.lidar_mode);

    if(lidarModel_ ==  RSFAIRYModel::RSFAIRY_CHANNEL_48)
    {

      float blk_ts = 69.462f;
      float firing_tss[] = 
      {
        0.00f,  0.00f,  0.00f,  0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
        5.280f, 5.280f, 5.280f, 5.280f, 5.280f, 5.280f, 5.280f, 5.280f,
        14.080f, 14.080f, 14.080f, 14.080f, 14.080f,14.080f,14.080f,14.080f,
        28.160f, 28.160f, 28.160f, 28.160f, 28.160f, 28.160f, 28.160f, 28.160f,
        42.240f, 42.240f, 42.240f, 42.240f, 42.240f, 42.240f, 42.240f, 42.240f,
        51.040f, 51.040f, 51.040f, 51.040f, 51.040f, 51.040f, 51.040f, 51.040f, 
      };

      this->mech_const_param_.BLOCK_DURATION = blk_ts / 1000000;
      for (uint16_t i = 0; i < sizeof(firing_tss)/sizeof(firing_tss[0]); i++)
      {
        this->mech_const_param_.CHAN_TSS[i] = (double)firing_tss[i] / 1000000;
        this->mech_const_param_.CHAN_AZIS[i] = firing_tss[i] / blk_ts;
      }

    }

    bInit_ = true;
  }
  
  if(lidarModel_ ==  RSFAIRYModel::RSFAIRY_CHANNEL_48)
  {
    if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
    {
      return internDecodeMsopPkt<SingleReturnBlockIterator<RSFAIRYMsopPkt>>(pkt, size);
    }
    else
    {
      return internDecodeMsopPkt<DualReturnBlockIterator<RSFAIRYMsopPkt>>(pkt, size);
    }
  }
  else if(lidarModel_ ==  RSFAIRYModel::RSFAIRY_CHANNEL_96)
  {
    if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
    {
      return internDecodeMsopPkt<TwoInOneBlockIterator<RSFAIRYMsopPkt>>(pkt, size);
    }
    else
    {
      return internDecodeMsopPkt<FourInOneBlockIterator<RSFAIRYMsopPkt>>(pkt, size);
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
inline bool DecoderRSFAIRY<T_PointCloud>::internDecodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RSFAIRYMsopPkt& pkt = *(const RSFAIRYMsopPkt*)(packet);
  bool ret = false;
  
  int Temp = pkt.header.temp;
  if (((Temp >> 15) & 0x1) == 1)
    Temp  = (-1) * (0xFFFF - Temp + 0x1);
  this->temperature_ = static_cast<float>(((double)Temp / 8) * this->const_param_.TEMPERATURE_RES);

  this->is_get_temperature_ = true;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs ((RSTimestampUTC*)&pkt.header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = ts * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs (ts, (RSTimestampUTC*)&pkt.header.timestamp);
    }
  }
  T_BlockIterator iter(pkt, this->const_param_.BLOCKS_PER_PKT, this->mech_const_param_.BLOCK_DURATION, 
      this->block_az_diff_, this->fov_blind_ts_diff_);
  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RSFAIRYMsopBlock& block = pkt.blocks[blk];

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
      const RSFAIRYChannel& channel = block.channels[chan]; 
      uint16_t chan_id = chan;
      if (lidarModel_ == RSFAIRYModel::RSFAIRY_CHANNEL_96 && (blk % 2) == 1)
      { 
          chan_id = chan +  48; 
      }
      double chan_ts = block_ts + this->mech_const_param_.CHAN_TSS[chan_id];
      int32_t angle_horiz = block_az + (int32_t)((float)block_az_diff * this->mech_const_param_.CHAN_AZIS[chan_id]);
   
      int32_t angle_vert = this->chan_angles_.vertAdjust(chan_id);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan_id, angle_horiz);

      uint16_t u16RawDistance = ntohs(channel.distance);
      uint16_t u16Distance = u16RawDistance & 0x7FFF;
      uint8_t feature = (u16RawDistance >> 15) & 0x01;
      float distance = u16Distance * this->const_param_.DISTANCE_RES;
      uint16_t user_ring = this->chan_angles_.toUserChan(chan_id);
      // Special user ring handling for 48-channel LiDAR.
      if (user_ring >= 48 && lidarModel_ == RSFAIRYModel::RSFAIRY_CHANNEL_48)
        user_ring -= 48;
      if (this->distance_section_.in(distance) && this->scan_section_.in(angle_horiz_final))
      {
        float x = distance * COS(angle_vert) * COS(angle_horiz_final) + this->lidar_lens_center_Rxy_* COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - this->lidar_lens_center_Rxy_* SIN(angle_horiz);
        float z = distance * SIN(angle_vert) + this->mech_const_param_.RZ;
        this->transformPoint(x, y, z);
        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.intensity);
        setTimestamp(point, chan_ts);
        setRing(point, user_ring);
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
        setTimestamp(point, chan_ts);
        setRing(point, user_ring);
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
inline bool DecoderRSFAIRY<T_PointCloud>::isNewFrame(const uint8_t* packet)
{
  const RSFAIRYMsopPkt& pkt = *(const RSFAIRYMsopPkt*)(packet);

  int data_type_ = (int)(pkt.header.data_type[0]);
  if(data_type_ > 0)
  {
    return false;
  }

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RSFAIRYMsopBlock& block = pkt.blocks[blk];

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
