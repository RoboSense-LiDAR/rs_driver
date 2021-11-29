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

#include <rs_driver/driver/decoder/decoder.hpp>

namespace robosense
{
namespace lidar
{

#pragma pack(push, 1)
typedef struct
{
  uint16_t id;
  uint16_t azimuth;
  RSChannel channels[32];
} RS32MsopBlock;

typedef struct
{
  RSMsopHeader header;
  RS32MsopBlock blocks[12];
  unsigned int index;
  uint16_t tail;
} RS32MsopPkt;

typedef struct
{
  uint64_t id;
  uint16_t rpm;
  RSEthNet eth;
  RSFOV fov;
  uint16_t reserved0;
  uint16_t phase_lock_angle;
  RSVersion version;
  uint8_t reserved_1[242];
  RSSn sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  uint16_t sw_ver;
  RSTimestampYMD timestamp;
  RSStatus status;
  uint8_t reserved_2[5];
  RSDiagno diagno;
  uint8_t gprmc[86];
  RSCalibrationAngle ver_angle_cali[32];
  RSCalibrationAngle hori_angle_cali[32];
  uint8_t reserved_3[586];
  uint16_t tail;
} RS32DifopPkt;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderRS32 : public Decoder<T_PointCloud>
{
public:

  virtual RSDecoderResult processDifopPkt(const uint8_t* pkt, size_t size);
  virtual RSDecoderResult decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual RSDecoderResult TsMsopPkt(const uint8_t* pkt, size_t size);
  virtual uint64_t usecToDelay() 
  {return 0;}
  virtual ~DecoderRS32() = default;

  explicit DecoderRS32(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);

};

template <typename T_PointCloud>
inline DecoderRS32<T_PointCloud>::DecoderRS32(const RSDecoderParam& param,
                                              const LidarConstantParameter& lidar_const_param)
  : Decoder<T_PointCloud>(param, lidar_const_param)
{
}

template <typename T_PointCloud>
inline RSDecoderResult DecoderRS32<T_PointCloud>::processDifopPkt(const uint8_t* packet, size_t size)
{
  uint8_t id[] = {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55};

  const RS32DifopPkt& pkt = *(const RS32DifopPkt*)(packet);

  if (memcmp(id, &(pkt.id), sizeof(id)) != 0)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }

  this->template decodeDifopCommon<RS32DifopPkt>(pkt);

  if (!this->difop_ready_)
  {
    this->chan_angles_.loadFromDifop(pkt.ver_angle_cali, pkt.hori_angle_cali, 32);
  }

  return RSDecoderResult::DECODE_OK;
}

template <typename T_PointCloud>
inline RSDecoderResult DecoderRS32<T_PointCloud>::decodeMsopPkt(const uint8_t* packet, size_t size)
{
  uint8_t id[] = {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0};
  uint8_t block_id[] = {0xFF, 0xEE};
  static const size_t BLOCKS_PER_PKT = 12;

  const RS32MsopPkt& pkt = *(const RS32MsopPkt*)(packet);

  if (memcpy((uint8_t*)&(pkt.header.id), id, sizeof(id)) != 0)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }

  this->current_temperature_ = calcTemp(&(pkt.header.temp));

  double block_timestamp = 0;
  double chan_ts = block_timestamp;
  //uint64_t ts = 0;
  if (this->param_.use_lidar_clock)
  {
    block_timestamp = calcTimeYMD(&pkt.header.timestamp);
  }
  else
  {
    block_timestamp = calcTimeHost();
  }

  float azi_diff = 0;
  for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PKT; blk_idx++)
  {
    const RS32MsopBlock& block = pkt.blocks[blk_idx];

    if (memcmp(&(block.id), block_id, sizeof(block_id)))
    {
      break;
    }

    uint16_t cur_azi = ntohs(block.azimuth);

#if 0
    if (this->echo_mode_ == ECHO_DUAL)
    {
      if (blk_idx % 2 == 0)
      {
        if (blk_idx == 0)
        {
          azi_diff = static_cast<float>(
              (RS_ONE_ROUND + RS_SWAP_SHORT(pkt.blocks[blk_idx + 2].azimuth) - cur_azi) % RS_ONE_ROUND);
        }
        else
        {
          azi_diff = static_cast<float>(
              (RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(pkt.blocks[blk_idx - 2].azimuth)) % RS_ONE_ROUND);
          block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                               (block_timestamp + this->block_duration_);
        }
      }
    }
    else
#endif
    {
      if (blk_idx < (BLOCKS_PER_PKT - 1))
      {
        azi_diff = static_cast<float>((RS_ONE_ROUND + ntohs(pkt.blocks[blk_idx + 1].azimuth) - cur_azi) %
                                      RS_ONE_ROUND);
      }
      else
      {
        azi_diff = static_cast<float>((RS_ONE_ROUND + cur_azi - ntohs(pkt.blocks[blk_idx - 1].azimuth)) %
                                      RS_ONE_ROUND);
      }

      if (blk_idx > 0)
      {
        block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                             (block_timestamp + this->block_duration_);
      }
    }

    azi_diff = (azi_diff > 100) ? this->azi_diff_between_block_theoretical_ : azi_diff; 

    for (uint16_t channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      static const float delta_ts[] = { 0.00,  2.88,  5.76,  8.64, 11.52, 14.40, 17.28, 20.16, 
                                       23.04, 25.92, 28.80, 31.68, 34.56, 37.44, 40.32, 44.64,
                                        1.44,  4.32,  7.20, 10.08, 12.96, 15.84, 18.72, 21.60,
                                       24.48, 27.36, 30.24, 33.12, 36.00, 38.88, 41.76, 46.08};
      static const float delta_block = 55.52;

      const RSChannel& channel = pkt.blocks[blk_idx].channels[channel_idx];

      chan_ts = block_timestamp + delta_ts[channel_idx];
      float azi_channel_ori = cur_azi + azi_diff * delta_ts[channel_idx] / delta_block;

      float distance = ntohs(channel.distance) * this->lidar_const_param_.DIS_RESOLUTION;

      uint16_t azi_channel_final = this->chan_angles_.horizAdjust(channel_idx, (int32_t)azi_channel_ori);

      uint16_t angle_horiz = azi_channel_ori;
      uint16_t angle_vert = this->chan_angles_.vertAdjust(channel_idx);

      typename T_PointCloud::PointT point;
      bool pointValid = false;

#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

        if (this->distance_block_.in(distance) && this->scan_block_.in(azi_channel_final))
      {
        float x =  distance * COS(angle_vert) * COS(azi_channel_final) + this->lidar_const_param_.RX * COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(azi_channel_final) - this->lidar_const_param_.RX * SIN(angle_horiz);
        float z =  distance * SIN(angle_vert) + this->lidar_const_param_.RZ;
        uint8_t intensity = channel.intensity;

        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, intensity);
        pointValid = true;
      }
      else if (!this->param_.is_dense)
      {
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        pointValid = true;
      }

      if (pointValid)
      {
        setRing(point, this->chan_angles_.toUserChan(channel_idx));
        setTimestamp(point, chan_ts);
        this->point_cloud_->points.emplace_back(point);
      }
    }

    this->toSplit(cur_azi, chan_ts);
  }

  return RSDecoderResult::DECODE_OK;
}

template <typename T_PointCloud>
inline RSDecoderResult DecoderRS32<T_PointCloud>::TsMsopPkt(const uint8_t* packet, size_t size)
{
  return RSDecoderResult::DECODE_OK;
}

}  // namespace lidar
}  // namespace robosense
