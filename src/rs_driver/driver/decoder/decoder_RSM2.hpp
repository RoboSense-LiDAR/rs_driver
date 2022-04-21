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
#include <rs_driver/driver/decoder/decoder_base.hpp>

namespace robosense
{
namespace lidar
{
const size_t M2_MSOP_LEN = 1336;
const size_t M2_DIFOP_LEN = 256;

const uint32_t M2_SINGLE_PKT_NUM = 1260;
const uint32_t M2_DUAL_PKT_NUM = 2520;

#pragma pack(push, 1)

typedef struct
{
  uint16_t distance;
  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t intensity;
  uint8_t point_attribute;
} RSM2Channel;

typedef struct
{
  uint8_t time_offset;
  uint8_t return_seq;
  RSM2Channel channel[5];
} RSM2Block;

typedef struct
{
  uint32_t id;
  uint16_t pkt_cnt;
  uint16_t protocol_version;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimestampUTC timestamp;
  uint8_t reserved[10];
  uint8_t lidar_type;
  int8_t temperature;
} RSM2MsopHeader;

typedef struct
{
  RSM2MsopHeader header;
  RSM2Block blocks[25];
  uint8_t reserved[4];
} RSM2MsopPkt;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderRSM2 : public DecoderBase<T_PointCloud>
{
public:
  DecoderRSM2(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, typename T_PointCloud::VectorT& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);
  RSDecoderResult processMsopPkt(const uint8_t* pkt, size_t size, typename T_PointCloud::VectorT& pointcloud_vec,
                                 int& height);

private:
  uint32_t max_pkt_num_;
  uint32_t last_pkt_cnt_;
  double last_pkt_time_;
};

template <typename T_PointCloud>
inline DecoderRSM2<T_PointCloud>::DecoderRSM2(const RSDecoderParam& param,
                                              const LidarConstantParameter& lidar_const_param)
  : DecoderBase<T_PointCloud>(param, lidar_const_param)
  , max_pkt_num_(M2_SINGLE_PKT_NUM)
  , last_pkt_cnt_(1)
  , last_pkt_time_(0)
{
  this->msop_pkt_len_ = M2_MSOP_LEN;
  this->difop_pkt_len_ = M2_DIFOP_LEN;

  if (this->param_.max_distance > 200.0f)
  {
    this->param_.max_distance = 200.0f;
  }
  if (this->param_.min_distance < 0.2f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 0.2f;
  }
  this->time_duration_between_blocks_ = 5 * 1e-6;
}

template <typename T_PointCloud>
inline double DecoderRSM2<T_PointCloud>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeUTC<RSM2MsopPkt>(pkt, LidarType::RSM2);
}

template <typename T_PointCloud>
inline RSDecoderResult DecoderRSM2<T_PointCloud>::processMsopPkt(const uint8_t* pkt, size_t size,
                                                                 typename T_PointCloud::VectorT& pointcloud_vec,
                                                                 int& height)
{
  if (size != this->msop_pkt_len_)
  {
    return WRONG_PKT_LENGTH;
  }

  int azimuth = 0;
  RSDecoderResult ret = decodeMsopPkt(pkt, pointcloud_vec, height, azimuth);
  this->pkt_count_++;
  switch (this->param_.split_frame_mode)
  {
    case SplitFrameMode::SPLIT_BY_ANGLE:
    case SplitFrameMode::SPLIT_BY_FIXED_PKTS:
      return ret;
    case SplitFrameMode::SPLIT_BY_CUSTOM_PKTS:
      if (this->pkt_count_ >= this->param_.num_pkts_split)
      {
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = RS_ONE_ROUND;
        return FRAME_SPLIT;
      }
      break;
    default:
      break;
  }
  return DECODE_OK;
}

inline int16_t RS_SWAP_INT16(int16_t value)
{
  uint8_t* v = (uint8_t*)&value;

  uint8_t temp;
  temp = v[0];
  v[0] = v[1];
  v[1] = temp;

  return value;
}

template <typename T_PointCloud>
inline RSDecoderResult DecoderRSM2<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, typename T_PointCloud::VectorT& vec,
                                                                int& height, int& azimuth)
{
  height = this->lidar_const_param_.LASER_NUM;
  const RSM2MsopPkt* mpkt_ptr = (RSM2MsopPkt*)pkt;
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  this->current_temperature_ = static_cast<float>(mpkt_ptr->header.temperature - 80);
  this->protocol_ver_ = RS_SWAP_SHORT(mpkt_ptr->header.protocol_version);
  double pkt_timestamp = 0;
  switch (mpkt_ptr->blocks[0].return_seq)
  {
    case 0:
      pkt_timestamp = this->get_point_time_func_(pkt);
      break;
    case 1:
      pkt_timestamp = this->get_point_time_func_(pkt);
      last_pkt_time_ = pkt_timestamp;
      break;
    case 2:
      pkt_timestamp = last_pkt_time_;
      break;
  }

  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    const RSM2Block& blk = mpkt_ptr->blocks[blk_idx];
    double point_time = pkt_timestamp + blk.time_offset * 1e-6;
    for (size_t channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      const RSM2Channel& channel = blk.channel[channel_idx];

      typename T_PointCloud::PointT point;
      bool pointValid = false;
      float distance = RS_SWAP_SHORT(channel.distance) * this->lidar_const_param_.DIS_RESOLUTION;
      if (distance <= this->param_.max_distance && distance >= this->param_.min_distance)
      {
        float x = RS_SWAP_INT16(channel.x) / 32768.0 * distance;
        float y = RS_SWAP_INT16(channel.y) / 32768.0 * distance;
        float z = RS_SWAP_INT16(channel.z) / 32768.0 * distance;

        uint8_t intensity = channel.intensity;

        this->transformPoint(x, y, z);
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
        setTimestamp(point, point_time);
        setRing(point, channel_idx + 1);
        vec.emplace_back(std::move(point));
      }
    }
  }
  unsigned int pkt_cnt = RS_SWAP_SHORT(mpkt_ptr->header.pkt_cnt);

  // TODO whatif packet loss or seq unorder
  if (pkt_cnt == max_pkt_num_ || pkt_cnt < last_pkt_cnt_)
  {
    last_pkt_cnt_ = 1;
    return RSDecoderResult::FRAME_SPLIT;
  }
  last_pkt_cnt_ = pkt_cnt;
  return RSDecoderResult::DECODE_OK;
}

template <typename T_PointCloud>
inline RSDecoderResult DecoderRSM2<T_PointCloud>::decodeDifopPkt(const uint8_t* pkt)
{
  RSM1DifopPkt* dpkt_ptr = (RSM1DifopPkt*)pkt;
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  if (!this->difop_flag_)
  {
    this->echo_mode_ = this->getEchoMode(LidarType::RSM2, dpkt_ptr->return_mode);
    if (this->echo_mode_ == RSEchoMode::ECHO_DUAL)
    {
      max_pkt_num_ = M2_DUAL_PKT_NUM;
    }
    this->difop_flag_ = true;
  }
  return RSDecoderResult::DECODE_OK;
}

}  // namespace lidar
}  // namespace robosense
