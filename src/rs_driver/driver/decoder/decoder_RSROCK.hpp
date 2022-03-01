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
#pragma pack(push, 1)

typedef struct
{
  uint16_t azimuth;
  RSChannel channels[4];
} RSROCKChannel;

typedef struct
{
  uint16_t id;
  RSROCKChannel channels[14];
} RSROCKMsopBlock;

typedef struct
{
  RSMsopHeader header;
  RSROCKMsopBlock blocks[6];
  unsigned int index;
  uint16_t tail;
} RSROCKMsopPkt;

// TODO
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
} RSROCKDifopPkt;

#pragma pack(pop)

template <typename T_Point>
class DecoderRSROCK : public DecoderBase<T_Point>
{
public:
  explicit DecoderRSROCK(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);
};

template <typename T_Point>
inline DecoderRSROCK<T_Point>::DecoderRSROCK(const RSDecoderParam& param,
                                             const LidarConstantParameter& lidar_const_param)
  : DecoderBase<T_Point>(param, lidar_const_param)
{
  this->vert_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->hori_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->beam_ring_table_.resize(this->lidar_const_param_.LASER_NUM);

  // TODO temp
  this->vert_angle_list_[0] = 5 * 100;
  this->vert_angle_list_[1] = 2.5 * 100;
  this->vert_angle_list_[2] = 0 * 100;
  this->vert_angle_list_[3] = -2.5 * 100;

  this->hori_angle_list_[0] = -0.2 * 100;
  this->hori_angle_list_[1] = -0.2 * 100;
  this->hori_angle_list_[2] = -0.2 * 100;
  this->hori_angle_list_[3] = -0.2 * 100;

  if (this->param_.max_distance > 200.0f)
  {
    this->param_.max_distance = 200.0f;
  }
  if (this->param_.min_distance < 0.4f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 0.4f;
  }
}

template <typename T_Point>
inline double DecoderRSROCK<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeYMD<RS32MsopPkt>(pkt);
}

template <typename T_Point>
inline RSDecoderResult DecoderRSROCK<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                             int& azimuth)
{
  height = this->lidar_const_param_.LASER_NUM;
  const RSROCKMsopPkt* mpkt_ptr = reinterpret_cast<const RSROCKMsopPkt*>(pkt);
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }

  azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].channels[0].azimuth);
  this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_raw);
  double block_timestamp = this->get_point_time_func_(pkt);
  this->check_camera_trigger_func_(azimuth, pkt);
  float azi_diff = 0;
  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != this->lidar_const_param_.BLOCK_ID)
    {
      break;
    }

    for (size_t firing_idx = 0;
         firing_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK / this->lidar_const_param_.LASER_NUM; firing_idx++)
    {
      int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[firing_idx].azimuth);

      azi_diff = 0.0;  // TODO
      for (int channel_idx = 0; channel_idx < this->lidar_const_param_.LASER_NUM; channel_idx++)
      {
        float azi_channel_ori = cur_azi + azi_diff * 1;  // this->lidar_const_param_.FIRING_FREQUENCY *
                                                         // this->lidar_const_param_.DSR_TOFFSET *
        // static_cast<float>(2 * (channel_idx % 16) + (channel_idx / 16)); // TODO
        int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx);
        float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[firing_idx].channels[channel_idx].distance) *
                         this->lidar_const_param_.DIS_RESOLUTION;
        int angle_horiz = static_cast<int>(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
        int angle_vert = ((this->vert_angle_list_[channel_idx]) + RS_ONE_ROUND) % RS_ONE_ROUND;

        T_Point point;
        if ((distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
            ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
             (!this->angle_flag_ &&
              ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))
        {
          float x = distance * this->checkCosTable(angle_vert) * this->checkCosTable(azi_channel_final) +
                    this->lidar_Rxy_ * this->checkCosTable(angle_horiz - this->lidar_alph0_);
          float y = -distance * this->checkCosTable(angle_vert) * this->checkSinTable(azi_channel_final) -
                    this->lidar_Rxy_ * this->checkSinTable(angle_horiz - this->lidar_alph0_);
          float z = distance * this->checkSinTable(angle_vert) + this->lidar_const_param_.RZ;
          uint8_t intensity = mpkt_ptr->blocks[blk_idx].channels[firing_idx].channels[channel_idx].intensity;
          this->transformPoint(x, y, z);
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, intensity);
        }
        else
        {
          setX(point, NAN);
          setY(point, NAN);
          setZ(point, NAN);
          setIntensity(point, 0);
        }
        setRing(point, this->beam_ring_table_[channel_idx]);
        setTimestamp(point, block_timestamp);
        vec.emplace_back(std::move(point));
      }
    }
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline RSDecoderResult DecoderRSROCK<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  const RSROCKDifopPkt* dpkt_ptr = reinterpret_cast<const RSROCKDifopPkt*>(pkt);
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  this->template decodeDifopCommon<RSROCKDifopPkt>(pkt, LidarType::RSROCK);
  if (!this->difop_flag_)
  {
    this->template decodeDifopCalibration<RSROCKDifopPkt>(pkt, LidarType::RSROCK);
  }
  return RSDecoderResult::DECODE_OK;
}

}  // namespace lidar
}  // namespace robosense
