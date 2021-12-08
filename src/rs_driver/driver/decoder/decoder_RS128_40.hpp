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
#include <rs_driver/driver/decoder/decoder_RS128.hpp>
namespace robosense
{
namespace lidar
{

template <typename T_Point>
class DecoderRS128_40 : public DecoderRS128<T_Point>
{
public:
  explicit DecoderRS128_40(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
};

template <typename T_Point>
inline DecoderRS128_40<T_Point>::DecoderRS128_40(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : DecoderRS128<T_Point>(param, lidar_const_param)
{
}

template <typename T_Point>
inline RSDecoderResult DecoderRS128_40<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                            int& azimuth)
{
  height = this->lidar_const_param_.LASER_NUM;

  const RS128MsopPkt* mpkt_ptr = reinterpret_cast<const RS128MsopPkt*>(pkt);
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }

  this->protocol_ver_ = RS_SWAP_SHORT(mpkt_ptr->header.protocol_version);

  azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  this->check_camera_trigger_func_(azimuth, pkt);

  this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_low, mpkt_ptr->header.temp_high);

  double block_timestamp = this->get_point_time_func_(pkt);
  float azi_diff = 0;
  for (uint16_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != this->lidar_const_param_.BLOCK_ID)
    {
      break;
    }

    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
#if 0
    if (this->echo_mode_ == ECHO_DUAL)
    {
      azi_diff = static_cast<float>(
          (RS_ONE_ROUND + RS_SWAP_SHORT(mpkt_ptr->blocks[2].azimuth) - RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth)) %
          RS_ONE_ROUND);
      if (RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth) == RS_SWAP_SHORT(mpkt_ptr->blocks[1].azimuth))  ///< AAB
      {
        if (blk_idx == 2)
        {
          block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                               (block_timestamp + this->time_duration_between_blocks_);
        }
      }
      else  ///< ABB
      {
        if (blk_idx == 1)
        {
          block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                               (block_timestamp + this->time_duration_between_blocks_);
        }
      }
    }
    else
#endif
    {
      //if (blk_idx == 0)
      if (blk_idx < (this->lidar_const_param_.BLOCKS_PER_PKT - 1))
      {
        azi_diff = static_cast<float>((RS_ONE_ROUND + RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth) - cur_azi) %
                                      RS_ONE_ROUND);
      }
      else
      {
        azi_diff = static_cast<float>((RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth)) %
                                      RS_ONE_ROUND);
      }

      if (blk_idx > 0)
      {
        block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                             (block_timestamp + this->time_duration_between_blocks_);
      }
    }

    //azi_diff = (azi_diff > 100) ? this->azi_diff_between_block_theoretical_ : azi_diff;

    for (uint16_t channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      static const float tss[128] = 
      {
        0.24, 0.24, 0.24, 0.24, 1.084, 1.084, 1.084, 1.084,
        1.932, 1.932, 1.932, 1.932, 2.872, 2.872, 2.872, 2.872, 
        3.824, 3.824, 3.824, 3.824, 4.684, 4.684, 4.684, 4.684, 
        5.540, 5.540, 5.540, 5.540, 6.496, 6.496, 6.496, 6.496,
        7.440, 7.440, 7.440, 7.440, 9.348, 9.348, 9.348, 9.348,
        11.260, 11.260, 11.260, 11.260, 13.264, 13.264, 13.264, 13.264,
        15.280, 15.280, 15.280, 15.280, 17.204, 17.204, 17.204, 17.204,    
        19.124, 19.124, 19.124, 19.124, 21.144, 21.144, 21.144, 21.144,
        23.152, 23.152, 23.152, 23.152, 25.060, 25.060, 25.060, 25.060, 
        26.972, 26.972, 26.972, 26.972, 28.976, 28.976, 28.976, 28.976, 
        30.992, 30.992, 30.992, 30.992, 32.916, 32.916, 32.916, 32.916, 
        34.836, 34.836, 34.836, 34.836, 36.856, 36.856, 36.856, 36.856, 
        38.864, 38.864, 38.864, 38.864, 40.272, 40.272, 40.272, 40.272, 
        41.684, 41.684, 41.684, 41.684, 43.188, 43.188, 43.188, 43.188, 
        44.704, 44.704, 44.704, 44.704, 46.128, 46.128, 46.128, 46.128, 
        47.548, 47.548, 47.548, 47.548, 49.068, 49.068, 49.068, 49.068
      };
      static const float blk_ts = 50.984;

      float azi_channel_ori = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth) + 
        azi_diff * tss[channel_idx] / blk_ts;

#if 0
      int dsr_temp = (channel_idx / 4) % 16;
      float azi_channel_ori = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth) +
        (azi_diff * static_cast<float>(dsr_temp) * this->lidar_const_param_.DSR_TOFFSET *
         this->lidar_const_param_.FIRING_FREQUENCY);
#endif

      int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx);
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) *
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
          this->lidar_const_param_.RX * this->checkCosTable(angle_horiz);
        float y = -distance * this->checkCosTable(angle_vert) * this->checkSinTable(azi_channel_final) -
          this->lidar_const_param_.RX * this->checkSinTable(angle_horiz);
        float z = distance * this->checkSinTable(angle_vert) + this->lidar_const_param_.RZ;
        uint8_t intensity = mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
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

  return RSDecoderResult::DECODE_OK;
}

}  // namespace lidar
}  // namespace robosense
