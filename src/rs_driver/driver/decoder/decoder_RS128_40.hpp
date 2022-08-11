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
    {
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

    azi_diff = (azi_diff > 100) ? this->azi_diff_between_block_theoretical_ : azi_diff;

    for (uint16_t channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      static const float tss[128] = 
      {
        0.24f, 0.24f, 0.24f, 0.24f, 1.084f, 1.084f, 1.084f, 1.084f,
        1.932f, 1.932f, 1.932f, 1.932f, 2.872f, 2.872f, 2.872f, 2.872f, 
        3.824f, 3.824f, 3.824f, 3.824f, 4.684f, 4.684f, 4.684f, 4.684f, 
        5.540f, 5.540f, 5.540f, 5.540f, 6.496f, 6.496f, 6.496f, 6.496f,
        7.440f, 7.440f, 7.440f, 7.440f, 9.348f, 9.348f, 9.348f, 9.348f,
        11.260f, 11.260f, 11.260f, 11.260f, 13.264f, 13.264f, 13.264f, 13.264f,
        15.280f, 15.280f, 15.280f, 15.280f, 17.204f, 17.204f, 17.204f, 17.204f,    
        19.124f, 19.124f, 19.124f, 19.124f, 21.144f, 21.144f, 21.144f, 21.144f,
        23.152f, 23.152f, 23.152f, 23.152f, 25.060f, 25.060f, 25.060f, 25.060f, 
        26.972f, 26.972f, 26.972f, 26.972f, 28.976f, 28.976f, 28.976f, 28.976f, 
        30.992f, 30.992f, 30.992f, 30.992f, 32.916f, 32.916f, 32.916f, 32.916f, 
        34.836f, 34.836f, 34.836f, 34.836f, 36.856f, 36.856f, 36.856f, 36.856f, 
        38.864f, 38.864f, 38.864f, 38.864f, 40.272f, 40.272f, 40.272f, 40.272f, 
        41.684f, 41.684f, 41.684f, 41.684f, 43.188f, 43.188f, 43.188f, 43.188f, 
        44.704f, 44.704f, 44.704f, 44.704f, 46.128f, 46.128f, 46.128f, 46.128f, 
        47.548f, 47.548f, 47.548f, 47.548f, 49.068f, 49.068f, 49.068f, 49.068f
      };
      static const float blk_ts = 50.984f;

      float azi_channel_ori = cur_azi + azi_diff * tss[channel_idx] / blk_ts;

      int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx);
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) *
        this->lidar_const_param_.DIS_RESOLUTION;
      //int angle_horiz = static_cast<int>(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_horiz_x = static_cast<int>(azi_channel_ori - this->lidar_alph0_ + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert = ((this->vert_angle_list_[channel_idx]) + RS_ONE_ROUND) % RS_ONE_ROUND;

      T_Point point;
      if ((distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
          ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
           (!this->angle_flag_ &&
            ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))
      {
        float x = distance * this->checkCosTable(angle_vert) * this->checkCosTable(azi_channel_final) +
          this->lidar_Rxy_ * this->checkCosTable(angle_horiz_x);
        float y = -distance * this->checkCosTable(angle_vert) * this->checkSinTable(azi_channel_final) -
          this->lidar_Rxy_ * this->checkSinTable(angle_horiz_x);
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
