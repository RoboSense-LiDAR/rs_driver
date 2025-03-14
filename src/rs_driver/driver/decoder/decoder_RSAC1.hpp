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

#include <rs_driver/driver/decoder/decoder.hpp>

#define POINT_WIDTH_NUMS            96
#define POINT_HEIGHT_NUMS           288
#define POINT_NUMS                  POINT_WIDTH_NUMS * POINT_HEIGHT_NUMS

namespace robosense
{
namespace lidar
{


template <typename T_PointCloud>
class DecoderRSAC1 : public Decoder<T_PointCloud>
{
public:
  constexpr static int VECTOR_BASE = 32768;

  void decodeDifopPkt(const uint8_t* pkt, size_t size){};
  bool decodeMsopPkt(const uint8_t* pkt, size_t size){return false;};
  
  void decodeImuPkt(const uint8_t* pkt, size_t size);
  void decodeImagePkt(const uint8_t* pkt, size_t size);
  void decodePcPkt(const uint8_t* pkt, size_t size);

  virtual ~DecoderRSAC1() = default;

  explicit DecoderRSAC1(const RSDecoderParam& param);

private:

  static RSDecoderConstParam& getConstParam();

};

template <typename T_PointCloud>
inline RSDecoderConstParam& DecoderRSAC1<T_PointCloud>::getConstParam()
{
  static RSDecoderConstParam param = 
  {
      0 // msop len
      , 0 // difop len
      , 0 // msop id len
      , 0 // difop id len
      , {} // msop id
      , {} // difop id
      , {}
      , 1  // laser number
      , 27648 // blocks per packet
      , 1 // channels per block
      , 0.2f // distance min
      , 200.0f // distance max
      , 0.005f // distance resolution
      , 80.0f // initial value of temperature 
  };

  return param;
}

template <typename T_PointCloud>
inline DecoderRSAC1<T_PointCloud>::DecoderRSAC1(const RSDecoderParam& param)
  : Decoder<T_PointCloud>(getConstParam(), param)
{

}

template <typename T_PointCloud>
inline void DecoderRSAC1<T_PointCloud>::decodeImagePkt(const uint8_t* packet, size_t size)
{
    if(this->imageDataPtr_ && this->cb_image_data_)
    {
      this->imageDataPtr_->data_bytes = size - 20;
      this->imageDataPtr_->data = std::shared_ptr<uint8_t>(new uint8_t[this->imageDataPtr_->data_bytes], std::default_delete<uint8_t[]>());
      memcpy(this->imageDataPtr_->data.get(), packet + 20, this->imageDataPtr_->data_bytes);

      memcpy(&this->imageDataPtr_->frame_format, packet + 2, 2);
      memcpy(&this->imageDataPtr_->timestamp, packet + 4, 8);
      memcpy(&this->imageDataPtr_->width, packet + 12, 4);
      memcpy(&this->imageDataPtr_->height,  packet + 16, 4);

      this->imageDataPtr_->state = true;
      this->cb_image_data_();
    }
}

template <typename T_PointCloud>
inline void DecoderRSAC1<T_PointCloud>::decodeImuPkt(const uint8_t* packet, size_t size)
{
    if(this->imuDataPtr_ && this->cb_imu_data_)
    {
      auto data = packet;
      memcpy(&this->imuDataPtr_->linear_acceleration_x, data + 10, sizeof(float));
      memcpy(&this->imuDataPtr_->linear_acceleration_y, data + 14, sizeof(float));
      memcpy(&this->imuDataPtr_->linear_acceleration_z, data + 18, sizeof(float));
      memcpy(&this->imuDataPtr_->angular_velocity_x, data + 22, sizeof(float));
      memcpy(&this->imuDataPtr_->angular_velocity_y, data + 26, sizeof(float));
      memcpy(&this->imuDataPtr_->angular_velocity_z, data + 30, sizeof(float));
      struct timespec time;
      memcpy(&time, data + 38, sizeof(struct timespec));
      this->imuDataPtr_->timestamp = time.tv_sec + time.tv_nsec * 1e-9;
      
      this->imuDataPtr_->state = true;
      this->cb_imu_data_();
    }
}

template <typename T_PointCloud>
inline void DecoderRSAC1<T_PointCloud>::decodePcPkt(const uint8_t* packet, size_t size)
{
    if(this->point_cloud_ && this->cb_split_frame_)
    {
      this->point_cloud_->points.resize(POINT_NUMS);
      auto data = packet + 20;

      int p_num = 0;
      float dist = 0;
      int width = POINT_WIDTH_NUMS * 12 + 10;
      int height = POINT_HEIGHT_NUMS;
      struct timeval time_tmp;
      for (int j = 0; j < height; ++j)
      {
          time_tmp.tv_sec = (uint64_t)data[width * j + 0] << 40 | (uint64_t)data[width * j + 1] << 32 |
                            (uint32_t)data[width * j + 2] << 24 | (uint32_t)data[width * j + 3] << 16 |
                            (uint16_t)data[width * j + 4] << 8 | data[width * j + 5];
          time_tmp.tv_usec = (uint32_t)data[width * j + 6] << 24 | (uint32_t)data[width * j + 7] << 16 |
                              (uint16_t)data[width * j + 8] << 8 | data[width * j + 9];

          for (int i = 10; i < width; i += 12)
          {
              uint16_t time_offset = (uint16_t)(data[width * j + i]) << 8 | data[width * j + i + 1];
              auto timestamp = (double)(time_offset * 1e-6) + double(time_tmp.tv_usec * 1e-6) + time_tmp.tv_sec;
              dist = ((uint16_t)data[width * j + i + 2] << 8 | data[width * j + i + 3]) * 0.005;
              if (this->distance_section_.in(dist))
              {
                auto x = dist * (int16_t)((uint16_t)(data[width * j + i + 4]) << 8 | data[width * j + i + 5]) / VECTOR_BASE;
                auto y = dist * (int16_t)((uint16_t)(data[width * j + i + 6]) << 8 | data[width * j + i + 7]) / VECTOR_BASE;
                auto z = dist * (int16_t)((uint16_t)(data[width * j + i + 8]) << 8 | data[width * j + i + 9]) / VECTOR_BASE;
                auto intensity = (uint16_t)data[width * j + i + 10];
                
                setX(this->point_cloud_->points[p_num], x);
                setY(this->point_cloud_->points[p_num], y);
                setZ(this->point_cloud_->points[p_num], z);
                setIntensity(this->point_cloud_->points[p_num], intensity);
                setTimestamp(this->point_cloud_->points[p_num], timestamp);
                setRing(this->point_cloud_->points[p_num], 0);
              }
              else if(!this->param_.dense_points)
              {
                setX(this->point_cloud_->points[p_num], NAN);
                setY(this->point_cloud_->points[p_num], NAN);
                setZ(this->point_cloud_->points[p_num], NAN);
                setIntensity(this->point_cloud_->points[p_num], 0);
                setTimestamp(this->point_cloud_->points[p_num], timestamp);
                setRing(this->point_cloud_->points[p_num], 0);
              }
              if(p_num == 0)
              {
                this->first_point_ts_ = timestamp;
              }
              if(p_num == POINT_NUMS - 1)
              {
                this->prev_point_ts_ = timestamp;
              }
              p_num++;
          }
      }

      this->cb_split_frame_(POINT_NUMS, this->cloudTs());
    }
}


}  // namespace lidar
}  // namespace robosense
