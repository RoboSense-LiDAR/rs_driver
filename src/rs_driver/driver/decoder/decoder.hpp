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

#include <rs_driver/common/error_code.h>
#include <rs_driver/driver/driver_param.h>
#include <rs_driver/driver/decoder/member_checker.hpp>
#include <rs_driver/driver/decoder/trigon.hpp>
#include <rs_driver/driver/decoder/section.hpp>
#include <rs_driver/driver/decoder/chan_angles.hpp>
#include <rs_driver/driver/decoder/member_checker.hpp>
#include <rs_driver/driver/decoder/basic_attr.hpp>
#include <rs_driver/driver/decoder/split_strategy.hpp>
#include <rs_driver/driver/decoder/block_diff.hpp>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES // for VC++, required to use const M_IP in <math.h>
#endif
#include <cmath>
#include <arpa/inet.h>
#include <functional>
#include <memory>

namespace robosense
{
namespace lidar
{

#pragma pack(push, 1)
typedef struct
{
  uint8_t id[8];
  uint8_t reserved_1[12];
  RSTimestampYMD timestamp;
  uint8_t lidar_type;
  uint8_t reserved_2[7];
  RSTemperature temp;
  uint8_t reserved_3[2];
} RSMsopHeaderV1;

typedef struct
{
  uint8_t id[4];
  uint16_t protocol_version;
  uint8_t reserved_1;
  uint8_t wave_mode;
  RSTemperature temp;
  RSTimestampUTC timestamp;
  uint8_t reserved_2[10];
  uint8_t lidar_type;
  uint8_t reserved_3[49];
} RSMsopHeaderV2;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} RSChannel;

typedef struct
{
  uint8_t lidar_ip[4];
  uint8_t host_ip[4];
  uint8_t mac_addr[6];
  uint16_t local_port;
  uint16_t dest_port;
  uint16_t port3;
  uint16_t port4;
} RSEthNetV1;

typedef struct
{
  uint8_t lidar_ip[4];
  uint8_t dest_ip[4];
  uint8_t mac_addr[6];
  uint16_t msop_port;
  uint16_t reserve_1;
  uint16_t difop_port;
  uint16_t reserve_2;
} RSEthNetV2;

typedef struct
{
  uint16_t start_angle;
  uint16_t end_angle;
} RSFOV;

typedef struct
{
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];
} RSVersionV1;

typedef struct
{
  uint8_t top_firmware_ver[5];
  uint8_t bot_firmware_ver[5];
  uint8_t bot_soft_ver[5];
  uint8_t motor_firmware_ver[5];
  uint8_t hw_ver[3];
} RSVersionV2;

typedef struct
{
  uint8_t num[6];
} RSSN;

typedef struct
{
  uint8_t device_current[3];
  uint8_t main_current[3];
  uint16_t vol_12v;
  uint16_t vol_sim_1v8;
  uint16_t vol_dig_3v3;
  uint16_t vol_sim_3v3;
  uint16_t vol_dig_5v4;
  uint16_t vol_sim_5v;
  uint16_t vol_ejc_5v;
  uint16_t vol_recv_5v;
  uint16_t vol_apd;
} RSStatusV1;

typedef struct
{
  uint16_t device_current;
  uint16_t vol_fpga;
  uint16_t vol_12v;
  uint16_t vol_dig_5v4;
  uint16_t vol_sim_5v;
  uint16_t vol_apd;
  uint8_t reserved[12];
} RSStatusV2;

typedef struct
{
  uint8_t reserved_1[9];
  uint16_t checksum;
  uint16_t manc_err1;
  uint16_t manc_err2;
  uint8_t gps_status;
  uint16_t temperature1;
  uint16_t temperature2;
  uint16_t temperature3;
  uint16_t temperature4;
  uint16_t temperature5;
  uint8_t reserved_2[5];
  uint16_t cur_rpm;
  uint8_t reserved_3[7];
} RSDiagnoV1;

typedef struct
{
  uint16_t bot_fpga_temperature;
  uint16_t recv_A_temperature;
  uint16_t recv_B_temperature;
  uint16_t main_fpga_temperature;
  uint16_t main_fpga_core_temperature;
  uint16_t real_rpm;
  uint8_t lane_up;
  uint16_t lane_up_cnt;
  uint16_t main_status;
  uint8_t gps_status;
  uint8_t reserved[22];
} RSDiagnoV2;

typedef struct
{
  uint8_t sync_mode;
  uint8_t sync_sts;
  RSTimestampUTC timestamp;
} RSTimeInfo;

#pragma pack(pop)

// Echo mode
enum RSEchoMode
{
  ECHO_SINGLE = 0,
  ECHO_DUAL
};

// decoder const param
struct RSDecoderConstParam
{
  // packet len
  uint16_t MSOP_LEN;
  uint16_t DIFOP_LEN;

  // packet identity
  uint8_t MSOP_ID_LEN;
  uint8_t DIFOP_ID_LEN;
  uint8_t MSOP_ID[8];
  uint8_t DIFOP_ID[8];
  uint8_t BLOCK_ID[2];

  // packet structure
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;

  // distance & temperature
  float DISTANCE_MIN;
  float DISTANCE_MAX;
  float DISTANCE_RES;
  float TEMPERATURE_RES;
};

template <typename T_PointCloud>
class Decoder
{
public:

  constexpr static uint16_t PROTOCOL_VER_0 = 0x00;

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual void decodeMsopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual ~Decoder() = default;

  void processDifopPkt(const uint8_t* pkt, size_t size);
  void processMsopPkt(const uint8_t* pkt, size_t size);

  explicit Decoder(const RSDecoderConstParam& const_param, 
      const RSDecoderParam& param, 
      const std::function<void(const Error&)>& excb);

  void regRecvCallback(const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put);

  float getTemperature();
  double getPacketDuration();

#ifndef UNIT_TEST
protected:
#endif

  void splitFrame();
  void setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, double chan_ts);

  RSDecoderConstParam const_param_; // const param
  RSDecoderParam param_; // user param
  std::function<void(const Error&)> excb_;
  std::function<std::shared_ptr<T_PointCloud>(void)> point_cloud_cb_get_;
  std::function<void(std::shared_ptr<T_PointCloud>)> point_cloud_cb_put_;

  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

  uint16_t height_; 
  double packet_duration_;
  DistanceSection distance_section_; // invalid section of distance

  RSEchoMode echo_mode_; // echo mode (defined by return mode)
  float temperature_; // lidar temperature

  bool angles_ready_; // is vert_angles/horiz_angles ready from csv file/difop packet?
  double prev_point_ts_; // last point's timestamp

  std::shared_ptr<T_PointCloud> point_cloud_; // curernt point cloud
  uint32_t point_cloud_seq_; // sequence of point cloud
};

template <typename T_PointCloud>
inline Decoder<T_PointCloud>::Decoder(const RSDecoderConstParam& const_param, 
    const RSDecoderParam& param, 
    const std::function<void(const Error&)>& excb)
  : const_param_(const_param)
  , param_(param)
  , excb_(excb)
  , height_(0)
  , packet_duration_(0)
  , distance_section_(const_param.DISTANCE_MIN, const_param.DISTANCE_MAX, 
      param.min_distance, param.max_distance)
  , echo_mode_(ECHO_SINGLE)
  , temperature_(0.0)
  , angles_ready_(false)
  , prev_point_ts_(0.0)
  , point_cloud_seq_(0)
{
}

template <typename T_PointCloud>
float Decoder<T_PointCloud>::getTemperature()
{
  return temperature_;
}

template <typename T_PointCloud>
double Decoder<T_PointCloud>::getPacketDuration()
{
  return packet_duration_;
}

template <typename T_PointCloud>
void Decoder<T_PointCloud>::regRecvCallback(
    const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get,
    const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put) 
{
  point_cloud_cb_get_ = cb_get;
  point_cloud_cb_put_ = cb_put;

  point_cloud_ = point_cloud_cb_get_();
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::splitFrame()
{
  if (point_cloud_->points.size() > 0)
  {
    setPointCloudHeader(point_cloud_, prev_point_ts_);
    point_cloud_cb_put_(point_cloud_);
    point_cloud_ = point_cloud_cb_get_();
  }
  else
  {
    excb_(Error(ERRCODE_ZEROPOINTS));
  }
}

template <typename T_PointCloud>
void Decoder<T_PointCloud>::setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, double chan_ts)
{
  msg->seq = point_cloud_seq_++;
  msg->timestamp = chan_ts;
  msg->is_dense = param_.dense_points;
  if (msg->is_dense)
  {
    msg->height = 1;
    msg->width = msg->points.size();
  }
  else
  {
    msg->height = height_;
    msg->width = msg->points.size() / msg->height;
  }
}

template <typename T_PointCloud>
void Decoder<T_PointCloud>::processDifopPkt(const uint8_t* pkt, size_t size)
{
  if (size != this->const_param_.DIFOP_LEN)
  {
     this->excb_(Error(ERRCODE_WRONGPKTLENGTH));
     return;
  }

  if (memcmp(pkt, this->const_param_.DIFOP_ID, const_param_.DIFOP_ID_LEN) != 0)
  {
    this->excb_(Error(ERRCODE_WRONGPKTHEADER));
    return;
  }

  decodeDifopPkt(pkt, size);
}

template <typename T_PointCloud>
void Decoder<T_PointCloud>::processMsopPkt(const uint8_t* pkt, size_t size)
{
  if (param_.wait_for_difop && !angles_ready_)
  {
     excb_(Error(ERRCODE_NODIFOPRECV));
     return;
  }

  if (size != this->const_param_.MSOP_LEN)
  {
     this->excb_(Error(ERRCODE_WRONGPKTLENGTH));
     return;
  }

  if (memcmp(pkt, this->const_param_.MSOP_ID, const_param_.MSOP_ID_LEN) != 0)
  {
    this->excb_(Error(ERRCODE_WRONGPKTHEADER));
    return;
  }

  decodeMsopPkt(pkt, size);
}

}  // namespace lidar
}  // namespace robosense
