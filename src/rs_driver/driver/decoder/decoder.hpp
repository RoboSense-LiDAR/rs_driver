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

//
// msop & difop
// 
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
typedef struct
{
  uint16_t MSOP_LEN;
  uint16_t DIFOP_LEN;

  // identity
  uint8_t MSOP_ID_LEN;
  uint8_t DIFOP_ID_LEN;
  uint8_t MSOP_ID[8];
  uint8_t DIFOP_ID[8];
  uint8_t BLOCK_ID[2];

  // duration
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;
  //uint16_t LASER_NUM; // diff from CHANNELS_PER_BLOCK ?

  // distance resolution
  float DISTANCE_MIN;
  float DISTANCE_MAX;
  float DISTANCE_RES;
  float TEMPERATURE_RES;

  // lens center
  float RX;
  float RY;
  float RZ;

  // firing_ts / block_ts, chan_ts
  double BLOCK_DURATION;
  double CHAN_TSS[128];
  float CHAN_AZIS[128];

} RSDecoderConstParam;

#if 0
const size_t MECH_PKT_LEN = 1248;
const size_t MEMS_MSOP_LEN = 1210;
const size_t MEMS_DIFOP_LEN = 256;
const size_t ROCK_MSOP_LEN = 1236;
#endif

template <typename T_PointCloud>
class Decoder
{
public:

  constexpr static int32_t RS_ONE_ROUND = 36000;
  constexpr static uint16_t PROTOCOL_VER_0 = 0x00;

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual void decodeMsopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual ~Decoder() = default;

  explicit Decoder(const RSDecoderParam& param, 
      const std::function<void(const Error&)>& excb,
      const RSDecoderConstParam& const_param, bool narrow_angles = false);

  void processDifopPkt(const uint8_t* pkt, size_t size);
  void processMsopPkt(const uint8_t* pkt, size_t size);

  void regRecvCallback(const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put);

  float getTemperature();
  double getPacketDuration();
  void print();

#ifndef UNIT_TEST
protected:
#endif

  template <typename T_Difop>
  void decodeDifopCommon(const T_Difop& pkt);

  void newBlock(int32_t azimuth);
  void setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, double chan_ts);

  RSDecoderConstParam const_param_; // const param of lidar/decoder
  RSDecoderParam param_; // user param of lidar/decoder
  std::function<void(const Error&)> excb_;
  uint16_t height_; 

  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

  ChanAngles chan_angles_; // vert_angles/horiz_angles adjustment
  DistanceSection distance_section_; // valid distance section
  AzimuthSection scan_section_; // valid azimuth section
  std::shared_ptr<SplitStrategy> split_strategy_; // split strategy

  uint16_t blks_per_frame_; // blocks per frame/round
  uint16_t split_blks_per_frame_; // blocks in msop pkt per frame/round. 
                                  // dependent on return mode.
  uint16_t block_azi_diff_; // azimuth difference between adjacent blocks.
  float fov_blind_ts_diff_; // timestamp difference across blind section(defined by fov)

  uint16_t rps_; // rounds per second
  RSEchoMode echo_mode_; // echo mode (defined by return mode)
  float temperature_; // lidar temperature

  bool angles_ready_; // is vert_angles/horiz_angles ready from csv file/difop packet?
  double prev_chan_ts_; // previous channel/point timestamp

  int lidar_alph0_;  // lens center related
  float lidar_Rxy_;  // lens center related

  std::function<std::shared_ptr<T_PointCloud>(void)> point_cloud_cb_get_;
  std::function<void(std::shared_ptr<T_PointCloud>)> point_cloud_cb_put_;
  std::shared_ptr<T_PointCloud> point_cloud_; // curernt point cloud
  uint32_t point_cloud_seq_; // sequence of point cloud
};

template <typename T_PointCloud>
inline Decoder<T_PointCloud>::Decoder(const RSDecoderParam& param, 
    const std::function<void(const Error&)>& excb,
    const RSDecoderConstParam& const_param, bool narrow_angles)
  : const_param_(const_param)
  , param_(param)
  , excb_(excb)
  , height_(const_param.CHANNELS_PER_BLOCK)
  , chan_angles_(const_param.CHANNELS_PER_BLOCK, narrow_angles)
  , distance_section_(const_param.DISTANCE_MIN, const_param.DISTANCE_MAX, 
      param.min_distance, param.max_distance)
  , scan_section_(param.start_angle * 100, param.end_angle * 100)
  , blks_per_frame_(1/(10*const_param.BLOCK_DURATION))
  , split_blks_per_frame_(blks_per_frame_)
  , block_azi_diff_(20)
  , fov_blind_ts_diff_(0)
  , rps_(10)
  , echo_mode_(ECHO_SINGLE)
  , temperature_(0.0)
  , angles_ready_(false)
  , prev_chan_ts_(0.0)
  , point_cloud_seq_(0)
{
  switch (param_.split_frame_mode)
  {
    case SplitFrameMode::SPLIT_BY_FIXED_BLKS:
      split_strategy_ = std::make_shared<SplitStrategyByNum>(&split_blks_per_frame_);
      break;

    case SplitFrameMode::SPLIT_BY_CUSTOM_BLKS:
      split_strategy_ = std::make_shared<SplitStrategyByNum>(&param_.num_blks_split);
      break;

    case SplitFrameMode::SPLIT_BY_ANGLE:
    default:
      uint16_t angle = (uint16_t)(param_.split_angle * 100);
      split_strategy_ = std::make_shared<SplitStrategyByAngle>(angle);
      break;
  }

  // calulate lidar_alph0 and lidar_Rxy
  lidar_alph0_ = std::atan2(const_param_.RY, const_param_.RX) * 180 / M_PI * 100;
  lidar_Rxy_ = std::sqrt(const_param_.RX * const_param_.RX + const_param_.RY * const_param_.RY);

  if (param.config_from_file)
  {
    if (param_.wait_for_difop)
    {
      RS_WARNING << "When config_from_file is true, wait_for_difop cannot be true."
                 << " Reset it to be false." << RS_REND;
    }

    int ret = chan_angles_.loadFromFile(param.angle_path);
    this->angles_ready_ = (ret == 0);
  }
}

template <typename T_PointCloud>
void Decoder<T_PointCloud>::print()
{
  std::cout << "-----------------------------------------" << std::endl
    << "rps:\t\t\t" << this->rps_ << std::endl
    << "echo_mode:\t\t" << this->echo_mode_ << std::endl
    << "blks_per_frame:\t\t" << this->blks_per_frame_ << std::endl
    << "split_blks_per_frame:\t" << this->split_blks_per_frame_ << std::endl
    << "block_azi_diff:\t\t" << this->block_azi_diff_ << std::endl
    << "fov_blind_ts_diff:\t" << this->fov_blind_ts_diff_ << std::endl
    << "angle_from_file:\t" << this->param_.config_from_file << std::endl
    << "angles_ready:\t\t" << this->angles_ready_ << std::endl;

  this->chan_angles_.print();
}

template <typename T_PointCloud>
float Decoder<T_PointCloud>::getTemperature()
{
  return temperature_;
}

template <typename T_PointCloud>
double Decoder<T_PointCloud>::getPacketDuration()
{
  return this->const_param_.BLOCK_DURATION * const_param_.BLOCKS_PER_PKT;
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
inline void Decoder<T_PointCloud>::newBlock(int32_t azimuth)
{
  bool split = this->split_strategy_->newBlock(azimuth);
  if (split)
  {
    if (point_cloud_->points.size() > 0)
    {
      setPointCloudHeader(point_cloud_, prev_chan_ts_);
      point_cloud_cb_put_(point_cloud_);
      point_cloud_ = point_cloud_cb_get_();
    }
    else
    {
      excb_(Error(ERRCODE_ZEROPOINTS));
    }
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
template <typename T_Difop>
inline void Decoder<T_PointCloud>::decodeDifopCommon(const T_Difop& pkt)
{
  // rounds per second
  this->rps_ = ntohs(pkt.rpm) / 60;
  if (this->rps_ == 0)
  {
    RS_WARNING << "LiDAR RPM is 0. Use default value 600." << RS_REND;
    this->rps_ = 10;
  }

  // blocks per frame
  this->blks_per_frame_ = 1 / (this->rps_ * this->const_param_.BLOCK_DURATION);

  // block diff of azimuth
  this->block_azi_diff_ = 
    std::round(RS_ONE_ROUND * this->rps_ * this->const_param_.BLOCK_DURATION);

  // fov related
  uint16_t fov_start_angle = ntohs(pkt.fov.start_angle);
  uint16_t fov_end_angle = ntohs(pkt.fov.end_angle);
  uint16_t fov_range = (fov_start_angle < fov_end_angle) ? 
    (fov_end_angle - fov_start_angle) : (fov_end_angle + RS_ONE_ROUND - fov_start_angle);
  uint16_t fov_blind_range = RS_ONE_ROUND - fov_range;

  // fov blind diff of timestamp
  this->fov_blind_ts_diff_ = 
    (float)fov_blind_range / ((float)RS_ONE_ROUND * (float)this->rps_);

  if (!this->param_.config_from_file && !this->angles_ready_)
  {
    int ret = this->chan_angles_.loadFromDifop(pkt.vert_angle_cali, pkt.horiz_angle_cali, 
        this->const_param_.CHANNELS_PER_BLOCK);
    this->angles_ready_ = (ret == 0);
  }
}

#if 0
template <typename T_PointCloud>
inline RSEchoMode Decoder<T_PointCloud>::getEchoMode(const LidarType& type, const uint8_t& return_mode)
{
  switch (type)
  {
    case LidarType::RS128:
    case LidarType::RS80:
    case LidarType::RSHELIOS:
      switch (return_mode)
      {
        case 0x00:
        case 0x03:
          return RSEchoMode::ECHO_DUAL;
        default:
          return RSEchoMode::ECHO_SINGLE;
      }
    case LidarType::RS16:
    case LidarType::RS32:
    case LidarType::RSBP:
      switch (return_mode)
      {
        case 0x00:
          return RSEchoMode::ECHO_DUAL;
        default:
          return RSEchoMode::ECHO_SINGLE;
      }
    case LidarType::RSM1:
      switch (return_mode)
      {
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03:
          return RSEchoMode::ECHO_DUAL;
        default:
          return RSEchoMode::ECHO_SINGLE;
      }
    case LidarType::RSROCK:  // TODO
      return RSEchoMode::ECHO_SINGLE;
  }
  return RSEchoMode::ECHO_SINGLE;
}
#endif

}  // namespace lidar
}  // namespace robosense
