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

#include <rs_driver/driver/decoder/set_member.hpp>
#include <rs_driver/driver/decoder/decoder_base_opt.hpp>
#include <rs_driver/driver/driver_param.h>
#include <rs_driver/utility/time.h>

#include <arpa/inet.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <functional>
#include <chrono>
#include <memory>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES // for VC++, required to use const M_IP in <math.h>
#endif

/*Eigen*/
#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

namespace robosense
{
namespace lidar
{

/*Packet Length*/
const size_t MECH_PKT_LEN = 1248;
const size_t MEMS_MSOP_LEN = 1210;
const size_t MEMS_DIFOP_LEN = 256;
const size_t ROCK_MSOP_LEN = 1236;

constexpr float RS_ANGLE_RESOLUTION = 0.01;
constexpr float MICRO = 1000000.0;
constexpr float NANO = 1000000000.0;
constexpr int RS_ONE_ROUND = 36000;
constexpr uint16_t PROTOCOL_VER_0 = 0x00;

/* Echo mode definition */
enum RSEchoMode
{
  ECHO_SINGLE,
  ECHO_DUAL
};

/* Decode result definition */
enum RSDecoderResult
{
  DECODE_OK = 0,
  FRAME_SPLIT = 1,
  WRONG_PKT_HEADER = -1,
  WRONG_PKT_LENGTH = -2,
  PKT_NULL = -3,
  DISCARD_PKT = -4
};

template <typename T_PointCloud>
class DecoderBase
{
public:

  virtual RSDecoderResult processDifopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual RSDecoderResult processMsopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual RSDecoderResult TsMsopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual uint64_t usecToDelay() = 0;
  virtual ~DecoderBase() = default;

  explicit DecoderBase(const RSDecoderParam& param, 
      const LidarConstantParameter& lidar_const_param);

  void toSplit(uint16_t azimuth, double chan_ts);

  void regRecvCallback(const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put, 
      const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get)
  {
    point_cloud_cb_put_ = cb_put;
    point_cloud_cb_get_ = cb_get;
  }

  std::shared_ptr<T_PointCloud> getPointCloud()
  {
    std::shared_ptr<T_PointCloud> pc;

    while (1)
    {
      if (point_cloud_cb_get_ != nullptr)
        pc = point_cloud_cb_get_();

      if (pc)
      {
        pc->points.resize(0);
        return pc;
      }

#if 0
      reportError(Error(ERRCODE_POINTCLOUDNULL));
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
#endif
    };
  }

  uint16_t height()
  {
    return height_;
  }

  double getLidarTemperature()
  {
    return current_temperature_;
  }

  void loadAngleFile(const std::string& angle_path)
  {
  }

protected:

#if 0
  RSEchoMode getEchoMode(const LidarType& type, const uint8_t& return_mode);
#endif

  template <typename T_Difop>
  void decodeDifopCommon(const T_Difop& pkt);

protected:

  const LidarConstantParameter lidar_const_param_;
  RSDecoderParam param_;
  uint16_t height_;
  uint32_t msop_pkt_len_;
  uint32_t difop_pkt_len_;

  Trigon trigon_;
  DistanceBlock distance_block_;
  ChanAngles chan_angles_;
  ScanBlock scan_block_;
  SplitAngle split_angle_;
  float block_duration_;
  uint16_t azi_diff_between_block_theoretical_;
  float fov_time_jump_diff_;
  uint32_t pkts_per_frame_;

  unsigned int protocol_ver_;
  unsigned int rpm_;
  RSEchoMode echo_mode_;
  double current_temperature_;

  bool difop_flag_;
  int last_azimuth_;
  unsigned int pkt_count_;

  int lidar_alph0_;  // atan2(Ry, Rx) * 180 / M_PI * 100
  float lidar_Rxy_;  // sqrt(Rx*Rx + Ry*Ry)

  std::function<std::shared_ptr<T_PointCloud>(void)> point_cloud_cb_get_;
  std::function<void(std::shared_ptr<T_PointCloud>)> point_cloud_cb_put_;
  std::shared_ptr<T_PointCloud> point_cloud_;
  uint32_t point_cloud_seq_;
};

template <typename T_PointCloud>
inline DecoderBase<T_PointCloud>::DecoderBase(const RSDecoderParam& param,
                                              const LidarConstantParameter& lidar_const_param)
  : lidar_const_param_(lidar_const_param)
  , param_(param)
  , height_(0)
  , msop_pkt_len_(MECH_PKT_LEN)
  , difop_pkt_len_(MECH_PKT_LEN)
  , distance_block_(0.4f, 200.0f, param.min_distance, param.max_distance)
  , scan_block_(param.start_angle * 100, param.end_angle * 100)
  , split_angle_(param.cut_angle * 100)
  , block_duration_(0)
  , azi_diff_between_block_theoretical_(20)
  , fov_time_jump_diff_(0)
  , pkts_per_frame_(0)
  , protocol_ver_(0)
  , rpm_(600)
  , echo_mode_(ECHO_SINGLE)
  , current_temperature_(0)
  , difop_flag_(false)
  , last_azimuth_(-36001)
  , pkt_count_(0)
  , point_cloud_seq_(0)
{
  /*  Calulate the lidar_alph0 and lidar_Rxy */
  lidar_alph0_ = std::atan2(lidar_const_param_.RY, lidar_const_param_.RX) * 180 / M_PI * 100;
  lidar_Rxy_ = std::sqrt(lidar_const_param_.RX * lidar_const_param_.RX + lidar_const_param_.RY * lidar_const_param_.RY);
}

template <typename T_PointCloud>
inline void DecoderBase<T_PointCloud>::toSplit(uint16_t azimuth, double chan_ts)
{
  bool split = false;

  split = split_angle_.toSplit(azimuth);

#if 1
#if 0
  if (azimuth < this->last_azimuth_)
  {
    this->last_azimuth_ -= RS_ONE_ROUND;
  }

  if ((this->last_azimuth_ != -36001) && 
      (this->last_azimuth_ < this->cut_angle_) && (azimuth >= this->cut_angle_))
  {
    this->last_azimuth_ = azimuth;
    this->pkt_count_ = 0;
    return FRAME_SPLIT;
  }

  this->last_azimuth_ = azimuth;
#endif

#else
  if (this->pkt_count_ >= this->pkts_per_frame_)
  {
    this->pkt_count_ = 0;
    split = true;
  }

  if (this->pkt_count_ >= this->param_.num_pkts_split)
  {
    this->pkt_count_ = 0;
    split = true;
  }
#endif

  if (split)
  {
    T_PointCloud& msg = *point_cloud_;

    msg.seq = point_cloud_seq_++;
    msg.timestamp = chan_ts;
    //msg.frame_id = param_.frame_id;
    msg.is_dense = param_.is_dense;
    if (msg.is_dense)
    {
      msg.height = 1;
      msg.width = msg.points.size();
    }
    else
    {
      msg.height = height_;
      msg.width = msg.points.size() / msg.height;
    }

    if (msg.points.size() == 0)
    {
#if 0
      reportError(Error(ERRCODE_ZEROPOINTS));
#endif
    }
    else
    {
      point_cloud_cb_put_(point_cloud_);
    }
  }
}

template <typename T_PointCloud>
template <typename T_Difop>
inline void DecoderBase<T_PointCloud>::decodeDifopCommon(const T_Difop& pkt)
{
  // return mode
  switch (pkt.return_mode)
  {
    case 0x00:
      this->echo_mode_ = RSEchoMode::ECHO_DUAL;
    default:
      this->echo_mode_ = RSEchoMode::ECHO_SINGLE;
  }

  pkts_per_frame_ = 
    this->lidar_const_param_.BLOCKS_PER_FRAME / this->lidar_const_param_.BLOCKS_PER_PKT;
  if (this->echo_mode_ == RSEchoMode::ECHO_DUAL)
  {
    pkts_per_frame_ *= 2;
  }

  // rpm
  this->rpm_ = ntohs(pkt.rpm);
  if (this->rpm_ == 0)
  {
    RS_WARNING << "LiDAR RPM is 0" << RS_REND;
    this->rpm_ = 600;
  }

  // block duration - azimuth
  this->azi_diff_between_block_theoretical_ = 
    RS_ONE_ROUND / this->lidar_const_param_.BLOCKS_PER_FRAME;

  // block duration - timestamp
  this->block_duration_ = 
    60 / (this->rpm_ * this->lidar_const_param_.BLOCKS_PER_FRAME);

  // blind block duration
  int fov_start_angle = ntohs(pkt.fov.start_angle);
  int fov_end_angle = ntohs(pkt.fov.end_angle);

  int fov_range = (fov_start_angle < fov_end_angle) ? (fov_end_angle - fov_start_angle) :
    (RS_ONE_ROUND - fov_start_angle + fov_end_angle);

  this->fov_time_jump_diff_ = this->block_duration_ * 
    (fov_range / (RS_ONE_ROUND / this->lidar_const_param_.BLOCKS_PER_FRAME));
}

#if 0
template <typename T_PointCloud>
inline RSEchoMode DecoderBase<T_PointCloud>::getEchoMode(const LidarType& type, const uint8_t& return_mode)
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
