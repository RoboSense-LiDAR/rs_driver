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

#define RS_TO_RADS(x) ((x) * (M_PI) / 180)

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

  virtual RSDecoderResult processMsopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual RSDecoderResult processDifopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual ~DecoderBase() = default;

  uint16_t height()
  {
    return height_;
  }

  void regRecvCallback(const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put, 
      const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get)
  {
    point_cloud_cb_put_vec_.emplace_back(cb_put);
    if (cb_get != nullptr) 
    {
      point_cloud_cb_get_ = cb_get;
    }
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

  double getLidarTemperature()
  {
    return current_temperature_;
  }

  explicit DecoderBase(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);

  void loadAngleFile(const std::string& angle_path);

protected:

#if 0
  RSEchoMode getEchoMode(const LidarType& type, const uint8_t& return_mode);
#endif

  template <typename T_Difop>
  void decodeDifopCommon(const T_Difop& pkt);

#if 0
  void transformPoint(float& x, float& y, float& z);
#endif

  float checkCosTable(const int& angle);
  float checkSinTable(const int& angle);

private:
  DecoderBase(const DecoderBase&) = delete;
  DecoderBase& operator=(const DecoderBase&) = delete;

protected:

  std::function<std::shared_ptr<T_PointCloud>(void)> point_cloud_cb_get_;
  std::vector<std::function<void(std::shared_ptr<T_PointCloud>)>> point_cloud_cb_put_vec_;
  std::shared_ptr<T_PointCloud> point_cloud_;

  const LidarConstantParameter lidar_const_param_;
  RSDecoderParam param_;
  double current_temperature_;
  uint16_t height_;

  uint32_t msop_pkt_len_;
  uint32_t difop_pkt_len_;

  RSEchoMode echo_mode_;

  //unsigned int pkts_per_frame_;
  unsigned int pkt_count_;
  unsigned int trigger_index_;
  unsigned int prev_angle_diff_;
  unsigned int rpm_;
  unsigned int protocol_ver_;

  int start_angle_;
  int end_angle_;
  bool angle_flag_;

  int cut_angle_;
  int last_azimuth_;
  bool difop_flag_;
  float fov_time_jump_diff_;
  float time_duration_between_blocks_;
  //float azi_diff_between_block_theoretical_;
  uint16_t azi_diff_between_block_theoretical_;

  std::vector<int> vert_angle_list_;
  std::vector<int> hori_angle_list_;
  std::vector<uint16_t> beam_ring_table_;

  int lidar_alph0_;  // atan2(Ry, Rx) * 180 / M_PI * 100
  float lidar_Rxy_;  // sqrt(Rx*Rx + Ry*Ry)

  ChanAngles chan_angles_;
  DistanceBlock distance_block_;
  ScanBlock scan_block_;
  Trigon trigon_;

private:
  std::vector<float> cos_lookup_table_;
  std::vector<float> sin_lookup_table_;
};

template <typename T_PointCloud>
inline DecoderBase<T_PointCloud>::DecoderBase(const RSDecoderParam& param,
                                              const LidarConstantParameter& lidar_const_param)
  : lidar_const_param_(lidar_const_param)
  , param_(param)
  , current_temperature_(0)
  , height_(0)
  , msop_pkt_len_(MECH_PKT_LEN)
  , difop_pkt_len_(MECH_PKT_LEN)
  , echo_mode_(ECHO_SINGLE)
  //, pkts_per_frame_(lidar_const_param.PKT_RATE / 10)
  , pkt_count_(0)
  , trigger_index_(0)
  , prev_angle_diff_(RS_ONE_ROUND)
  , rpm_(600)
  , protocol_ver_(0)
  , start_angle_(param.start_angle * 100)
  , end_angle_(param.end_angle * 100)
  , angle_flag_(true)
  , cut_angle_(param.cut_angle * 100)
  , last_azimuth_(-36001)
  , difop_flag_(false)
  , fov_time_jump_diff_(0)
  , time_duration_between_blocks_(0)
  , azi_diff_between_block_theoretical_(20)
  , distance_block_(0.4f, 200.0f, param.min_distance, param.max_distance)
  , scan_block_(param.start_angle * 100, param.end_angle)
{
  if (cut_angle_ > RS_ONE_ROUND)
  {
    cut_angle_ = 0;
  }

  if (this->start_angle_ > RS_ONE_ROUND || this->start_angle_ < 0 || 
      this->end_angle_ > RS_ONE_ROUND || this->end_angle_ < 0)
  {
    RS_WARNING << "start_angle & end_angle should be in range 0~360° " << RS_REND;
    this->start_angle_ = 0;
    this->end_angle_ = RS_ONE_ROUND;
  }

  if (this->start_angle_ > this->end_angle_)
  {
    this->angle_flag_ = false;
  }

  /*  Calulate the lidar_alph0 and lidar_Rxy */
  lidar_alph0_ = std::atan2(lidar_const_param_.RY, lidar_const_param_.RX) * 180 / M_PI * 100;
  lidar_Rxy_ = std::sqrt(lidar_const_param_.RX * lidar_const_param_.RX + lidar_const_param_.RY * lidar_const_param_.RY);
}

#if 0
template <typename T_PointCloud>
inline RSDecoderResult DecoderBase<T_PointCloud>::processMsopPkt(
    const uint8_t* pkt, size_t size)
{
  if (size != this->msop_pkt_len_)
  {
    return WRONG_PKT_LENGTH;
  }

  int azimuth = 0;
  RSDecoderResult ret = decodeMsopPkt(pkt, point_cloud_vec, height, azimuth);
  if (ret != RSDecoderResult::DECODE_OK)
  {
    return ret;
  }
  this->pkt_count_++;
  switch (this->param_.split_frame_mode)
  {
    case SplitFrameMode::SPLIT_BY_ANGLE:
      if (azimuth < this->last_azimuth_)
      {
        this->last_azimuth_ -= RS_ONE_ROUND;
      }
      if (this->last_azimuth_ != -36001 && this->last_azimuth_ < this->cut_angle_ && azimuth >= this->cut_angle_)
      {
        this->last_azimuth_ = azimuth;
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = RS_ONE_ROUND;
        return FRAME_SPLIT;
      }
      this->last_azimuth_ = azimuth;
      break;
    case SplitFrameMode::SPLIT_BY_FIXED_PKTS:
      if (this->pkt_count_ >= this->pkts_per_frame_)
      {
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = RS_ONE_ROUND;
        return FRAME_SPLIT;
      }
      break;
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
#endif

template <typename T_PointCloud>
inline void DecoderBase<T_PointCloud>::loadAngleFile(const std::string& angle_path)
{
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
  this->time_duration_between_blocks_ = 
    60 / (this->rpm_ * this->lidar_const_param_.BLOCKS_PER_FRAME);

  // blind block duration
  int fov_start_angle = ntohs(pkt.fov.start_angle);
  int fov_end_angle = ntohs(pkt.fov.end_angle);

  int fov_range = (fov_start_angle < fov_end_angle) ? (fov_end_angle - fov_start_angle) :
    (RS_ONE_ROUND - fov_start_angle + fov_end_angle);

  this->fov_time_jump_diff_ = this->time_duration_between_blocks_ * 
    (fov_range / (RS_ONE_ROUND / this->lidar_const_param_.BLOCKS_PER_FRAME));
}

#if 0
template <typename T_PointCloud>
inline void DecoderBase<T_PointCloud>::transformPoint(float& x, float& y, float& z)
{
#ifdef ENABLE_TRANSFORM
  Eigen::AngleAxisd current_rotation_x(param_.transform_param.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd current_rotation_y(param_.transform_param.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd current_rotation_z(param_.transform_param.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d current_translation(param_.transform_param.x, param_.transform_param.y,
                                           param_.transform_param.z);
  Eigen::Matrix4d trans = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
  Eigen::Vector4d target_ori(x, y, z, 1);
  Eigen::Vector4d target_rotate = trans * target_ori;
  x = target_rotate(0);
  y = target_rotate(1);
  z = target_rotate(2);
#endif
}
#endif

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

template <typename T_PointCloud>
inline float DecoderBase<T_PointCloud>::checkCosTable(const int& angle)
{
  return cos_lookup_table_[angle + RS_ONE_ROUND];
}
template <typename T_PointCloud>
inline float DecoderBase<T_PointCloud>::checkSinTable(const int& angle)
{
  return sin_lookup_table_[angle + RS_ONE_ROUND];
}

}  // namespace lidar
}  // namespace robosense
