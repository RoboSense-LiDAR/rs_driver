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

#define RS_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define RS_SWAP_LONG(x) ((((x)&0xFF) << 24) | (((x)&0xFF00) << 8) | (((x)&0xFF0000) >> 8) | (((x)&0xFF000000) >> 24))
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

#if 0
  virtual double getLidarTime(const uint8_t* pkt) = 0;
#endif

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
  int azimuthCalibration(const float& azimuth, const int& channel);
#endif

#if 0
  RSEchoMode getEchoMode(const LidarType& type, const uint8_t& return_mode);
#endif

#if 0
  template <typename T_Msop>
  double calculateTimeUTC(const uint8_t* pkt, const LidarType& type);
  template <typename T_Msop>
  double calculateTimeYMD(const uint8_t* pkt);
#endif

  template <typename T_Difop>
  void decodeDifopCommon(const uint8_t* pkt, const LidarType& type);

#if 0
  template <typename T_Difop>
  void decodeDifopCalibration(const uint8_t* pkt, const LidarType& type);
  void sortBeamTable();
#endif

#if 0
  void transformPoint(float& x, float& y, float& z);
#endif

  float checkCosTable(const int& angle);
  float checkSinTable(const int& angle);

#if 0
  std::vector<float> initTrigonometricLookupTable(const std::function<double(const double)>& func);
#endif

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

  unsigned int pkts_per_frame_;
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
  float azi_diff_between_block_theoretical_;

  std::vector<int> vert_angle_list_;
  std::vector<int> hori_angle_list_;
  std::vector<uint16_t> beam_ring_table_;

#if 0
  std::function<double(const uint8_t*)> get_point_time_func_;
#endif
  int lidar_alph0_;  // atan2(Ry, Rx) * 180 / M_PI * 100
  float lidar_Rxy_;  // sqrt(Rx*Rx + Ry*Ry)

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
  , pkts_per_frame_(lidar_const_param.PKT_RATE / 10)
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

#if 0
  // even though T_Point does not have timestamp, it gives the timestamp
  /* Point time function*/
  // typedef typename T_PointCloud::PointT T_Point;
  // if (RS_HAS_MEMBER(T_Point, timestamp))  ///< return the timestamp of the first block in one packet
  // {
  if (this->param_.use_lidar_clock)
  {
    get_point_time_func_ = [this](const uint8_t* pkt) { return getLidarTime(pkt); };
  }
  else
  {
    get_point_time_func_ = [this](const uint8_t* pkt) {
      double ret_time = getTime() - (this->lidar_const_param_.BLOCKS_PER_PKT - 1) * this->time_duration_between_blocks_;
      return ret_time;
    };
  }
// }
// else
// {
//   get_point_time_func_ = [this](const uint8_t* pkt) { return 0; };
// }

  /* Cos & Sin look-up table*/
  cos_lookup_table_ = initTrigonometricLookupTable([](const double rad) -> double { return std::cos(rad); });
  sin_lookup_table_ = initTrigonometricLookupTable([](const double rad) -> double { return std::sin(rad); });
#endif


  /*  Calulate the lidar_alph0 and lidar_Rxy */
  lidar_alph0_ = std::atan2(lidar_const_param_.RY, lidar_const_param_.RX) * 180 / M_PI * 100;
  lidar_Rxy_ = std::sqrt(lidar_const_param_.RX * lidar_const_param_.RX + lidar_const_param_.RY * lidar_const_param_.RY);
}

#if 0
template <typename T_PointCloud>
inline RSDecoderResult DecoderBase<T_PointCloud>::processDifopPkt(const uint8_t* pkt, size_t size)
{
  if (pkt == NULL)
  {
    return PKT_NULL;
  }

  if (size != this->difop_pkt_len_)
  {
    return WRONG_PKT_LENGTH;
  }

  return decodeDifopPkt(pkt);
}

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

#if 0
template <typename T_PointCloud>
inline int DecoderBase<T_PointCloud>::azimuthCalibration(const float& azimuth, const int& channel)
{
  return (static_cast<int>(azimuth) + this->hori_angle_list_[channel] + RS_ONE_ROUND) % RS_ONE_ROUND;
}
#endif

template <typename T_PointCloud>
template <typename T_Difop>
inline void DecoderBase<T_PointCloud>::decodeDifopCommon(const uint8_t* pkt, const LidarType& type)
{
  const T_Difop* dpkt_ptr = reinterpret_cast<const T_Difop*>(pkt);

#if 0
  this->echo_mode_ = this->getEchoMode(type, dpkt_ptr->return_mode);
#endif

  this->rpm_ = RS_SWAP_SHORT(dpkt_ptr->rpm);
  if (this->rpm_ == 0)
  {
    RS_WARNING << "LiDAR RPM is 0" << RS_REND;
    this->rpm_ = 600;
  }

  this->time_duration_between_blocks_ = 
    (60 / static_cast<float>(this->rpm_)) / ((this->lidar_const_param_.PKT_RATE * 60 / this->rpm_) * this->lidar_const_param_.BLOCKS_PER_PKT);

  int fov_start_angle = RS_SWAP_SHORT(dpkt_ptr->fov.start_angle);
  int fov_end_angle = RS_SWAP_SHORT(dpkt_ptr->fov.end_angle);
  int fov_range = (fov_start_angle < fov_end_angle) ? (fov_end_angle - fov_start_angle) :
                                                      (RS_ONE_ROUND - fov_start_angle + fov_end_angle);
  int blocks_per_round =
      (this->lidar_const_param_.PKT_RATE / (this->rpm_ / 60)) * this->lidar_const_param_.BLOCKS_PER_PKT;

  this->fov_time_jump_diff_ =
      this->time_duration_between_blocks_ * (fov_range / (RS_ONE_ROUND / static_cast<float>(blocks_per_round)));

  if (this->echo_mode_ == RSEchoMode::ECHO_DUAL)
  {
    this->pkts_per_frame_ = ceil(2 * this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  }
  else
  {
    this->pkts_per_frame_ = ceil(this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  }

  this->azi_diff_between_block_theoretical_ =
      (RS_ONE_ROUND / this->lidar_const_param_.BLOCKS_PER_PKT) /
      static_cast<float>(this->pkts_per_frame_);  ///< ((rpm/60)*360)/pkts_rate/blocks_per_pkt
}

#if 0
template <typename T_PointCloud>
template <typename T_Difop>
inline void DecoderBase<T_PointCloud>::decodeDifopCalibration(const uint8_t* pkt, const LidarType& type)
{
  const T_Difop* dpkt_ptr = reinterpret_cast<const T_Difop*>(pkt);

  const uint8_t* p_ver_cali = reinterpret_cast<const uint8_t*>(dpkt_ptr->ver_angle_cali);

  if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) && 
      (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
      (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) && 
      (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF))
  {
    return;
  }

  int neg = 1;
  for (size_t i = 0; i < this->lidar_const_param_.LASER_NUM; i++)
  {
    /* vert angle calibration data */
    neg = static_cast<int>(dpkt_ptr->ver_angle_cali[i].sign) == 0 ? 1 : -1;
    this->vert_angle_list_[i] = static_cast<int>(RS_SWAP_SHORT(dpkt_ptr->ver_angle_cali[i].value)) * neg;

    /* horizon angle calibration data */
    neg = static_cast<int>(dpkt_ptr->hori_angle_cali[i].sign) == 0 ? 1 : -1;
    this->hori_angle_list_[i] = static_cast<int>(RS_SWAP_SHORT(dpkt_ptr->hori_angle_cali[i].value)) * neg;

    if (type == LidarType::RS32)
    {
      this->vert_angle_list_[i] *= 0.1f;
      this->hori_angle_list_[i] *= 0.1f;
    }
  }

  this->sortBeamTable();

  this->difop_flag_ = true;
}
#endif

#if 0
template <typename T_PointCloud>
template <typename T_Msop>
inline double DecoderBase<T_PointCloud>::calculateTimeUTC(const uint8_t* pkt, const LidarType& type)
{
  const T_Msop* mpkt_ptr = reinterpret_cast<const T_Msop*>(pkt);
  union u_ts
  {
    uint8_t data[8];
    uint64_t ts;
  } timestamp;
  timestamp.data[7] = 0;
  timestamp.data[6] = 0;
  timestamp.data[5] = mpkt_ptr->header.timestamp.sec[0];
  timestamp.data[4] = mpkt_ptr->header.timestamp.sec[1];
  timestamp.data[3] = mpkt_ptr->header.timestamp.sec[2];
  timestamp.data[2] = mpkt_ptr->header.timestamp.sec[3];
  timestamp.data[1] = mpkt_ptr->header.timestamp.sec[4];
  timestamp.data[0] = mpkt_ptr->header.timestamp.sec[5];

  if ((type == LidarType::RS80 || type == LidarType::RS128) && this->protocol_ver_ == PROTOCOL_VER_0)
  {
    return static_cast<double>(timestamp.ts) +
           (static_cast<double>(RS_SWAP_LONG(mpkt_ptr->header.timestamp.us))) / NANO;
  }

  return static_cast<double>(timestamp.ts) + (static_cast<double>(RS_SWAP_LONG(mpkt_ptr->header.timestamp.us))) / MICRO;
}

template <typename T_PointCloud>
template <typename T_Msop>
inline double DecoderBase<T_PointCloud>::calculateTimeYMD(const uint8_t* pkt)
{
#ifdef _MSC_VER
  long timezone = 0;
  _get_timezone(&timezone);
#endif
  const T_Msop* mpkt_ptr = reinterpret_cast<const T_Msop*>(pkt);
  std::tm stm;
  memset(&stm, 0, sizeof(stm));
  stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
  stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
  stm.tm_mday = mpkt_ptr->header.timestamp.day;
  stm.tm_hour = mpkt_ptr->header.timestamp.hour;
  stm.tm_min = mpkt_ptr->header.timestamp.minute;
  stm.tm_sec = mpkt_ptr->header.timestamp.second;
  return std::mktime(&stm) + static_cast<double>(RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms)) / 1000.0 +
         static_cast<double>(RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us)) / 1000000.0 - static_cast<double>(timezone);
}

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
inline void DecoderBase<T_PointCloud>::sortBeamTable()
{
  std::vector<size_t> sorted_idx(this->lidar_const_param_.LASER_NUM);
  std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
  std::sort(sorted_idx.begin(), sorted_idx.end(), [this](std::size_t i1, std::size_t i2) -> bool {
    return this->vert_angle_list_[i1] < this->vert_angle_list_[i2];
  });
  for (size_t i = 0; i < this->lidar_const_param_.LASER_NUM; i++)
  {
    this->beam_ring_table_[sorted_idx[i]] = i;
  }
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

#if 0
template <typename T_PointCloud>
inline std::vector<float>
DecoderBase<T_PointCloud>::initTrigonometricLookupTable(const std::function<double(const double)>& func)
{
  std::vector<float> temp_table = std::vector<float>(2 * RS_ONE_ROUND, 0.0);

  for (int i = 0; i < 2 * RS_ONE_ROUND; i++)
  {
    const double rad = RS_TO_RADS(static_cast<double>(i - RS_ONE_ROUND) * RS_ANGLE_RESOLUTION);
    temp_table[i] = (float)func(rad);
  }
  return temp_table;
}
#endif

}  // namespace lidar
}  // namespace robosense
