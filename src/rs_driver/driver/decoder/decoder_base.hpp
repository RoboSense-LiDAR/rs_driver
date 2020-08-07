/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#pragma once
#include <rs_driver/common/common_header.h>
namespace robosense
{
namespace lidar
{
#define RS_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define RS_SWAP_LONG(x) ((((x)&0xFF) << 24) | (((x)&0xFF00) << 8) | (((x)&0xFF0000) >> 8) | (((x)&0xFF000000) >> 24))
#define RS_TO_RADS(x) ((x) * (M_PI) / 180)
const double RS_RESOLUTION = 0.005;

/* Echo mode definition */
enum RSEchoMode
{
  ECHO_STRONGEST = 0,
  ECHO_LAST,
  ECHO_DUAL
};

/* Decode result definition */
enum RSDecoderResult
{
  DECODE_OK = 0,
  FRAME_SPLIT = 1,
  WRONG_PKT_HEADER = -1,
  PKT_NULL = -2
};

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif
typedef struct
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t ms;
  uint16_t us;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSTimestamp;

typedef struct
{
  uint8_t sec[6];
  uint32_t ns;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSTimestampUTC;

typedef struct
{
  uint64_t id;
  uint8_t reserved1[12];
  RSTimestamp timestamp;
  uint8_t lidar_type;
  uint8_t reserved2[7];
  uint16_t temp_raw;
  uint8_t reserved3[2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSMsopHeader;

typedef struct
{
  uint8_t lidar_ip[4];
  uint8_t host_ip[4];
  uint8_t mac_addr[6];
  uint16_t local_port;
  uint16_t dest_port;
  uint16_t port3;
  uint16_t port4;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSEthNet;

typedef struct
{
  uint16_t start_angle;
  uint16_t end_angle;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSFOV;

typedef struct
{
  uint8_t sign;
  uint8_t value[2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSCorAngle;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSChannel;

typedef struct
{
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSVersion;

typedef struct
{
  uint8_t num[6];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSSn;

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
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSStatus;

typedef struct
{
  uint8_t reserved1[9];
  uint16_t checksum;
  uint16_t manc_err1;
  uint16_t manc_err2;
  uint8_t gps_status;
  uint16_t temperature1;
  uint16_t temperature2;
  uint16_t temperature3;
  uint16_t temperature4;
  uint16_t temperature5;
  uint8_t reserved2[5];
  uint16_t cur_rpm;
  uint8_t reserved3[7];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSDiagno;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

//----------------- Decoder ---------------------
template <typename T_Point>
class DecoderBase
{
public:
  DecoderBase(const RSDecoderParam& param);
  DecoderBase(const DecoderBase&) = delete;
  DecoderBase& operator=(const DecoderBase&) = delete;
  virtual ~DecoderBase();
  virtual RSDecoderResult processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& point_cloud_vec, int& height);
  virtual RSDecoderResult processDifopPkt(const uint8_t* pkt);
  virtual void loadCalibrationFile(const std::string& angle_path);
  virtual void regRecvCallback(const std::function<void(const CameraTrigger&)> callback);  ///< Camera trigger
  virtual double getLidarTime(const uint8_t* pkt) = 0;
  virtual double getLidarTemperature();

protected:
  virtual float computeTemperature(const uint16_t& temp_raw);
  virtual float computeTemperature(const uint8_t& temp_low, const uint8_t& temp_high);
  virtual int32_t azimuthCalibration(float azimuth, const int& channel);
  virtual void checkTriggerAngle(const int& angle, const double& timestamp);
  virtual RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth) = 0;
  virtual RSDecoderResult decodeDifopPkt(const uint8_t* pkt) = 0;

protected:
  int rpm_;
  uint8_t echo_mode_;
  int angle_file_index_;
  float Rx_;
  float Ry_;
  float Rz_;
  float max_distance_;
  float min_distance_;
  int start_angle_;
  int end_angle_;
  bool angle_flag_;
  bool difop_flag_;
  bool trigger_flag_;
  bool use_lidar_clock_;
  uint32_t pkts_per_frame_;
  uint32_t pkt_counter_;
  uint16_t mode_split_frame_;  // 1 - angle,  2 - theoretical packets; 3 - setting packets
  uint32_t num_pkts_split_;    // number of setting packets
  int32_t cut_angle_;
  int32_t last_azimuth_;
  unsigned int trigger_index_;
  unsigned int prev_angle_diff_;
  RSDecoderParam param_;
  double current_temperature_;
  // calibration data
  float vert_angle_list_[128];
  float hori_angle_list_[128];
  std::vector<std::function<void(const CameraTrigger&)>> camera_trigger_cb_vec_;
  static std::vector<double> cos_lookup_table_;
  static std::vector<double> sin_lookup_table_;
};

template <typename T_Point>
DecoderBase<T_Point>::DecoderBase(const RSDecoderParam& param)
  : param_(param)
  , rpm_(600)
  , pkts_per_frame_(84)
  , pkt_counter_(0)
  , last_azimuth_(-36001)
  , current_temperature_(0)
  , trigger_index_(0)
  , prev_angle_diff_(36000)
  , difop_flag_(false)
  , angle_flag_(true)
  , trigger_flag_(false)
  , use_lidar_clock_(param.use_lidar_clock)
  , start_angle_(param.start_angle * 100)
  , end_angle_(param.end_angle * 100)
  , echo_mode_(ECHO_STRONGEST)
  , max_distance_(param.max_distance)
  , min_distance_(param.min_distance)
  , mode_split_frame_(param.mode_split_frame)
  , num_pkts_split_(param.num_pkts_split)
  , cut_angle_(param.cut_angle * 100)
{
  if (cut_angle_ > 36000)
  {
    cut_angle_ = 0;
  }

  if (this->start_angle_ > 36000 || this->start_angle_ < 0 || this->end_angle_ > 36000 || this->end_angle_ < 0)
  {
    this->start_angle_ = 0;
    this->end_angle_ = 36000;
  }
  if (this->start_angle_ > this->end_angle_)
  {
    this->angle_flag_ = false;
  }

  if (param.trigger_param.trigger_map.size() != 0)
  {
    trigger_flag_ = true;
  }
}

template <typename T_Point>
DecoderBase<T_Point>::~DecoderBase()
{
}

template <typename T_Point>
RSDecoderResult DecoderBase<T_Point>::processDifopPkt(const uint8_t* pkt)
{
  if (pkt == NULL)
  {
    return PKT_NULL;
  }
  int ret = decodeDifopPkt(pkt);
  if (ret < 0)
  {
    return WRONG_PKT_HEADER;
  }
  return DECODE_OK;
}

template <typename T_Point>
RSDecoderResult DecoderBase<T_Point>::processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& point_cloud_vec,
                                                     int& height)
{
  if (pkt == NULL)
  {
    return PKT_NULL;
  }
  int azimuth = 0;
  RSDecoderResult ret = decodeMsopPkt(pkt, point_cloud_vec, height, azimuth);
  if (ret != RSDecoderResult::DECODE_OK)
  {
    return ret;
  }
  this->pkt_counter_++;
  if (mode_split_frame_ == 1)
  {
    if (azimuth < this->last_azimuth_)
    {
      this->last_azimuth_ -= 36000;
    }
    if (this->last_azimuth_ != -36001 && this->last_azimuth_ < this->cut_angle_ && azimuth >= this->cut_angle_)
    {
      this->last_azimuth_ = azimuth;
      this->pkt_counter_ = 0;
      this->trigger_index_ = 0;
      this->prev_angle_diff_ = 36000;
      return FRAME_SPLIT;
    }
    this->last_azimuth_ = azimuth;
  }
  else if (mode_split_frame_ == 2)
  {
    if (this->pkt_counter_ >= this->pkts_per_frame_)
    {
      this->pkt_counter_ = 0;
      this->trigger_index_ = 0;
      this->prev_angle_diff_ = 36000;
      return FRAME_SPLIT;
    }
  }
  else if (mode_split_frame_ == 3)
  {
    if (this->pkt_counter_ >= this->num_pkts_split_)
    {
      this->pkt_counter_ = 0;
      this->trigger_index_ = 0;
      this->prev_angle_diff_ = 36000;
      return FRAME_SPLIT;
    }
  }
  return DECODE_OK;
}

template <typename T_Point>
void DecoderBase<T_Point>::checkTriggerAngle(const int& angle, const double& timestamp)
{
  std::map<double, std::string>::iterator iter = param_.trigger_param.trigger_map.begin();
  for (size_t i = 0; i < trigger_index_; i++)
  {
    iter++;
  }
  if (iter != param_.trigger_param.trigger_map.end())
  {
    unsigned int angle_diff = std::abs(iter->first * 100 - angle);
    if (angle_diff < prev_angle_diff_)
    {
      prev_angle_diff_ = angle_diff;
      return;
    }
    else
    {
      trigger_index_++;
      prev_angle_diff_ = 36000;
      for (auto cb : camera_trigger_cb_vec_)
      {
        cb(std::make_pair(iter->second, timestamp));
      }
    }
  }
}

template <typename T_Point>
void DecoderBase<T_Point>::regRecvCallback(const std::function<void(const CameraTrigger&)> callback)
{
  camera_trigger_cb_vec_.emplace_back(callback);
}

template <typename T_Point>
double DecoderBase<T_Point>::getLidarTemperature()
{
  return current_temperature_;
}

/* 16, 32, & BP */
template <typename T_Point>
float DecoderBase<T_Point>::computeTemperature(const uint16_t& temp_raw)
{
  uint8_t neg_flag = (temp_raw >> 8) & 0x80;
  float msb = (temp_raw >> 8) & 0x7F;
  float lsb = (temp_raw & 0x00FF) >> 3;
  float temp;
  if (neg_flag == 0x80)
  {
    temp = -1 * (msb * 32 + lsb) * 0.0625f;
  }
  else
  {
    temp = (msb * 32 + lsb) * 0.0625f;
  }

  return temp;
}

/* 128 & 80 */
template <typename T_Point>
float DecoderBase<T_Point>::computeTemperature(const uint8_t& temp_low, const uint8_t& temp_high)
{
  uint8_t neg_flag = temp_low & 0x80;
  float msb = temp_low & 0x7F;
  float lsb = temp_high >> 4;
  float temp;
  if (neg_flag == 0x80)
  {
    temp = -1 * (msb * 16 + lsb) * 0.0625f;
  }
  else
  {
    temp = (msb * 16 + lsb) * 0.0625f;
  }

  return temp;
}

template <typename T_Point>
int DecoderBase<T_Point>::azimuthCalibration(float azimuth, const int& channel)
{
  int azi_ret;
  azimuth += this->hori_angle_list_[channel];
  azi_ret = (int)azimuth;
  azi_ret = ((azi_ret % 36000) + 36000) % 36000;
  return azi_ret;
}

template <typename T_Point>
void DecoderBase<T_Point>::loadCalibrationFile(const std::string& angle_path)
{
  int row_index = 0;
  std::string line_str;
  // read angle.csv
  std::ifstream fd_angle(angle_path.c_str(), std::ios::in);
  if (fd_angle.is_open())
  {
    row_index = 0;
    while (std::getline(fd_angle, line_str))
    {
      std::stringstream ss(line_str);
      std::string str;
      std::vector<std::string> vect_str;
      while (std::getline(ss, str, ','))
      {
        vect_str.emplace_back(str);
      }
      try
      {
        this->vert_angle_list_[row_index] = std::stof(vect_str.at(0)) * 100;  // degree
        this->hori_angle_list_[row_index] = std::stof(vect_str.at(1)) * 100;  // degree
      }
      catch (...)
      {
        WARNING << "Wrong calibration file format! Please check your angle.csv file!" << REND;
        break;
      }
      row_index++;
      if (row_index >= angle_file_index_)
      {
        break;
      }
    }
    fd_angle.close();
  }
}

inline const std::vector<double>
initTrigonometricLookupTable(const std::function<double(const double)> trigonometric_fun)
{
  std::vector<double> temp_table = std::vector<double>(36000, 0.0);

  for (int i = 0; i < 36000; ++i)
  {
    const double rad = RS_TO_RADS(static_cast<double>(i) / 100.0);
    temp_table[i] = trigonometric_fun(rad);
  }
  return temp_table;
}

template <typename T_Point>
std::vector<double> DecoderBase<T_Point>::cos_lookup_table_ =
    initTrigonometricLookupTable([](const double rad) -> double { return std::cos(rad); });
template <typename T_Point>
std::vector<double> DecoderBase<T_Point>::sin_lookup_table_ =
    initTrigonometricLookupTable([](const double rad) -> double { return std::sin(rad); });

}  // namespace lidar
}  // namespace robosense