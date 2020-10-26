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
#include <rs_driver/driver/decoder/decoder_base.hpp>

namespace robosense
{
namespace lidar
{
const uint32_t SINGLE_PKT_NUM = 630;
const uint32_t DUAL_PKT_NUM = 1260;
const int ANGLE_OFFSET = 32768;
#pragma pack(push, 1)

typedef struct
{
  uint16_t distance;
  uint16_t pitch;
  uint16_t yaw;
  uint8_t intensity;
  uint8_t point_attribute;
  uint8_t elongation;
} RSM1Channel;

typedef struct
{
  uint8_t time_offset;
  uint8_t return_seq;
  RSM1Channel channel[5];
} RSM1Block;

typedef struct
{
  uint8_t sec[6];
  uint32_t ms;
} RSTimestampM1;

typedef struct
{
  uint32_t id;
  uint16_t pkt_cnt;
  uint16_t protocal_version;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimestampM1 timestamp;
  uint8_t reserve[10];
  uint8_t lidar_type;
  uint8_t temperature;
} RSM1MsopHeader;

typedef struct
{
  RSM1MsopHeader header;
  RSM1Block blocks[25];
  uint8_t reserve[3];
} RSM1MsopPkt;

typedef struct
{
  uint8_t ip_local[4];
  uint8_t ip_remote[4];
  uint8_t mac[6];
  uint8_t msop_port[2];
  uint8_t difop_port[2];
} RSM1DifopEther;

typedef struct
{
  uint8_t horizontal_fov_start[2];
  uint8_t horizontal_fov_end[2];
  uint8_t vertical_fov_start[2];
  uint8_t vertical_fov_end[2];
} RSM1DifopFov;

typedef struct
{
  uint8_t pl_ver[5];
  uint8_t ps_ver[5];
} RSM1DifopVerInfo;

typedef struct
{
  uint8_t syn_methord;
  uint8_t syn_status;
  uint8_t current_time[10];
} RSM1DifopTimeInfo;

typedef struct
{
  uint8_t current1[3];
  uint8_t current2[3];
  uint8_t voltage1[2];
  uint8_t voltage2[2];
  uint8_t reserve[10];
} RSM1DifopRunSts;

typedef struct
{
  uint8_t param_sign;
  uint8_t data[2];
} RSM1DifopCalibration;

typedef struct
{
  uint64_t id;
  uint8_t reserve_1;
  uint8_t frame_rate;
  RSM1DifopEther ether;
  RSM1DifopFov fov_setting;
  RSM1DifopVerInfo ver_info;
  uint8_t serial_number[6];
  uint8_t return_mode;
  RSM1DifopTimeInfo time_info;
  RSM1DifopRunSts run_status;
  uint8_t diag_reserve[40];
  RSM1DifopCalibration cali_param[20];
  uint8_t reserve_2[71];
} RSM1DifopPkt;
#pragma pack(pop)

template <typename T_Point>
class DecoderRSM1 : public DecoderBase<T_Point>
{
public:
  DecoderRSM1(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);
  RSDecoderResult processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& pointcloud_vec, int& height);

private:
  double calculateTimeM1(const uint8_t* pkt);
  uint32_t last_pkt_cnt_;
  uint32_t max_pkt_num_;
  double last_pkt_time_;
};

template <typename T_Point>
inline DecoderRSM1<T_Point>::DecoderRSM1(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : DecoderBase<T_Point>(param, lidar_const_param), last_pkt_cnt_(1), max_pkt_num_(SINGLE_PKT_NUM), last_pkt_time_(0)
{
  if (this->param_.max_distance > 200.0f)
  {
    this->param_.max_distance = 200.0f;
  }
  if (this->param_.min_distance < 0.2f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 0.2f;
  }
  this->time_duration_between_blocks_ = 5 * 1e-6;
}

template <typename T_Point>
inline double DecoderRSM1<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeM1(pkt);
}

template <typename T_Point>
inline RSDecoderResult DecoderRSM1<T_Point>::processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& pointcloud_vec,
                                                            int& height)
{
  int azimuth = 0;
  RSDecoderResult ret = decodeMsopPkt(pkt, pointcloud_vec, height, azimuth);
  return ret;
}

template <typename T_Point>
inline RSDecoderResult DecoderRSM1<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                           int& azimuth)
{
  height = this->lidar_const_param_.LASER_NUM;
  RSM1MsopPkt* mpkt_ptr = (RSM1MsopPkt*)pkt;
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  double pkt_timestamp = 0;
  switch (mpkt_ptr->blocks[0].return_seq)
  {
    case 0:
      pkt_timestamp = this->get_point_time_func_(pkt);
      break;
    case 1:
      pkt_timestamp = this->get_point_time_func_(pkt);
      last_pkt_time_ = pkt_timestamp;
      break;
    case 2:
      pkt_timestamp = last_pkt_time_;
      break;
  }

  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    RSM1Block blk = mpkt_ptr->blocks[blk_idx];
    double point_time = pkt_timestamp + blk.time_offset * 1e-6;
    for (size_t channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      T_Point point;
      float distance = RS_SWAP_SHORT(blk.channel[channel_idx].distance) * RS_DIS_RESOLUTION;
      if (distance <= this->param_.max_distance && distance >= this->param_.min_distance)
      {
        int pitch = RS_SWAP_SHORT(blk.channel[channel_idx].pitch) - ANGLE_OFFSET;
        int yaw = RS_SWAP_SHORT(blk.channel[channel_idx].yaw) - ANGLE_OFFSET;
        uint8_t intensity = blk.channel[channel_idx].intensity;
        float x = distance * this->checkCosTable(pitch) * this->checkCosTable(yaw);
        float y = distance * this->checkCosTable(pitch) * this->checkSinTable(yaw);
        float z = distance * this->checkSinTable(pitch);
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
      setTimestamp(point, point_time);
      setRing(point,channel_idx+1);
      vec.emplace_back(std::move(point));
    }
  }
  unsigned int pkt_cnt = RS_SWAP_SHORT(mpkt_ptr->header.pkt_cnt);
  if (pkt_cnt == max_pkt_num_ || pkt_cnt < last_pkt_cnt_)
  {
    last_pkt_cnt_ = 1;
    return RSDecoderResult::FRAME_SPLIT;
  }
  last_pkt_cnt_ = pkt_cnt;
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline RSDecoderResult DecoderRSM1<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  RSM1DifopPkt* dpkt_ptr = (RSM1DifopPkt*)pkt;
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  if (!this->difop_flag_)
  {
    switch (dpkt_ptr->return_mode)
    {
      case 0x00:
        this->echo_mode_ = RSEchoMode::ECHO_DUAL;
        max_pkt_num_ = DUAL_PKT_NUM;
        break;
      case 0x01:
        this->echo_mode_ = RSEchoMode::ECHO_STRONGEST;
        break;
      case 0x02:
        this->echo_mode_ = RSEchoMode::ECHO_LAST;
        break;
      default:
        break;
    }
    this->difop_flag_ = true;
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline double DecoderRSM1<T_Point>::calculateTimeM1(const uint8_t* pkt)
{
  RSM1MsopPkt* mpkt_ptr = (RSM1MsopPkt*)pkt;
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
  return (double)timestamp.ts + ((double)(RS_SWAP_LONG(mpkt_ptr->header.timestamp.ms))) / 1000000.0d;
}

}  // namespace lidar
}  // namespace robosense
