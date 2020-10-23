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
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint8_t intensity;
  uint8_t point_attribute;
  uint8_t elongation;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1Channel;

typedef struct
{
  uint8_t time_offset;
  uint8_t return_seq;
  RSM1Channel channel[5];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1Block;

typedef struct
{
  uint32_t id;
  uint16_t pkt_cnt;
  uint16_t protocal_version;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimestampUTC timestamp_utc;
  uint8_t reserve[10];
  uint8_t lidar_type;
  uint8_t temperature;
} RSM1MsopHeader;

typedef struct
{
  RSM1MsopHeader header;
  RSM1Block blocks[25];
  uint8_t reserve[3];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1MsopPkt;

typedef struct
{
  uint8_t ip_local[4];
  uint8_t ip_remote[4];
  uint8_t mac[6];
  uint8_t msop_port[2];
  uint8_t difop_port[2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopEther;

typedef struct
{
  uint8_t horizontal_fov_start[2];
  uint8_t horizontal_fov_end[2];
  uint8_t vertical_fov_start[2];
  uint8_t vertical_fov_end[2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopFov;

typedef struct
{
  uint8_t pl_ver[5];
  uint8_t ps_ver[5];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopVerInfo;

typedef struct
{
  uint8_t syn_methord;
  uint8_t syn_status;
  uint8_t current_time[10];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopTimeInfo;

typedef struct
{
  uint8_t current1[3];
  uint8_t current2[3];
  uint8_t voltage1[2];
  uint8_t voltage2[2];
  uint8_t reserve[10];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopRunSts;

typedef struct
{
  uint8_t param_sign;
  uint8_t data[2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopCalibration;

typedef struct
{
  uint64_t id;
  uint8_t reserve_1;
  uint8_t frame_rate;
  RSM1DifopEther ether;
  RSM1DifopFov fov_setting;
  RSM1DifopVerInfo ver_info;
  uint8_t serial_number[6];
  uint8_t wave_mode;
  RSM1DifopTimeInfo time_info;
  RSM1DifopRunSts run_status;
  uint8_t diag_reserve[40];
  RSM1DifopCalibration cali_param[20];
  uint8_t reserve_2[71];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopPkt;
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
};

template <typename T_Point>
DecoderRSM1<T_Point>::DecoderRSM1(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : DecoderBase<T_Point>(param, lidar_const_param)
{
}

template <typename T_Point>
double DecoderRSM1<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeUTC<RSM1MsopPkt>(pkt);
}

template <typename T_Point>
RSDecoderResult DecoderRSM1<T_Point>::processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& pointcloud_vec,
                                                     int& height)
{
  int azimuth = 0;
  RSDecoderResult ret = decodeMsopPkt(pkt, pointcloud_vec, height, azimuth);
  return ret;
}

template <typename T_Point>
RSDecoderResult DecoderRSM1<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                    int& azimuth)
{
  height = this->lidar_const_param_.LASER_NUM;
  RSM1MsopPkt* mpkt_ptr = (RSM1MsopPkt*)pkt;
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }

  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    RSM1Block blk = mpkt_ptr->blocks[blk_idx];

    for (size_t channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      T_Point point;
      double dis = RS_SWAP_SHORT(blk.channel[channel_idx].x) * RS_DIS_RESOLUTION;
      double pitch = (RS_SWAP_SHORT(blk.channel[channel_idx].y) - ANGLE_OFFSET);
      double yaw = (RS_SWAP_SHORT(blk.channel[channel_idx].z) - ANGLE_OFFSET);
      double intensity = blk.channel[channel_idx].intensity;
      double x = dis * this->checkCosTable(pitch) * this->checkCosTable(yaw);
      double y = dis * this->checkCosTable(pitch) * this->checkSinTable(yaw);
      double z = dis * this->checkSinTable(pitch);
      setX(point, x);
      setY(point, y);
      setZ(point, z);
      setIntensity(point, intensity);
      vec.emplace_back(std::move(point));
    }
  }
  unsigned int pkt_cnt = RS_SWAP_SHORT(mpkt_ptr->header.pkt_cnt);
  if (pkt_cnt == SINGLE_PKT_NUM)
  {
    return RSDecoderResult::FRAME_SPLIT;
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
RSDecoderResult DecoderRSM1<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  RSM1DifopPkt* difop_ptr = (RSM1DifopPkt*)pkt;

  if (difop_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  if (!this->difop_flag_)
  {
    this->difop_flag_ = true;
  }
  return RSDecoderResult::DECODE_OK;
}

}  // namespace lidar
}  // namespace robosense
