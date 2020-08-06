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
#define RS128_MSOP_ID (0x5A05AA55)  // big endian
#define RS128_BLOCK_ID (0xFE)
#define RS128_DIFOP_ID (0x555511115A00FFA5)  // big endian
#define RS128_CHANNELS_PER_BLOCK (128)
#define RS128_BLOCKS_PER_PKT (3)
#define RS128_TEMPERATURE_MIN (31)
#define RS128_TEMPERATURE_RANGE (50)
#define RS128_DSR_TOFFSET (3.23)
#define RS128_BLOCK_TDURATION (55.55)
const int RS128_PKT_RATE = 6000;
#ifdef _MSC_VER
#pragma pack(push, 1)
#endif
typedef struct
{
  uint8_t id;
  uint8_t ret_id;
  uint16_t azimuth;
  RSChannel channels[RS128_CHANNELS_PER_BLOCK];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS128MsopBlock;

typedef struct
{
  uint32_t id;
  uint8_t reserved1[3];
  uint8_t wave_mode;
  uint8_t temp_low;
  uint8_t temp_high;
  RSTimestampUTC timestamp_utc;
  uint8_t reserved2[10];
  uint8_t lidar_type;
  uint8_t reserved3[49];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS128MsopHeader;

typedef struct
{
  RS128MsopHeader header;
  RS128MsopBlock blocks[RS128_BLOCKS_PER_PKT];
  uint32_t index;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS128MsopPkt;

typedef struct
{
  uint8_t sync_mode;
  uint8_t sync_sts;
  RSTimestamp timestamp;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS128TimeInfo;

typedef struct
{
  uint8_t lidar_ip[4];
  uint8_t dest_ip[4];
  uint8_t mac_addr[6];
  uint16_t msop_port;
  uint16_t reserve_1;
  uint16_t difop_port;
  uint16_t reserve_2;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS128EthNet;

typedef struct
{
  uint8_t top_firmware_ver[5];
  uint8_t bot_firmware_ver[5];
  uint8_t bot_soft_ver[5];
  uint8_t motor_firmware_ver[5];
  uint8_t hw_ver[3];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS128Version;

typedef struct
{
  uint64_t id;
  uint16_t rpm;
  RS128EthNet eth;
  RSFOV fov;
  uint16_t reserved_0;
  uint16_t phase_lock_angle;
  RS128Version version;
  uint8_t reserved_1[229];
  RSSn sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  RS128TimeInfo time_info;
  RSStatus status;
  uint8_t reserved_2[5];
  RSDiagno diagno;
  uint8_t gprmc[86];
  RSCorAngle ver_angle_cali[128];
  RSCorAngle hori_angle_cali[128];
  uint8_t reserved_3[10];
  uint16_t tail;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS128DifopPkt;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

template <typename T_Point>
class DecoderRS128 : public DecoderBase<T_Point>
{
public:
  DecoderRS128(const RSDecoderParam& param);
  int32_t decodeDifopPkt(const uint8_t* pkt);
  int32_t decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height);
  double getLidarTime(const uint8_t* pkt);
};

template <typename T_Point>
DecoderRS128<T_Point>::DecoderRS128(const RSDecoderParam& param) : DecoderBase<T_Point>(param)
{
  this->Rx_ = 0.03615;
  this->Ry_ = -0.017;
  this->Rz_ = 0;
  this->angle_file_index_ = 128;
  if (this->max_distance_ > 250.0f)
  {
    this->max_distance_ = 250.0f;
  }
  if (this->min_distance_ < 1.0f || this->min_distance_ > this->max_distance_)
  {
    this->min_distance_ = 1.0f;
  }
}

template <typename T_Point>
double DecoderRS128<T_Point>::getLidarTime(const uint8_t* pkt)
{
  RS128MsopPkt* mpkt_ptr = (RS128MsopPkt*)pkt;
  union u_ts
  {
    uint8_t data[8];
    uint64_t ts;
  } timestamp;

  timestamp.data[7] = 0;
  timestamp.data[6] = 0;
  timestamp.data[5] = mpkt_ptr->header.timestamp_utc.sec[0];
  timestamp.data[4] = mpkt_ptr->header.timestamp_utc.sec[1];
  timestamp.data[3] = mpkt_ptr->header.timestamp_utc.sec[2];
  timestamp.data[2] = mpkt_ptr->header.timestamp_utc.sec[3];
  timestamp.data[1] = mpkt_ptr->header.timestamp_utc.sec[4];
  timestamp.data[0] = mpkt_ptr->header.timestamp_utc.sec[5];
  return (double)timestamp.ts + ((double)(RS_SWAP_LONG(mpkt_ptr->header.timestamp_utc.ns))) / 1000000000.0d;
}

template <typename T_Point>
int DecoderRS128<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height)
{
  height = 128;
  RS128MsopPkt* mpkt_ptr = (RS128MsopPkt*)pkt;
  if (mpkt_ptr->header.id != RS128_MSOP_ID)
  {
    return RSDecoderResult::DECODE_FAIL;
  }
  this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_low, mpkt_ptr->header.temp_high);
  int first_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  int second_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[1].azimuth);
  int third_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[2].azimuth);
  if (this->trigger_flag_)
  {
    if (this->use_lidar_clock_)
    {
      this->checkTriggerAngle(first_azimuth, getLidarTime(pkt));
    }
    else
    {
      this->checkTriggerAngle(first_azimuth, getTime());
    }
  }
  for (int blk_idx = 0; blk_idx < RS128_BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != RS128_BLOCK_ID)
    {
      break;
    }
    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    float azi_diff = 0;
    if (this->echo_mode_ == ECHO_DUAL)
    {
      azi_diff = (float)((36000 + third_azimuth - first_azimuth) % 36000);
    }
    else
    {
      switch (blk_idx)
      {
        case 0:
          azi_diff = (float)((36000 + second_azimuth - first_azimuth) % 36000);
          break;
        case 1:
        case 2:
          azi_diff = (float)((36000 + third_azimuth - second_azimuth) % 36000);
          break;
      }
    }

    for (int channel_idx = 0; channel_idx < RS128_CHANNELS_PER_BLOCK; channel_idx++)
    {
      int dsr_temp = (channel_idx / 4) % 16;
      float azi_channel_ori = cur_azi + (azi_diff * (dsr_temp * RS128_DSR_TOFFSET) / RS128_BLOCK_TDURATION);
      int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx);
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * RS_RESOLUTION;
      int angle_horiz = (int)(azi_channel_ori + 36000) % 36000;
      int angle_vert = (((int)(this->vert_angle_list_[channel_idx]) % 36000) + 36000) % 36000;

      T_Point point;
      if ((distance <= this->max_distance_ && distance >= this->min_distance_) &&
          ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
           (!this->angle_flag_ &&
            ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))
      {
        point.x = distance * this->cos_lookup_table_[angle_vert] * this->cos_lookup_table_[azi_channel_final] +
                  this->Rx_ * this->cos_lookup_table_[angle_horiz];
        point.y = -distance * this->cos_lookup_table_[angle_vert] * this->sin_lookup_table_[azi_channel_final] -
                  this->Rx_ * this->sin_lookup_table_[angle_horiz];
        point.z = distance * this->sin_lookup_table_[angle_vert] + this->Rz_;
        point.intensity = mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
        if (std::isnan(point.intensity))
        {
          point.intensity = 0;
        }
      }
      else
      {
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
        point.intensity = 0;
      }
      vec.emplace_back(std::move(point));
    }
  }
  return first_azimuth;
}

template <typename T_Point>
int DecoderRS128<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  RS128DifopPkt* dpkt_ptr = (RS128DifopPkt*)pkt;
  if (dpkt_ptr->id != RS128_DIFOP_ID)
  {
    return RSDecoderResult::DECODE_FAIL;
  }

  this->rpm_ = RS_SWAP_SHORT(dpkt_ptr->rpm);

  if (dpkt_ptr->return_mode == 0x01 || dpkt_ptr->return_mode == 0x02)
  {
    this->echo_mode_ = dpkt_ptr->return_mode;
  }
  else
  {
    this->echo_mode_ = ECHO_DUAL;
  }

  if (this->echo_mode_ == ECHO_DUAL)
  {
    this->pkts_per_frame_ = ceil(2 * RS128_PKT_RATE * 60 / this->rpm_);
  }
  else
  {
    this->pkts_per_frame_ = ceil(RS128_PKT_RATE * 60 / this->rpm_);
  }

  if (!this->difop_flag_)
  {
    bool angle_flag = true;
    const uint8_t* p_ver_cali;
    p_ver_cali = (uint8_t*)(dpkt_ptr->ver_angle_cali);
    if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) && (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
        (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) && (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF))
    {
      angle_flag = false;
    }

    if (angle_flag)
    {
      int lsb, mid, msb, neg = 1;
      for (int i = 0; i < 128; i++)
      {
        // calculation of vertical angle
        lsb = dpkt_ptr->ver_angle_cali[i].sign;
        mid = dpkt_ptr->ver_angle_cali[i].value[0];
        msb = dpkt_ptr->ver_angle_cali[i].value[1];
        if (lsb == 0)
        {
          neg = 1;
        }
        else
        {
          neg = -1;
        }
        this->vert_angle_list_[i] = (mid * 256 + msb) * neg;  // * 0.01f;

        // horizontal offset angle
        lsb = dpkt_ptr->hori_angle_cali[i].sign;
        mid = dpkt_ptr->hori_angle_cali[i].value[0];
        msb = dpkt_ptr->hori_angle_cali[i].value[1];
        if (lsb == 0)
        {
          neg = 1;
        }
        else
        {
          neg = -1;
        }
        this->hori_angle_list_[i] = (mid * 256 + msb) * neg;  // * 0.01f;
      }
      this->difop_flag_ = true;
    }
  }
  return 0;
}

}  // namespace lidar
}  // namespace robosense