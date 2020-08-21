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
#define RS80_MSOP_ID (0x5A05AA55)
#define RS80_DIFOP_ID (0x555511115A00FFA5)
#define RS80_BLOCK_ID (0xFE)
#define RS80_BLOCKS_PER_PKT (4)
#define RS80_CHANNELS_PER_BLOCK (80)
#define RS80_DSR_TOFFSET (3.23)
#define RS80_BLOCK_TDURATION (55.55)
const int RS80_PKT_RATE = 4500;
const double RS80_RX = 0.03615;
const double RS80_RY = -0.017;
const double RS80_RZ = 0;

#pragma pack(push, 1)

typedef struct
{
  uint8_t id;
  uint8_t ret_id;
  uint16_t azimuth;
  RSChannel channels[RS80_CHANNELS_PER_BLOCK];
} RS80MsopBlock;

typedef struct
{
  unsigned int id;
  uint8_t reserved1[3];
  uint8_t wave_mode;
  uint8_t temp_low;
  uint8_t temp_high;
  RSTimestampUTC timestamp_utc;
  uint8_t reserved2[10];
  uint8_t lidar_type;
  uint8_t reserved3[49];
} RS80MsopHeader;

typedef struct
{
  RS80MsopHeader header;
  RS80MsopBlock blocks[RS80_BLOCKS_PER_PKT];
  uint8_t reserved[188];
  unsigned int index;
} RS80MsopPkt;

typedef struct
{
  uint8_t sync_mode;
  uint8_t sync_sts;
  RSTimestamp timestamp;
} RS80TimeInfo;

typedef struct
{
  uint8_t lidar_ip[4];
  uint8_t dest_ip[4];
  uint8_t mac_addr[6];
  uint16_t msop_port;
  uint16_t reserve_1;
  uint16_t difop_port;
  uint16_t reserve_2;
} RS80EthNet;

typedef struct
{
  uint8_t top_firmware_ver[5];
  uint8_t bot_firmware_ver[5];
  uint8_t bot_soft_ver[5];
  uint8_t motor_firmware_ver[5];
  uint8_t hw_ver[3];
} RS80Version;

typedef struct
{
  uint64_t id;
  uint16_t rpm;
  RS80EthNet eth;
  RSFOV fov;
  uint16_t reserved_0;
  uint16_t phase_lock_angle;
  RS80Version version;
  uint8_t reserved_1[229];
  RSSn sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  RS80TimeInfo time_info;
  RSStatus status;
  uint8_t reserved_2[5];
  RSDiagno diagno;
  uint8_t gprmc[86];
  RSCalibrationAngle ver_angle_cali[128];
  RSCalibrationAngle hori_angle_cali[128];
  uint8_t reserved_3[10];
  uint16_t tail;
} RS80DifopPkt;

#pragma pack(pop)

template <typename T_Point>
class DecoderRS80 : public DecoderBase<T_Point>
{
public:
  explicit DecoderRS80(const RSDecoderParam& param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);
};

template <typename T_Point>
DecoderRS80<T_Point>::DecoderRS80(const RSDecoderParam& param) : DecoderBase<T_Point>(param)
{
  this->lasers_num_ = 80;
  this->vert_angle_list_.resize(this->lasers_num_);
  this->hori_angle_list_.resize(this->lasers_num_);
  this->beam_ring_table_.resize(this->lasers_num_);
  if (this->param_.max_distance > 230.0f)
  {
    this->param_.max_distance = 230.0f;
  }
  if (this->param_.min_distance < 1.0f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 1.0f;
  }
}

template <typename T_Point>
double DecoderRS80<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeUTC<RS80MsopPkt>(pkt);
}

template <typename T_Point>
RSDecoderResult DecoderRS80<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                    int& azimuth)
{
  height = this->lasers_num_;
  RS80MsopPkt* mpkt_ptr = (RS80MsopPkt*)pkt;
  if (mpkt_ptr->header.id != RS80_MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  double block_timestamp = 0;
  if (HAS_MEMBER(T_Point, timestamp))
  {
    if (this->param_.use_lidar_clock)
    {
      block_timestamp = getLidarTime(pkt);
    }
    else
    {
      block_timestamp = getTime();
    }
  }
  this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_low, mpkt_ptr->header.temp_high);
  azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  if (this->trigger_flag_)
  {
    if (this->param_.use_lidar_clock)
    {
      this->checkTriggerAngle(azimuth, getLidarTime(pkt));
    }
    else
    {
      this->checkTriggerAngle(azimuth, getTime());
    }
  }
  float azi_diff = 0;
  for (size_t blk_idx = 0; blk_idx < RS80_BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != RS80_BLOCK_ID)
    {
      break;
    }
    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    if (this->echo_mode_ == ECHO_DUAL)
    {
      if (blk_idx % 2 == 0)
      {
        if (blk_idx == 0)
        {
          azi_diff =
              (float)((RS_ONE_ROUND + RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 2].azimuth) - cur_azi) % RS_ONE_ROUND);
        }
        else
        {
          azi_diff =
              (float)((RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 2].azimuth)) % RS_ONE_ROUND);
        }
        block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                             (block_timestamp + this->time_duration_between_blocks_);
      }
    }
    else
    {
      if (blk_idx == 0)
      {
        azi_diff =
            (float)((RS_ONE_ROUND + RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth) - cur_azi) % RS_ONE_ROUND);
      }
      else
      {
        azi_diff =
            (float)((RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth)) % RS_ONE_ROUND);
        block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                             (block_timestamp + this->time_duration_between_blocks_);
      }
    }
    azi_diff = (azi_diff > 100) ? this->azi_diff_between_block_theoretical_ : azi_diff;
    for (size_t channel_idx = 0; channel_idx < RS80_CHANNELS_PER_BLOCK; channel_idx++)
    {
      int dsr_temp = (channel_idx / 4) % 16;
      float azi_channel_ori = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth) +
                              (azi_diff * (dsr_temp * RS80_DSR_TOFFSET) / RS80_BLOCK_TDURATION);
      int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx);
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * RS_RESOLUTION;
      int angle_horiz = (int)(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert = (((this->vert_angle_list_[channel_idx]) % RS_ONE_ROUND) + RS_ONE_ROUND) % RS_ONE_ROUND;

      T_Point point;
      if ((distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
          ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
           (!this->angle_flag_ &&
            ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))
      {
        double x = distance * this->cos_lookup_table_[angle_vert] * this->cos_lookup_table_[azi_channel_final] +
                   RS80_RX * this->cos_lookup_table_[angle_horiz];
        double y = -distance * this->cos_lookup_table_[angle_vert] * this->sin_lookup_table_[azi_channel_final] -
                   RS80_RX * this->sin_lookup_table_[angle_horiz];
        double z = distance * this->sin_lookup_table_[angle_vert] + RS80_RZ;
        double intensity = mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
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
      setRing(point, this->beam_ring_table_[channel_idx]);
      setTimestamp(point, block_timestamp);
      vec.emplace_back(std::move(point));
    }
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
RSDecoderResult DecoderRS80<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  RS80DifopPkt* dpkt_ptr = (RS80DifopPkt*)pkt;
  if (dpkt_ptr->id != RS80_DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  this->echo_mode_ = (RSEchoMode)dpkt_ptr->return_mode;
  this->rpm_ = RS_SWAP_SHORT(dpkt_ptr->rpm);
  this->time_duration_between_blocks_ =
      (60 / (float)this->rpm_) / ((RS80_PKT_RATE * 60 / this->rpm_) * RS80_BLOCKS_PER_PKT);
  int fov_start_angle = RS_SWAP_SHORT(dpkt_ptr->fov.start_angle);
  int fov_end_angle = RS_SWAP_SHORT(dpkt_ptr->fov.end_angle);
  int fov_range = (fov_start_angle < fov_end_angle) ? (fov_end_angle - fov_start_angle) :
                                                      (RS_ONE_ROUND - fov_start_angle + fov_end_angle);
  int blocks_per_round = (RS80_PKT_RATE / (this->rpm_ / 60)) * RS80_BLOCKS_PER_PKT;
  this->fov_time_jump_diff_ =
      this->time_duration_between_blocks_ * (fov_range / (RS_ONE_ROUND / (float)blocks_per_round));
  if (this->echo_mode_ == ECHO_DUAL)
  {
    this->pkts_per_frame_ = ceil(2 * RS80_PKT_RATE * 60 / this->rpm_);
  }
  else
  {
    this->pkts_per_frame_ = ceil(RS80_PKT_RATE * 60 / this->rpm_);
  }
  this->azi_diff_between_block_theoretical_ =
      (RS_ONE_ROUND / RS80_BLOCKS_PER_PKT) / (float)this->pkts_per_frame_;  ///< ((rpm/60)*360)/pkts_rate/blocks_per_pkt
  if (!this->difop_flag_)
  {
    const uint8_t* p_ver_cali = (uint8_t*)(dpkt_ptr->ver_angle_cali);
    if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) && (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
        (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) && (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF))
    {
      return RSDecoderResult::DECODE_OK;
    }
    int lsb, mid, msb, neg = 1;
    for (size_t i = 0; i < RS80_CHANNELS_PER_BLOCK; i++)
    {
      // calculation of vertical angle
      lsb = dpkt_ptr->ver_angle_cali[i].sign;
      mid = dpkt_ptr->ver_angle_cali[i].value[0];
      msb = dpkt_ptr->ver_angle_cali[i].value[1];
      neg = lsb == 0 ? 1 : -1;
      this->vert_angle_list_[i] = (mid * 256 + msb) * neg;  // * 0.01f;

      // horizontal offset angle
      lsb = dpkt_ptr->hori_angle_cali[i].sign;
      mid = dpkt_ptr->hori_angle_cali[i].value[0];
      msb = dpkt_ptr->hori_angle_cali[i].value[1];
      neg = lsb == 0 ? 1 : -1;
      this->hori_angle_list_[i] = (mid * 256 + msb) * neg;  // * 0.01f;
    }
    this-> sortBeamTable();
    this->difop_flag_ = true;
  }
  return RSDecoderResult::DECODE_OK;
}

}  // namespace lidar
}  // namespace robosense