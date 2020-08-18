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
#define RS16_MSOP_ID (0xA050A55A0A05AA55)
#define RS16_DIFOP_ID (0x555511115A00FFA5)
#define RS16_BLOCK_ID (0xEEFF)
#define RS16_BLOCKS_PER_PKT (12)
#define RS16_CHANNELS_PER_BLOCK (32)
#define RS16_BLOCK_TDURATION_DUAL (50)
#define RS16_BLOCK_TDURATION_SINGLE (100)
#define RS16_CHANNEL_TOFFSET (3)
#define RS16_FIRING_TDURATION (50)

const int RS16_PKT_RATE = 750;
const double RS16_RX = 0.03825;
const double RS16_RY = -0.01088;
const double RS16_RZ = 0;

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif
typedef struct
{
  uint16_t id;
  uint16_t azimuth;
  RSChannel channels[RS16_CHANNELS_PER_BLOCK];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS16MsopBlock;

typedef struct
{
  RSMsopHeader header;
  RS16MsopBlock blocks[RS16_BLOCKS_PER_PKT];
  unsigned int index;
  uint16_t tail;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS16MsopPkt;

typedef struct
{
  uint8_t intensity_cali[240];
  uint8_t coef;
  uint8_t ver;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS16Intensity;

typedef struct
{
  uint64_t id;
  uint16_t rpm;
  RSEthNet eth;
  RSFOV fov;
  uint16_t static_base;
  uint16_t phase_lock_angle;
  RSVersion version;
  RS16Intensity intensity;
  RSSn sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  uint16_t sw_ver;
  RSTimestamp timestamp;
  RSStatus status;
  uint8_t reserved1[5];
  RSDiagno diagno;
  uint8_t gprmc[86];
  uint8_t static_cali[697];
  uint8_t pitch_cali[48];
  uint8_t reserved2[33];
  uint16_t tail;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS16DifopPkt;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

template <typename T_Point>
class DecoderRS16 : public DecoderBase<T_Point>
{
public:
  DecoderRS16(const RSDecoderParam& param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);

private:
  void initTable();

private:
  std::array<int, 128> beam_ring_table_;
};

template <typename T_Point>
DecoderRS16<T_Point>::DecoderRS16(const RSDecoderParam& param) : DecoderBase<T_Point>(param)
{
  this->angle_file_index_ = 16;
  if (this->param_.max_distance > 150.0f)
  {
    this->param_.max_distance = 150.0f;
  }
  if (this->param_.min_distance < 0.2f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 0.2f;
  }
  initTable();
}

template <typename T_Point>
double DecoderRS16<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeYMD<RS16MsopPkt>(pkt);
}

template <typename T_Point>
RSDecoderResult DecoderRS16<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                    int& azimuth)
{
  height = 16;
  RS16MsopPkt* mpkt_ptr = (RS16MsopPkt*)pkt;
  if (mpkt_ptr->header.id != RS16_MSOP_ID)
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
  this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_raw);
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
  for (size_t blk_idx = 0; blk_idx < RS16_BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != RS16_BLOCK_ID)
    {
      break;
    }
    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    float azi_diff_theoretical = (RS_ONE_ROUND / RS16_BLOCKS_PER_PKT) /
                                 (float)this->pkts_per_frame_;  ///< ((rpm/60)*360)/pkts_rate/blocks_per_pkt
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
    azi_diff = (azi_diff > 100) ? azi_diff_theoretical : azi_diff;
    for (size_t channel_idx = 0; channel_idx < RS16_CHANNELS_PER_BLOCK; channel_idx++)
    {
      float azi_channel_ori = 0;
      if (this->echo_mode_ == ECHO_DUAL)
      {
        azi_channel_ori = cur_azi + azi_diff * RS16_CHANNEL_TOFFSET * (channel_idx % 16) / RS16_BLOCK_TDURATION_DUAL;
      }
      else
      {
        azi_channel_ori =
            cur_azi + azi_diff *
                          (RS16_FIRING_TDURATION * (channel_idx / 16) + RS16_CHANNEL_TOFFSET * (channel_idx % 16)) /
                          RS16_BLOCK_TDURATION_SINGLE;
      }
      int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx);
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * RS_RESOLUTION;
      int angle_horiz_ori = (int)(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert = (((int)(this->vert_angle_list_[channel_idx % 16]) % RS_ONE_ROUND) + RS_ONE_ROUND) % RS_ONE_ROUND;
      T_Point point;
      if ((distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
          ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
           (!this->angle_flag_ &&
            ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))
      {
        double x = distance * this->cos_lookup_table_[angle_vert] * this->cos_lookup_table_[azi_channel_final] +
                   RS16_RX * this->cos_lookup_table_[angle_horiz_ori];

        double y = -distance * this->cos_lookup_table_[angle_vert] * this->sin_lookup_table_[azi_channel_final] -
                   RS16_RX * this->sin_lookup_table_[angle_horiz_ori];
        double z = distance * this->sin_lookup_table_[angle_vert] + RS16_RZ;
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
      setRing(point, beam_ring_table_[channel_idx]);
      setTimestamp(point, block_timestamp);
      vec.emplace_back(std::move(point));
    }
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
RSDecoderResult DecoderRS16<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  RS16DifopPkt* dpkt_ptr = (RS16DifopPkt*)pkt;
  if (dpkt_ptr->id != RS16_DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  switch (dpkt_ptr->return_mode)
  {
    case 0x00:
      this->echo_mode_ = RSEchoMode::ECHO_DUAL;
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
  this->rpm_ = RS_SWAP_SHORT(dpkt_ptr->rpm);
  this->time_duration_between_blocks_ =
      (60 / (float)this->rpm_) / ((RS16_PKT_RATE * 60 / this->rpm_) * RS16_BLOCKS_PER_PKT);
  int fov_start_angle = RS_SWAP_SHORT(dpkt_ptr->fov.start_angle);
  int fov_end_angle = RS_SWAP_SHORT(dpkt_ptr->fov.end_angle);
  int fov_range = (fov_start_angle < fov_end_angle) ? (fov_end_angle - fov_start_angle) :
                                                      (RS_ONE_ROUND - fov_start_angle + fov_end_angle);
  int blocks_per_round = (RS16_PKT_RATE / (this->rpm_ / 60)) * RS16_BLOCKS_PER_PKT;
  this->fov_time_jump_diff_ =
      this->time_duration_between_blocks_ * (fov_range / (RS_ONE_ROUND / (float)blocks_per_round));
  if (this->echo_mode_ == RSEchoMode::ECHO_DUAL)
  {
    this->pkts_per_frame_ = ceil(2 * RS16_PKT_RATE * 60 / this->rpm_);
  }
  else
  {
    this->pkts_per_frame_ = ceil(RS16_PKT_RATE * 60 / this->rpm_);
  }
  if (!this->difop_flag_)
  {
    bool angle_flag = true;
    const uint8_t* p_ver_cali;
    p_ver_cali = dpkt_ptr->pitch_cali;
    if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) && (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
        (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) && (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF))
    {
      angle_flag = false;
    }
    if (angle_flag)
    {
      int lsb, mid, msb, neg = 1;

      for (int i = 0; i < 16; i++)
      {
        /* vert angle calibration data */
        lsb = p_ver_cali[i * 3];
        mid = p_ver_cali[i * 3 + 1];
        msb = p_ver_cali[i * 3 + 2];
        if (i < 8)
        {
          neg = -1;
        }
        else
        {
          neg = 1;
        }
        this->vert_angle_list_[i] = (lsb * 256 * 256 + mid * 256 + msb) * neg * 0.01f;  // / 180 * M_PI;
        /* horizon angle calibration data */
        this->hori_angle_list_[i] = 0;
      }
      this->difop_flag_ = true;
    }
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
void DecoderRS16<T_Point>::initTable()
{
  beam_ring_table_[0] = 0;
  beam_ring_table_[1] = 1;
  beam_ring_table_[2] = 2;
  beam_ring_table_[3] = 3;
  beam_ring_table_[4] = 4;
  beam_ring_table_[5] = 5;
  beam_ring_table_[6] = 6;
  beam_ring_table_[7] = 7;
  beam_ring_table_[8] = 15;
  beam_ring_table_[9] = 14;
  beam_ring_table_[10] = 13;
  beam_ring_table_[11] = 12;
  beam_ring_table_[12] = 11;
  beam_ring_table_[13] = 10;
  beam_ring_table_[14] = 9;
  beam_ring_table_[15] = 8;
}

}  // namespace lidar
}  // namespace robosense