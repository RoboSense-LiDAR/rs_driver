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
  DecoderRS80(const RSDecoderParam& param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);

private:
  int azimuthCalibration(const float& azimuth, const int& channel);
  void initTable();

private:
  std::array<int, 80> beam_mapping_table_;
  std::array<int, 80> beam_ring_table_;
};

template <typename T_Point>
DecoderRS80<T_Point>::DecoderRS80(const RSDecoderParam& param) : DecoderBase<T_Point>(param)
{
  this->angle_file_row_num_ = 128;
  this->vert_angle_list_.resize(this->angle_file_row_num_);
  this->hori_angle_list_.resize(this->angle_file_row_num_);
  if (this->param_.max_distance > 230.0f)
  {
    this->param_.max_distance = 230.0f;
  }
  if (this->param_.min_distance < 1.0f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 1.0f;
  }

  initTable();
}

template <typename T_Point>
double DecoderRS80<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeUTC<RS80MsopPkt>(pkt);
}

template <typename T_Point>
int DecoderRS80<T_Point>::azimuthCalibration(const float& azimuth, const int& channel)
{
  return ((int)(azimuth + this->hori_angle_list_[beam_mapping_table_[channel]]) + RS_ONE_ROUND) % RS_ONE_ROUND;
}

template <typename T_Point>
RSDecoderResult DecoderRS80<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                    int& azimuth)
{
  height = RS80_CHANNELS_PER_BLOCK;
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
      int angle_vert =
          (((int)(this->vert_angle_list_[beam_mapping_table_[channel_idx]]) % RS_ONE_ROUND) + RS_ONE_ROUND) %
          RS_ONE_ROUND;

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
      setRing(point, beam_ring_table_[channel_idx]);
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
      for (size_t i = 0; i < this->angle_file_row_num_; i++)
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
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
void DecoderRS80<T_Point>::initTable()
{
  beam_mapping_table_[0] = 0;
  beam_mapping_table_[1] = 1;
  beam_mapping_table_[2] = 2;
  beam_mapping_table_[3] = 5;
  beam_mapping_table_[4] = 6;
  beam_mapping_table_[5] = 8;
  beam_mapping_table_[6] = 9;
  beam_mapping_table_[7] = 10;
  beam_mapping_table_[8] = 11;
  beam_mapping_table_[9] = 12;
  beam_mapping_table_[10] = 14;
  beam_mapping_table_[11] = 15;
  beam_mapping_table_[12] = 16;
  beam_mapping_table_[13] = 18;
  beam_mapping_table_[14] = 19;
  beam_mapping_table_[15] = 20;
  beam_mapping_table_[16] = 22;
  beam_mapping_table_[17] = 23;
  beam_mapping_table_[18] = 24;
  beam_mapping_table_[19] = 26;
  beam_mapping_table_[20] = 27;
  beam_mapping_table_[21] = 28;
  beam_mapping_table_[22] = 30;
  beam_mapping_table_[23] = 32;
  beam_mapping_table_[24] = 35;
  beam_mapping_table_[25] = 36;
  beam_mapping_table_[26] = 39;
  beam_mapping_table_[27] = 40;
  beam_mapping_table_[28] = 41;
  beam_mapping_table_[29] = 44;
  beam_mapping_table_[30] = 45;
  beam_mapping_table_[31] = 49;
  beam_mapping_table_[32] = 50;
  beam_mapping_table_[33] = 53;
  beam_mapping_table_[34] = 54;
  beam_mapping_table_[35] = 58;
  beam_mapping_table_[36] = 59;
  beam_mapping_table_[37] = 60;
  beam_mapping_table_[38] = 62;
  beam_mapping_table_[39] = 63;
  beam_mapping_table_[40] = 64;
  beam_mapping_table_[41] = 65;
  beam_mapping_table_[42] = 66;
  beam_mapping_table_[43] = 68;
  beam_mapping_table_[44] = 69;
  beam_mapping_table_[45] = 70;
  beam_mapping_table_[46] = 72;
  beam_mapping_table_[47] = 73;
  beam_mapping_table_[48] = 74;
  beam_mapping_table_[49] = 76;
  beam_mapping_table_[50] = 78;
  beam_mapping_table_[51] = 80;
  beam_mapping_table_[52] = 81;
  beam_mapping_table_[53] = 82;
  beam_mapping_table_[54] = 83;
  beam_mapping_table_[55] = 84;
  beam_mapping_table_[56] = 87;
  beam_mapping_table_[57] = 88;
  beam_mapping_table_[58] = 90;
  beam_mapping_table_[59] = 91;
  beam_mapping_table_[60] = 92;
  beam_mapping_table_[61] = 93;
  beam_mapping_table_[62] = 94;
  beam_mapping_table_[63] = 96;
  beam_mapping_table_[64] = 99;
  beam_mapping_table_[65] = 100;
  beam_mapping_table_[66] = 103;
  beam_mapping_table_[67] = 104;
  beam_mapping_table_[68] = 105;
  beam_mapping_table_[69] = 108;
  beam_mapping_table_[70] = 109;
  beam_mapping_table_[71] = 110;
  beam_mapping_table_[72] = 113;
  beam_mapping_table_[73] = 114;
  beam_mapping_table_[74] = 118;
  beam_mapping_table_[75] = 122;
  beam_mapping_table_[76] = 123;
  beam_mapping_table_[77] = 124;
  beam_mapping_table_[78] = 126;
  beam_mapping_table_[79] = 127;
  beam_ring_table_[0] = 3;
  beam_ring_table_[1] = 47;
  beam_ring_table_[2] = 21;
  beam_ring_table_[3] = 55;
  beam_ring_table_[4] = 25;
  beam_ring_table_[5] = 14;
  beam_ring_table_[6] = 63;
  beam_ring_table_[7] = 30;
  beam_ring_table_[8] = 74;
  beam_ring_table_[9] = 18;
  beam_ring_table_[10] = 38;
  beam_ring_table_[11] = 75;
  beam_ring_table_[12] = 22;
  beam_ring_table_[13] = 1;
  beam_ring_table_[14] = 45;
  beam_ring_table_[15] = 26;
  beam_ring_table_[16] = 9;
  beam_ring_table_[17] = 53;
  beam_ring_table_[18] = 32;
  beam_ring_table_[19] = 13;
  beam_ring_table_[20] = 61;
  beam_ring_table_[21] = 40;
  beam_ring_table_[22] = 17;
  beam_ring_table_[23] = 48;
  beam_ring_table_[24] = 0;
  beam_ring_table_[25] = 56;
  beam_ring_table_[26] = 8;
  beam_ring_table_[27] = 64;
  beam_ring_table_[28] = 31;
  beam_ring_table_[29] = 70;
  beam_ring_table_[30] = 39;
  beam_ring_table_[31] = 2;
  beam_ring_table_[32] = 46;
  beam_ring_table_[33] = 10;
  beam_ring_table_[34] = 54;
  beam_ring_table_[35] = 62;
  beam_ring_table_[36] = 29;
  beam_ring_table_[37] = 76;
  beam_ring_table_[38] = 69;
  beam_ring_table_[39] = 37;
  beam_ring_table_[40] = 7;
  beam_ring_table_[41] = 51;
  beam_ring_table_[42] = 23;
  beam_ring_table_[43] = 12;
  beam_ring_table_[44] = 59;
  beam_ring_table_[45] = 27;
  beam_ring_table_[46] = 16;
  beam_ring_table_[47] = 67;
  beam_ring_table_[48] = 34;
  beam_ring_table_[49] = 20;
  beam_ring_table_[50] = 42;
  beam_ring_table_[51] = 24;
  beam_ring_table_[52] = 73;
  beam_ring_table_[53] = 5;
  beam_ring_table_[54] = 49;
  beam_ring_table_[55] = 28;
  beam_ring_table_[56] = 57;
  beam_ring_table_[57] = 36;
  beam_ring_table_[58] = 15;
  beam_ring_table_[59] = 65;
  beam_ring_table_[60] = 44;
  beam_ring_table_[61] = 78;
  beam_ring_table_[62] = 19;
  beam_ring_table_[63] = 52;
  beam_ring_table_[64] = 4;
  beam_ring_table_[65] = 60;
  beam_ring_table_[66] = 11;
  beam_ring_table_[67] = 68;
  beam_ring_table_[68] = 35;
  beam_ring_table_[69] = 72;
  beam_ring_table_[70] = 43;
  beam_ring_table_[71] = 77;
  beam_ring_table_[72] = 6;
  beam_ring_table_[73] = 50;
  beam_ring_table_[74] = 58;
  beam_ring_table_[75] = 66;
  beam_ring_table_[76] = 33;
  beam_ring_table_[77] = 79;
  beam_ring_table_[78] = 71;
  beam_ring_table_[79] = 41;
}

}  // namespace lidar
}  // namespace robosense