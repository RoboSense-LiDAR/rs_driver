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
#define RS128_MSOP_ID (0x5A05AA55)
#define RS128_DIFOP_ID (0x555511115A00FFA5)
#define RS128_BLOCK_ID (0xFE)
#define RS128_BLOCKS_PER_PKT (3)
#define RS128_CHANNELS_PER_BLOCK (128)
#define RS128_DSR_TOFFSET (3.23)
#define RS128_BLOCK_TDURATION (55.55)
const int RS128_PKT_RATE = 6000;
const double RS128_RX = 0.03615;
const double RS128_RY = -0.017;
const double RS128_RZ = 0;

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
  unsigned int id;
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
  unsigned int index;
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
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);

private:
  void initTable();

private:
  std::array<int, 128> beam_ring_table_;
};

template <typename T_Point>
DecoderRS128<T_Point>::DecoderRS128(const RSDecoderParam& param) : DecoderBase<T_Point>(param)
{
  this->angle_file_index_ = 128;
  if (this->param_.max_distance > 250.0f)
  {
    this->param_.max_distance = 250.0f;
  }
  if (this->param_.min_distance < 1.0f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 1.0f;
  }
  initTable();
}

template <typename T_Point>
double DecoderRS128<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeUTC<RS128MsopPkt>(pkt);
}

template <typename T_Point>
RSDecoderResult DecoderRS128<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                     int& azimuth)
{
  height = RS128_CHANNELS_PER_BLOCK;
  RS128MsopPkt* mpkt_ptr = (RS128MsopPkt*)pkt;
  if (mpkt_ptr->header.id != RS128_MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  double pkt_timestamp = 0;
  if (HAS_MEMBER(T_Point, timestamp))
  {
    if (this->param_.use_lidar_clock)
    {
      pkt_timestamp = getLidarTime(pkt);
    }
    else
    {
      pkt_timestamp = getTime();
    }
  }
  this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_low, mpkt_ptr->header.temp_high);
  int first_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  int second_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[1].azimuth);
  int third_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[2].azimuth);
  azimuth = first_azimuth;
  if (this->trigger_flag_)
  {
    if (this->param_.use_lidar_clock)
    {
      this->checkTriggerAngle(first_azimuth, getLidarTime(pkt));
    }
    else
    {
      this->checkTriggerAngle(first_azimuth, getTime());
    }
  }
  for (size_t blk_idx = 0; blk_idx < RS128_BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != RS128_BLOCK_ID)
    {
      break;
    }
    float azi_diff_theoretical =
        (360 / RS128_BLOCKS_PER_PKT) * 100 / (float)this->pkts_per_frame_;  ///< ((rpm/60)*360)/pkts_rate/blocks_per_pkt
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
    azi_diff = (azi_diff > 100) ? azi_diff_theoretical : azi_diff;
    if (HAS_MEMBER(T_Point, timestamp))
    {
      pkt_timestamp += this->time_duration_between_blocks_ * blk_idx;
    }
    for (size_t channel_idx = 0; channel_idx < RS128_CHANNELS_PER_BLOCK; channel_idx++)
    {
      int dsr_temp = (channel_idx / 4) % 16;
      float azi_channel_ori = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth) +
                              (azi_diff * (dsr_temp * RS128_DSR_TOFFSET) / RS128_BLOCK_TDURATION);
      int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx);
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * RS_RESOLUTION;
      int angle_horiz = (int)(azi_channel_ori + 36000) % 36000;
      int angle_vert = (((int)(this->vert_angle_list_[channel_idx]) % 36000) + 36000) % 36000;

      T_Point point;
      if ((distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
          ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
           (!this->angle_flag_ &&
            ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))
      {
        double x = distance * this->cos_lookup_table_[angle_vert] * this->cos_lookup_table_[azi_channel_final] +
                   RS128_RX * this->cos_lookup_table_[angle_horiz];

        double y = -distance * this->cos_lookup_table_[angle_vert] * this->sin_lookup_table_[azi_channel_final] -
                   RS128_RX * this->sin_lookup_table_[angle_horiz];
        double z = distance * this->sin_lookup_table_[angle_vert] + RS128_RZ;
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
      setTimestamp(point, pkt_timestamp);
      vec.emplace_back(std::move(point));
    }
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
RSDecoderResult DecoderRS128<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  RS128DifopPkt* dpkt_ptr = (RS128DifopPkt*)pkt;
  if (dpkt_ptr->id != RS128_DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  this->rpm_ = RS_SWAP_SHORT(dpkt_ptr->rpm);
  this->time_duration_between_blocks_ =
      (60 / (float)this->rpm_) / ((RS128_PKT_RATE * 60 / this->rpm_) * RS128_BLOCKS_PER_PKT);
  this->echo_mode_ = (RSEchoMode)dpkt_ptr->return_mode;
  if (this->echo_mode_ == RSEchoMode::ECHO_DUAL)
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
      for (size_t i = 0; i < this->angle_file_index_; i++)
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
void DecoderRS128<T_Point>::initTable()
{
  beam_ring_table_[0] = 3;
  beam_ring_table_[1] = 66;
  beam_ring_table_[2] = 33;
  beam_ring_table_[3] = 96;
  beam_ring_table_[4] = 11;
  beam_ring_table_[5] = 74;
  beam_ring_table_[6] = 41;
  beam_ring_table_[7] = 104;
  beam_ring_table_[8] = 19;
  beam_ring_table_[9] = 82;
  beam_ring_table_[10] = 49;
  beam_ring_table_[11] = 112;
  beam_ring_table_[12] = 27;
  beam_ring_table_[13] = 90;
  beam_ring_table_[14] = 57;
  beam_ring_table_[15] = 120;
  beam_ring_table_[16] = 35;
  beam_ring_table_[17] = 98;
  beam_ring_table_[18] = 1;
  beam_ring_table_[19] = 64;
  beam_ring_table_[20] = 43;
  beam_ring_table_[21] = 106;
  beam_ring_table_[22] = 9;
  beam_ring_table_[23] = 72;
  beam_ring_table_[24] = 51;
  beam_ring_table_[25] = 114;
  beam_ring_table_[26] = 17;
  beam_ring_table_[27] = 80;
  beam_ring_table_[28] = 59;
  beam_ring_table_[29] = 122;
  beam_ring_table_[30] = 25;
  beam_ring_table_[31] = 88;
  beam_ring_table_[32] = 67;
  beam_ring_table_[33] = 34;
  beam_ring_table_[34] = 97;
  beam_ring_table_[35] = 0;
  beam_ring_table_[36] = 75;
  beam_ring_table_[37] = 42;
  beam_ring_table_[38] = 105;
  beam_ring_table_[39] = 8;
  beam_ring_table_[40] = 83;
  beam_ring_table_[41] = 50;
  beam_ring_table_[42] = 113;
  beam_ring_table_[43] = 16;
  beam_ring_table_[44] = 91;
  beam_ring_table_[45] = 58;
  beam_ring_table_[46] = 121;
  beam_ring_table_[47] = 24;
  beam_ring_table_[48] = 99;
  beam_ring_table_[49] = 2;
  beam_ring_table_[50] = 65;
  beam_ring_table_[51] = 32;
  beam_ring_table_[52] = 107;
  beam_ring_table_[53] = 10;
  beam_ring_table_[54] = 73;
  beam_ring_table_[55] = 40;
  beam_ring_table_[56] = 115;
  beam_ring_table_[57] = 18;
  beam_ring_table_[58] = 81;
  beam_ring_table_[59] = 48;
  beam_ring_table_[60] = 123;
  beam_ring_table_[61] = 26;
  beam_ring_table_[62] = 89;
  beam_ring_table_[63] = 56;
  beam_ring_table_[64] = 7;
  beam_ring_table_[65] = 70;
  beam_ring_table_[66] = 37;
  beam_ring_table_[67] = 100;
  beam_ring_table_[68] = 15;
  beam_ring_table_[69] = 78;
  beam_ring_table_[70] = 45;
  beam_ring_table_[71] = 108;
  beam_ring_table_[72] = 23;
  beam_ring_table_[73] = 86;
  beam_ring_table_[74] = 53;
  beam_ring_table_[75] = 116;
  beam_ring_table_[76] = 31;
  beam_ring_table_[77] = 94;
  beam_ring_table_[78] = 61;
  beam_ring_table_[79] = 124;
  beam_ring_table_[80] = 39;
  beam_ring_table_[81] = 102;
  beam_ring_table_[82] = 5;
  beam_ring_table_[83] = 68;
  beam_ring_table_[84] = 47;
  beam_ring_table_[85] = 110;
  beam_ring_table_[86] = 13;
  beam_ring_table_[87] = 76;
  beam_ring_table_[88] = 55;
  beam_ring_table_[89] = 118;
  beam_ring_table_[90] = 21;
  beam_ring_table_[91] = 84;
  beam_ring_table_[92] = 63;
  beam_ring_table_[93] = 126;
  beam_ring_table_[94] = 29;
  beam_ring_table_[95] = 92;
  beam_ring_table_[96] = 71;
  beam_ring_table_[97] = 38;
  beam_ring_table_[98] = 101;
  beam_ring_table_[99] = 4;
  beam_ring_table_[100] = 79;
  beam_ring_table_[101] = 46;
  beam_ring_table_[102] = 109;
  beam_ring_table_[103] = 12;
  beam_ring_table_[104] = 87;
  beam_ring_table_[105] = 54;
  beam_ring_table_[106] = 117;
  beam_ring_table_[107] = 20;
  beam_ring_table_[108] = 95;
  beam_ring_table_[109] = 62;
  beam_ring_table_[110] = 125;
  beam_ring_table_[111] = 28;
  beam_ring_table_[112] = 103;
  beam_ring_table_[113] = 6;
  beam_ring_table_[114] = 69;
  beam_ring_table_[115] = 36;
  beam_ring_table_[116] = 111;
  beam_ring_table_[117] = 14;
  beam_ring_table_[118] = 77;
  beam_ring_table_[119] = 44;
  beam_ring_table_[120] = 119;
  beam_ring_table_[121] = 22;
  beam_ring_table_[122] = 85;
  beam_ring_table_[123] = 52;
  beam_ring_table_[124] = 127;
  beam_ring_table_[125] = 30;
  beam_ring_table_[126] = 93;
  beam_ring_table_[127] = 60;
}

}  // namespace lidar
}  // namespace robosense