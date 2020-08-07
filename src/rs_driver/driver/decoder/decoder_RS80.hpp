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
#define RS80_MSOP_ID (0x5A05AA55)           // big endian
#define RS80_DIFOP_ID (0x555511115A00FFA5)  // big endian
#define RS80_BLOCK_ID (0xFE)
#define RS80_BLOCKS_PER_PKT (4)
#define RS80_CHANNELS_PER_BLOCK (80)
#define RS80_DSR_TOFFSET (3.23)
#define RS80_BLOCK_TDURATION (55.55)
const int RS80_PKT_RATE = 4500;
const double RS80_RX = 0.03615;
const double RS80_RY = -0.017;
const double RS80_RZ = 0;

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif
typedef struct
{
  uint8_t id;
  uint8_t ret_id;
  uint16_t azimuth;
  RSChannel channels[RS80_CHANNELS_PER_BLOCK];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS80MsopBlock;

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
RS80MsopHeader;

typedef struct
{
  RS80MsopHeader header;
  RS80MsopBlock blocks[RS80_BLOCKS_PER_PKT];
  uint8_t reserved[188];
  unsigned int index;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS80MsopPkt;

typedef struct
{
  uint8_t sync_mode;
  uint8_t sync_sts;
  RSTimestamp timestamp;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS80TimeInfo;

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
RS80EthNet;

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
RS80Version;

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
  RSCorAngle ver_angle_cali[128];
  RSCorAngle hori_angle_cali[128];
  uint8_t reserved_3[10];
  uint16_t tail;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RS80DifopPkt;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

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
  void initBeamTable();

private:
  std::array<int, 80> beam_table_;
};

template <typename T_Point>
DecoderRS80<T_Point>::DecoderRS80(const RSDecoderParam& param) : DecoderBase<T_Point>(param)
{
  this->angle_file_index_ = 128;
  if (this->param_.max_distance > 230.0f)
  {
    this->param_.max_distance = 230.0f;
  }
  if (this->param_.min_distance < 1.0f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 1.0f;
  }

  initBeamTable();
}

template <typename T_Point>
double DecoderRS80<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeUTC<RS80MsopPkt>(pkt);
}

template <typename T_Point>
int DecoderRS80<T_Point>::azimuthCalibration(const float& azimuth, const int& channel)
{
  return ((int)(azimuth + this->hori_angle_list_[beam_table_[channel]]) + 36000) % 36000;
}

template <typename T_Point>
RSDecoderResult DecoderRS80<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                    int& azimuth)
{
  height = 80;
  RS80MsopPkt* mpkt_ptr = (RS80MsopPkt*)pkt;
  if (mpkt_ptr->header.id != RS80_MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  float azimuth_corrected_float;
  this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_low, mpkt_ptr->header.temp_high);
  int first_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  int second_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[1].azimuth);
  int third_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[2].azimuth);
  int forth_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[3].azimuth);
  azimuth = first_azimuth;
  if (this->trigger_flag_)
  {
    double timestamp = 0;
    if (this->param_.use_lidar_clock)
    {
      timestamp = getLidarTime(pkt);
    }
    else
    {
      timestamp = getTime();
    }
    this->checkTriggerAngle(first_azimuth, timestamp);
  }
  for (int blk_idx = 0; blk_idx < RS80_BLOCKS_PER_PKT; blk_idx++)
  {
    int cur_azimuith = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    if (mpkt_ptr->blocks[blk_idx].id != RS80_BLOCK_ID)
    {
      break;
    }
    float azimuth_diff = 0;
    if (this->echo_mode_ == ECHO_DUAL)
    {
      azimuth_diff = (float)((36000 + third_azimuth - first_azimuth) % 36000);
    }
    else
    {
      switch (blk_idx)
      {
        case 0:
          azimuth_diff = (float)((36000 + second_azimuth - first_azimuth) % 36000);
          break;
        case 1:
          azimuth_diff = (float)((36000 + third_azimuth - second_azimuth) % 36000);
          break;
        case 2:
        case 3:
          azimuth_diff = (float)((36000 + forth_azimuth - third_azimuth) % 36000);
          break;
      }
    }

    for (int channel_idx = 0; channel_idx < RS80_CHANNELS_PER_BLOCK; channel_idx++)
    {
      int dsr_temp = (channel_idx / 4) % 16;

      azimuth_corrected_float = cur_azimuith + (azimuth_diff * (dsr_temp * RS80_DSR_TOFFSET) / RS80_BLOCK_TDURATION);
      int azimuth_final = this->azimuthCalibration(azimuth_corrected_float, channel_idx);

      int distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance);
      float distance_cali = distance * RS_RESOLUTION;

      int angle_horiz_ori = (int)(azimuth_corrected_float + 36000) % 36000;
      int angle_vert = (((int)(this->vert_angle_list_[beam_table_[channel_idx]]) % 36000) + 36000) % 36000;

      T_Point point;
      if ((distance_cali <= this->param_.max_distance && distance_cali >= this->param_.min_distance) &&
          ((this->angle_flag_ && azimuth_final >= this->start_angle_ && azimuth_final <= this->end_angle_) ||
           (!this->angle_flag_ && ((azimuth_final >= this->start_angle_) || (azimuth_final <= this->end_angle_)))))
      {
        point.x = distance_cali * this->cos_lookup_table_[angle_vert] * this->cos_lookup_table_[azimuth_final] +
                  RS80_RX * this->cos_lookup_table_[angle_horiz_ori];
        point.y = -distance_cali * this->cos_lookup_table_[angle_vert] * this->sin_lookup_table_[azimuth_final] -
                  RS80_RX * this->sin_lookup_table_[angle_horiz_ori];
        point.z = distance_cali * this->sin_lookup_table_[angle_vert] + RS80_RZ;
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

  int pkt_rate = 6000;
  this->echo_mode_ = (RSEchoMode)dpkt_ptr->return_mode;

  if (this->echo_mode_ == ECHO_DUAL)
  {
    pkt_rate = pkt_rate * 2;
  }
  this->pkts_per_frame_ = ceil(pkt_rate * 60 / RS_SWAP_SHORT(dpkt_ptr->rpm));

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
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
void DecoderRS80<T_Point>::initBeamTable()
{
  beam_table_[0] = 0;
  beam_table_[1] = 1;
  beam_table_[2] = 2;
  beam_table_[3] = 5;
  beam_table_[4] = 6;
  beam_table_[5] = 8;
  beam_table_[6] = 9;
  beam_table_[7] = 10;
  beam_table_[8] = 11;
  beam_table_[9] = 12;
  beam_table_[10] = 14;
  beam_table_[11] = 15;
  beam_table_[12] = 16;
  beam_table_[13] = 18;
  beam_table_[14] = 19;
  beam_table_[15] = 20;
  beam_table_[16] = 22;
  beam_table_[17] = 23;
  beam_table_[18] = 24;
  beam_table_[19] = 26;
  beam_table_[20] = 27;
  beam_table_[21] = 28;
  beam_table_[22] = 30;
  beam_table_[23] = 32;
  beam_table_[24] = 35;
  beam_table_[25] = 36;
  beam_table_[26] = 39;
  beam_table_[27] = 40;
  beam_table_[28] = 41;
  beam_table_[29] = 44;
  beam_table_[30] = 45;
  beam_table_[31] = 49;
  beam_table_[32] = 50;
  beam_table_[33] = 53;
  beam_table_[34] = 54;
  beam_table_[35] = 58;
  beam_table_[36] = 59;
  beam_table_[37] = 60;
  beam_table_[38] = 62;
  beam_table_[39] = 63;
  beam_table_[40] = 64;
  beam_table_[41] = 65;
  beam_table_[42] = 66;
  beam_table_[43] = 68;
  beam_table_[44] = 69;
  beam_table_[45] = 70;
  beam_table_[46] = 72;
  beam_table_[47] = 73;
  beam_table_[48] = 74;
  beam_table_[49] = 76;
  beam_table_[50] = 78;
  beam_table_[51] = 80;
  beam_table_[52] = 81;
  beam_table_[53] = 82;
  beam_table_[54] = 83;
  beam_table_[55] = 84;
  beam_table_[56] = 87;
  beam_table_[57] = 88;
  beam_table_[58] = 90;
  beam_table_[59] = 91;
  beam_table_[60] = 92;
  beam_table_[61] = 93;
  beam_table_[62] = 94;
  beam_table_[63] = 96;
  beam_table_[64] = 99;
  beam_table_[65] = 100;
  beam_table_[66] = 103;
  beam_table_[67] = 104;
  beam_table_[68] = 105;
  beam_table_[69] = 108;
  beam_table_[70] = 109;
  beam_table_[71] = 110;
  beam_table_[72] = 113;
  beam_table_[73] = 114;
  beam_table_[74] = 118;
  beam_table_[75] = 122;
  beam_table_[76] = 123;
  beam_table_[77] = 124;
  beam_table_[78] = 126;
  beam_table_[79] = 127;
}

}  // namespace lidar
}  // namespace robosense