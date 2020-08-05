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
#define RS80_MSOP_ID (0x5A05AA55)  // big endian
#define RS80_BLOCK_ID (0xFE)
#define RS80_DIFOP_ID (0x555511115A00FFA5)  // big endian
#define RS80_CHANNELS_PER_BLOCK (80)
#define RS80_BLOCKS_PER_PKT (4)
#define RS80_TEMPERATURE_MIN (31)
#define RS80_TEMPERATURE_RANGE (50)
#define RS80_DSR_TOFFSET (3.23)
#define RS80_BLOCK_TDURATION (55.55)

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
RS80MsopHeader;

typedef struct
{
  RS80MsopHeader header;
  RS80MsopBlock blocks[RS80_BLOCKS_PER_PKT];
  uint8_t reserved[188];
  uint32_t index;
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

template <typename vpoint>
class DecoderRS80 : public DecoderBase<vpoint>
{
public:
  DecoderRS80(const RSDecoderParam& param);
  int32_t decodeDifopPkt(const uint8_t* pkt);
  int32_t decodeMsopPkt(const uint8_t* pkt, std::vector<vpoint>& vec, int& height);
  double getLidarTime(const uint8_t* pkt);
  float computeTemperature(const uint8_t temp_low, const uint8_t temp_high);
};

template <typename vpoint>
DecoderRS80<vpoint>::DecoderRS80(const RSDecoderParam& param) : DecoderBase<vpoint>(param)
{
  this->Rx_ = 0.03615;
  this->Ry_ = -0.017;
  this->Rz_ = 0;
  this->beam_num_ = 80;

  if (this->max_distance_ > 250.0f)
  {
    this->max_distance_ = 250.0f;
  }
  if (this->min_distance_ < 1.0f || this->min_distance_ > this->max_distance_)
  {
    this->min_distance_ = 1.0f;
  }

  int pkt_rate = 6000;
  this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);
}

template <typename vpoint>
double DecoderRS80<vpoint>::getLidarTime(const uint8_t* pkt)
{
  RS80MsopPkt* mpkt_ptr = (RS80MsopPkt*)pkt;
  union u_ts
  {
    uint8_t data[8];
    uint64_t ts;
  } t;

  t.data[7] = 0;
  t.data[6] = 0;
  t.data[5] = mpkt_ptr->header.timestamp_utc.sec[0];
  t.data[4] = mpkt_ptr->header.timestamp_utc.sec[1];
  t.data[3] = mpkt_ptr->header.timestamp_utc.sec[2];
  t.data[2] = mpkt_ptr->header.timestamp_utc.sec[3];
  t.data[1] = mpkt_ptr->header.timestamp_utc.sec[4];
  t.data[0] = mpkt_ptr->header.timestamp_utc.sec[5];
  return (double)t.ts + ((double)(RS_SWAP_LONG(mpkt_ptr->header.timestamp_utc.ns))) / 1000000000.0d;
}

template <typename vpoint>
float DecoderRS80<vpoint>::computeTemperature(const uint8_t temp_low, const uint8_t temp_high)
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

template <typename vpoint>
int DecoderRS80<vpoint>::decodeMsopPkt(const uint8_t* pkt, std::vector<vpoint>& vec, int& height)
{
  height = 80;
  RS80MsopPkt* mpkt_ptr = (RS80MsopPkt*)pkt;
  if (mpkt_ptr->header.id != RS80_MSOP_ID)
  {
    //      rs_print(RS_ERROR, "[RS80] MSOP pkt ID no match.");
    return -2;
  }

  float azimuth_corrected_float;
  this->current_temperature_ = computeTemperature(mpkt_ptr->header.temp_low, mpkt_ptr->header.temp_high);
  int first_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  int second_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[1].azimuth);
  int third_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[2].azimuth);
  int forth_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[3].azimuth);

  if (this->trigger_flag_)
  {
    double timestamp = 0;
    if (this->use_lidar_clock_)
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
      int angle_vert = (((int)(this->vert_angle_list_[channel_idx]) % 36000) + 36000) % 36000;


      vpoint point;
      if ((distance_cali <= this->max_distance_ && distance_cali >= this->min_distance_) &&
          ((this->angle_flag_ && azimuth_final >= this->start_angle_ && azimuth_final <= this->end_angle_) ||
           (!this->angle_flag_ && ((azimuth_final >= this->start_angle_) || (azimuth_final <= this->end_angle_)))))
      {
        point.x = distance_cali * this->cos_lookup_table_[angle_vert] * this->cos_lookup_table_[azimuth_final] +
                  this->Rx_ * this->cos_lookup_table_[angle_horiz_ori];
        point.y = -distance_cali * this->cos_lookup_table_[angle_vert] * this->sin_lookup_table_[azimuth_final] -
                  this->Rx_ * this->sin_lookup_table_[angle_horiz_ori];
        point.z = distance_cali * this->sin_lookup_table_[angle_vert] + this->Rz_;
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

template <typename vpoint>
int DecoderRS80<vpoint>::decodeDifopPkt(const uint8_t* pkt)
{
  RS80DifopPkt* difop_ptr = (RS80DifopPkt*)pkt;
  if (difop_ptr->id != RS80_DIFOP_ID)
  {
    //		rs_print(RS_ERROR, "[RS80] DIFOP pkt ID no match.");
    return -2;
  }

  int pkt_rate = 6000;
  this->rpm_ = RS_SWAP_SHORT(difop_ptr->rpm);

  if (difop_ptr->return_mode == 0x01 || difop_ptr->return_mode == 0x02)
  {  // 1,2: single echo
    this->echo_mode_ = difop_ptr->return_mode;
  }
  else
  {  // 3: dual echo
    this->echo_mode_ = ECHO_DUAL;
  }

  if (this->echo_mode_ == ECHO_DUAL)
  {
    pkt_rate = pkt_rate * 2;
  }
  this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);

  if (!this->difop_flag_)
  {
    bool angle_flag = true;
    const uint8_t* p_ver_cali;
    p_ver_cali = (uint8_t*)(difop_ptr->ver_angle_cali);
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
        lsb = difop_ptr->ver_angle_cali[i].sign;
        mid = difop_ptr->ver_angle_cali[i].value[0];
        msb = difop_ptr->ver_angle_cali[i].value[1];
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
        lsb = difop_ptr->hori_angle_cali[i].sign;
        mid = difop_ptr->hori_angle_cali[i].value[0];
        msb = difop_ptr->hori_angle_cali[i].value[1];
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