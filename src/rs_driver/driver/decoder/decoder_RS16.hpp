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
#define RS16_CHANNELS_PER_BLOCK (32)
#define RS16_BLOCKS_PER_PKT (12)
#define RS16_BLOCK_TDURATION_DUAL (50)
#define RS16_BLOCK_TDURATION_SINGLE (100)
#define RS16_POINTS_CHANNEL_PER_SECOND (18000)
#define RS16_BLOCKS_CHANNEL_PER_PKT (12)
#define RS16_MSOP_ID (0xA050A55A0A05AA55)
#define RS16_BLOCK_ID (0xEEFF)
#define RS16_DIFOP_ID (0x555511115A00FFA5)
#define RS16_CHANNEL_TOFFSET (3)
#define RS16_FIRING_TDURATION (50)

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
  uint32_t index;
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
  uint8_t reserved1[11];
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

template <typename vpoint>
class DecoderRS16 : public DecoderBase<vpoint>
{
public:
  DecoderRS16(const RSDecoderParam& param);
  int32_t decodeDifopPkt(const uint8_t* pkt);
  int32_t decodeMsopPkt(const uint8_t* pkt, std::vector<vpoint>& vec, int& height);
  double getLidarTime(const uint8_t* pkt);
};

template <typename vpoint>
DecoderRS16<vpoint>::DecoderRS16(const RSDecoderParam& param) : DecoderBase<vpoint>(param)
{
  this->Rx_ = 0.03825;
  this->Ry_ = -0.01088;
  this->Rz_ = 0;
  this->channel_num_ = 16;
  if (this->max_distance_ > 200.0f)
  {
    this->max_distance_ = 200.0f;
  }
  if (this->min_distance_ < 0.2f || this->min_distance_ > this->max_distance_)
  {
    this->min_distance_ = 0.2f;
  }
  //    rs_print(RS_INFO, "[RS16] Constructor.");
}

template <typename vpoint>
double DecoderRS16<vpoint>::getLidarTime(const uint8_t* pkt)
{
  RS16MsopPkt* mpkt_ptr = (RS16MsopPkt*)pkt;
  std::tm stm;
  memset(&stm, 0, sizeof(stm));
  stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
  stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
  stm.tm_mday = mpkt_ptr->header.timestamp.day;
  stm.tm_hour = mpkt_ptr->header.timestamp.hour;
  stm.tm_min = mpkt_ptr->header.timestamp.minute;
  stm.tm_sec = mpkt_ptr->header.timestamp.second;
  return std::mktime(&stm) + (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms) / 1000.0 +
         (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us) / 1000000.0;
}

template <typename vpoint>
int DecoderRS16<vpoint>::decodeMsopPkt(const uint8_t* pkt, std::vector<vpoint>& vec, int& height)
{
  height = 16;
  RS16MsopPkt* mpkt_ptr = (RS16MsopPkt*)pkt;
  if (mpkt_ptr->header.id != RS16_MSOP_ID)
  {
    //      rs_print(RS_ERROR, "[RS16] MSOP pkt ID no match.");
    return -2;
  }
  int first_azimuth;
  first_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  float temperature = this->computeTemperature(mpkt_ptr->header.temp_raw);
  for (int blk_idx = 0; blk_idx < RS16_BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != RS16_BLOCK_ID)
    {
      break;
    }
    int azimuth_blk = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    int azi_prev = 0;
    int azi_cur = 0;

    if (blk_idx < (RS16_BLOCKS_PER_PKT - 1))  // 12
    {
      azi_prev = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth);
      azi_cur = azimuth_blk;
    }
    else
    {
      azi_prev = azimuth_blk;
      azi_cur = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth);
    }
    float azimuth_diff = (float)((36000 + azi_prev - azi_cur) % 36000);

    float azimuth_channel;
    for (int channel_idx = 0; channel_idx < RS16_CHANNELS_PER_BLOCK; channel_idx++)
    {
      if (this->echo_mode_ == ECHO_DUAL)
      {
        azimuth_channel =
            azimuth_blk + azimuth_diff * RS16_CHANNEL_TOFFSET * (channel_idx % 16) / RS16_BLOCK_TDURATION_DUAL;
      }
      else
      {
        azimuth_channel =
            azimuth_blk + azimuth_diff *
                              (RS16_FIRING_TDURATION * (channel_idx / 16) + RS16_CHANNEL_TOFFSET * (channel_idx % 16)) /
                              RS16_BLOCK_TDURATION_SINGLE;
      }
      int azimuth_final = (((int)azimuth_channel % 36000) + 36000) % 36000;

      int distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance);
      float distance_cali = distance * RS_RESOLUTION_5mm_DISTANCE_COEF;

      int angle_horiz_ori = azimuth_final;
      int angle_vert = (((int)(this->vert_angle_list_[channel_idx % 16]) % 36000) + 36000) % 36000;

      // store to pointcloud buffer
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
        point.intensity = NAN;
      }
#ifdef RS_POINT_COMPLEX
      point.distance = distance_cali;
      point.ring_id = channel_idx % 16;
      point.echo_id = (this->echo_mode_ == ECHO_DUAL) ? (channel_idx / 16) : 0;
#endif
      vec.emplace_back(std::move(point));
    }
  }
  return first_azimuth;
}

template <typename vpoint>
int32_t DecoderRS16<vpoint>::decodeDifopPkt(const uint8_t* pkt)
{
  RS16DifopPkt* rs16_ptr = (RS16DifopPkt*)pkt;
  if (rs16_ptr->id != RS16_DIFOP_ID)
  {
    //        rs_print(RS_ERROR, "[RS16] DIFOP pkt ID no match.");
    return -2;
  }
  this->rpm_ = RS_SWAP_SHORT(rs16_ptr->rpm);
  if (rs16_ptr->return_mode == 0x01 || rs16_ptr->return_mode == 0x02)
  {
    this->echo_mode_ = rs16_ptr->return_mode;
  }
  else
  {
    this->echo_mode_ = ECHO_DUAL;
  }

  int pkt_rate = ceil(RS16_POINTS_CHANNEL_PER_SECOND / RS16_BLOCKS_CHANNEL_PER_PKT);
  if (this->echo_mode_ == ECHO_LAST || this->echo_mode_ == ECHO_STRONGEST)
  {
    pkt_rate = ceil(pkt_rate / 2);
  }
  this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);

  if (!this->difop_flag_)
  {
    bool angle_flag = true;
    const uint8_t* p_ver_cali;

    p_ver_cali = rs16_ptr->pitch_cali;

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

  return 0;
}

}  // namespace lidar
}  // namespace robosense