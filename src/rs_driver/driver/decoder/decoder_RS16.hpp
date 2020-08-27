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
#pragma pack(push, 1)
typedef struct
{
  uint16_t id;
  uint16_t azimuth;
  RSChannel channels[32];
} RS16MsopBlock;

typedef struct
{
  RSMsopHeader header;
  RS16MsopBlock blocks[12];
  unsigned int index;
  uint16_t tail;
} RS16MsopPkt;

typedef struct
{
  uint64_t id;
  uint16_t rpm;
  RSEthNet eth;
  RSFOV fov;
  uint16_t static_base;
  uint16_t phase_lock_angle;
  RSVersion version;
  RSIntensity intensity;
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
} RS16DifopPkt;

#pragma pack(pop)

template <typename T_Point>
class DecoderRS16 : public DecoderBase<T_Point>
{
public:
  DecoderRS16(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);
};

template <typename T_Point>
inline DecoderRS16<T_Point>::DecoderRS16(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : DecoderBase<T_Point>(param, lidar_const_param)
{
  this->vert_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->hori_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->beam_ring_table_.resize(this->lidar_const_param_.LASER_NUM);
  if (this->param_.max_distance > 150.0f)
  {
    this->param_.max_distance = 150.0f;
  }
  if (this->param_.min_distance < 0.2f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 0.2f;
  }
}

template <typename T_Point>
inline double DecoderRS16<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeYMD<RS16MsopPkt>(pkt);
}

template <typename T_Point>
inline RSDecoderResult DecoderRS16<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                           int& azimuth)
{
  height = this->lidar_const_param_.LASER_NUM;
  RS16MsopPkt* mpkt_ptr = (RS16MsopPkt*)pkt;
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_raw);
  double block_timestamp = this->get_point_time_func_(pkt);
  this->check_camera_trigger_func_(azimuth, pkt);
  float azi_diff = 0;
  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != this->lidar_const_param_.BLOCK_ID)
    {
      break;
    }
    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
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
    azi_diff = (azi_diff > 100) ? this->azi_diff_between_block_theoretical_ : azi_diff;
    for (size_t channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      float azi_channel_ori = 0;
      if (this->echo_mode_ == ECHO_DUAL)
      {
        azi_channel_ori = cur_azi + azi_diff * this->lidar_const_param_.DSR_TOFFSET *
                                        this->lidar_const_param_.FIRING_FREQUENCY * 2.0f * float(channel_idx % 16);
      }
      else
      {
        azi_channel_ori = cur_azi + azi_diff * ((this->lidar_const_param_.DSR_TOFFSET *
                                                 this->lidar_const_param_.FIRING_FREQUENCY * float(channel_idx % 16)) +
                                                float(channel_idx / 16) * 0.5f);
      }
      int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx % 16);
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * RS_RESOLUTION;
      int angle_horiz_ori = (int)(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert = ((this->vert_angle_list_[channel_idx % 16]) + RS_ONE_ROUND) % RS_ONE_ROUND;
      T_Point point;
      if ((distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
          ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
           (!this->angle_flag_ &&
            ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))
      {
        double x = distance * this->cos_lookup_table_[angle_vert] * this->cos_lookup_table_[azi_channel_final] +
                   this->lidar_const_param_.RX * this->cos_lookup_table_[angle_horiz_ori];

        double y = -distance * this->cos_lookup_table_[angle_vert] * this->sin_lookup_table_[azi_channel_final] -
                   this->lidar_const_param_.RX * this->sin_lookup_table_[angle_horiz_ori];
        double z = distance * this->sin_lookup_table_[angle_vert] + this->lidar_const_param_.RZ;
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
      setRing(point, this->beam_ring_table_[channel_idx % 16]);
      if (this->echo_mode_ != ECHO_DUAL && channel_idx > 15)
      {
        setTimestamp(point, block_timestamp + this->time_duration_between_blocks_ / 2);
      }
      else
      {
        setTimestamp(point, block_timestamp);
      }
      vec.emplace_back(std::move(point));
    }
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline RSDecoderResult DecoderRS16<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  RS16DifopPkt* dpkt_ptr = (RS16DifopPkt*)pkt;
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
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
      (60 / (float)this->rpm_) /
      ((this->lidar_const_param_.PKT_RATE * 60 / this->rpm_) * this->lidar_const_param_.BLOCKS_PER_PKT);
  int fov_start_angle = RS_SWAP_SHORT(dpkt_ptr->fov.start_angle);
  int fov_end_angle = RS_SWAP_SHORT(dpkt_ptr->fov.end_angle);
  int fov_range = (fov_start_angle < fov_end_angle) ? (fov_end_angle - fov_start_angle) :
                                                      (RS_ONE_ROUND - fov_start_angle + fov_end_angle);
  int blocks_per_round =
      (this->lidar_const_param_.PKT_RATE / (this->rpm_ / 60)) * this->lidar_const_param_.BLOCKS_PER_PKT;
  this->fov_time_jump_diff_ =
      this->time_duration_between_blocks_ * (fov_range / (RS_ONE_ROUND / (float)blocks_per_round));
  if (this->echo_mode_ == RSEchoMode::ECHO_DUAL)
  {
    this->pkts_per_frame_ = ceil(2 * this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  }
  else
  {
    this->pkts_per_frame_ = ceil(this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  }
  this->azi_diff_between_block_theoretical_ =
      (RS_ONE_ROUND / this->lidar_const_param_.BLOCKS_PER_PKT) /
      (float)this->pkts_per_frame_;  ///< ((rpm/60)*360)/pkts_rate/blocks_per_pkt
  if (!this->difop_flag_)
  {
    const uint8_t* p_ver_cali = dpkt_ptr->pitch_cali;
    if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) && (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
        (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) && (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF))
    {
      return RSDecoderResult::DECODE_OK;
    }
    int lsb, mid, msb, neg = 1;
    for (size_t i = 0; i < this->lidar_const_param_.CHANNELS_PER_BLOCK / 2; i++)
    {
      /* vert angle calibration data */
      lsb = p_ver_cali[i * 3];
      mid = p_ver_cali[i * 3 + 1];
      msb = p_ver_cali[i * 3 + 2];
      neg = i < 8 ? -1 : 1;
      this->vert_angle_list_[i] = (lsb * 256 * 256 + mid * 256 + msb) * neg * 0.01f;  // / 180 * M_PI;
      this->hori_angle_list_[i] = 0;
    }
    this->sortBeamTable();
    this->difop_flag_ = true;
  }
  return RSDecoderResult::DECODE_OK;
}

}  // namespace lidar
}  // namespace robosense