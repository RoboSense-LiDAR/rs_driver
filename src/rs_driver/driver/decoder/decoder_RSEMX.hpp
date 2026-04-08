/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <rs_driver/driver/decoder/decoder.hpp>

#define EMX_SURFACE_NUM 2
#define EMX_PIXELS_PER_COLUMN 192
#define EMX_VECSELS_PER_COLUMN 24
#define EMX_PIXELS_PER_VCSEL 8

namespace robosense
{
namespace lidar
{
#pragma pack(push, 1)

typedef struct
{
  uint8_t ip_local[4];
  uint8_t net_mask[4];
  uint8_t mac[6];
  uint8_t msop_ip_remote[4];
  uint8_t msop_local_port[2];
  uint8_t msop_remote_port[2];
  uint8_t difop_remote_ip[4];
  uint8_t difop_local_port[2];
  uint8_t difop_remote_port[2];
  uint8_t doip_ip_remote[4];
  uint8_t doip_local_port[2];
  uint8_t doip_remote_port[2];
  uint8_t logic_ecu_addr[2];
  uint8_t logic_func_addr[2];
  uint8_t logic_tester_addr[2];
  uint8_t net_addr[2];
  uint8_t reserved1[7];
} RSEMXDifopEther;

typedef struct
{
  uint8_t pwr_spl_fault[3];
  uint8_t temp_fault[3];
  uint8_t sw_fault[3];
  uint8_t perf_fault[6];
  uint8_t fault_status;
  uint8_t fault_level;
} RSEMXFaults;

typedef struct
{
  uint8_t id[8];
  uint8_t sw_ver[4];
  uint8_t AIPn[4];
  uint8_t PIPn[4];
  uint8_t mcu_ver[4];
  uint8_t hw_ver;
  RSSN int_sn;
  uint8_t _sn[4];
  uint8_t reserved_0[9];
  RSEMXDifopEther ether;
  uint8_t work_mode;
  uint8_t frame_rate;
  uint8_t wave_mode;
  RSTimeInfo time_info;
  uint8_t master_slave_mode;
  uint8_t surface_id;
  int8_t yaw_offset[26];
  int16_t pitch_angle[EMX_PIXELS_PER_COLUMN];
  int16_t roll_offset;
  uint8_t voltage_temp[66];
  uint8_t reserved2[5];
  RSEMXFaults faults;
  uint8_t reserved3[28];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEMXDifop2_1Pkt;

typedef struct
{
  uint8_t id[4];
  uint8_t reserved0[63];
  uint8_t surface_id;
  uint8_t pixelCnt;  
  uint8_t vcselCnt;
  int8_t yaw_offset[24];
  int16_t pitch_angle[EMX_PIXELS_PER_COLUMN];
  int16_t surface_pitch_offset[2];
  int16_t roll_offset;
  uint8_t reserved1[4];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEMXDifop2_2Pkt;

typedef struct
{
  uint8_t id[4];
  uint8_t payload[303];
  uint8_t surfaceCnt;
  uint8_t pixelCnt;
  uint8_t vcselCnt;
  int8_t yaw_offset[24];
  int16_t pitch_angle[EMX_PIXELS_PER_COLUMN];
  int16_t surface_pitch_offset[2];
  int16_t roll_offset;
  uint8_t reserved1[4];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEMXDifop2_3Pkt;

typedef struct
{
  uint8_t id[4];
  uint8_t reserved1[319];
  uint8_t surfaceCnt;
  uint8_t pixelCnt;
  uint8_t vcselCnt;
  int8_t yaw_offset[24];
  int16_t pitch_angle[EMX_PIXELS_PER_COLUMN];
  int16_t surface_pitch_offset[2];
  int16_t roll_offset;
  uint8_t reserved2[4];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEMXDifop2_4Pkt; 

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
  uint8_t point_attribute;
} RSEMXChannel;  // 4-bytes

typedef struct
{
  RSEMXChannel channel[1];
} RSEMXBlock;

typedef struct
{
  uint8_t id[4];
  uint16_t pkt_seq;
  uint16_t pitch_offset;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimestampUTC timestamp;
  uint8_t fram_sync;
  uint8_t frame_rate;
  uint16_t column_num;
  int16_t yaw_angle;
  uint8_t pack_mode;
  uint8_t surface_id;
  uint16_t temperature;
  uint8_t lidar_type;
  uint8_t reserved;
} RSEMXMsopHeader;  // 32-bytes

typedef struct
{
  RSEMXMsopHeader header;
  RSEMXBlock blocks[EMX_PIXELS_PER_COLUMN];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEMXMsopPkt;  // 812-bytes

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderRSEMX : public Decoder<T_PointCloud>
{
public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 1520;

  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) override;
  virtual ~DecoderRSEMX(){};

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size) override;

  explicit DecoderRSEMX(const RSDecoderParam& param);

private:
  int last_pkt_cnt_;
  int last_pkt_pack_num_;

  std::array<int, EMX_PIXELS_PER_COLUMN> pitch_angle_;
  std::array<int, 2 * EMX_PIXELS_PER_COLUMN> dual_return_pitch_index_;

  static RSDecoderConstParam& getConstParam();
  SplitStrategyBySeq split_strategy_;
};
template <typename T_PointCloud>
inline RSDecoderConstParam& DecoderRSEMX<T_PointCloud>::getConstParam()
{
  static RSDecoderConstParam param = {
    812  // msop len
    ,
    654  // difop len
    ,
    4  // msop id len
    ,
    3  // difop id len
    ,
    { 0x55, 0xAA, 0x5A, 0xA5 }  // msop id
    ,
    { 0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55 }  // difop id
    ,
    { 0x00, 0x00 },
    1  // laser number
    ,
    EMX_PIXELS_PER_COLUMN  // blocks per packet
    ,
    1  // channels per block
    ,
    0.2f  // distance min
    ,
    300.0f  // distance max
    ,
    0.005f  // distance resolution
    ,
    80.0f  // initial value of temperature
  };

  return param;
}

template <typename T_PointCloud>
inline DecoderRSEMX<T_PointCloud>::DecoderRSEMX(const RSDecoderParam& param)
  : Decoder<T_PointCloud>(getConstParam(), param), last_pkt_cnt_(-1), last_pkt_pack_num_(-1)

{
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;

  this->bCheckMsopLen_ = true;
  this->bCheckDifopLen_ = false;

  std::array<int, EMX_PIXELS_PER_COLUMN> defaultAngle;
  constexpr int START_ANGLE = -24160;
  constexpr int ANGLE_STEP = 210;
  for (int i = 0; i < EMX_PIXELS_PER_COLUMN; ++i)
  {
    defaultAngle[i] = (START_ANGLE + i * ANGLE_STEP)/2;
  }
  this->pitch_angle_ = defaultAngle;

  for (int i = 0, j = 0; i < EMX_PIXELS_PER_COLUMN; i++)
  {
    this->dual_return_pitch_index_[j++] = i;
    this->dual_return_pitch_index_[j++] = i;
  }
}

template <typename T_PointCloud>
inline void DecoderRSEMX<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  if (packet == nullptr){
    return;
  }
  const static uint16_t DIFOP2_1_LEN = sizeof(RSEMXDifop2_1Pkt);
  const static uint16_t DIFOP2_2_LEN = sizeof(RSEMXDifop2_2Pkt);
  const static uint16_t DIFOP2_3_LEN = sizeof(RSEMXDifop2_3Pkt);
  const static uint16_t DIFOP2_4_LEN = sizeof(RSEMXDifop2_4Pkt);
  auto processDifopPkt = [this](const auto& pkt) {
    for (int i = 0; i < EMX_PIXELS_PER_COLUMN; i++)
    {
      this->pitch_angle_[i] = static_cast<int>(RS_SWAP_INT16(pkt.pitch_angle[i])*10 / 2);
    }
    this->angles_ready_ = true;
  };

  if (size == DIFOP2_1_LEN)
  {
    processDifopPkt(*reinterpret_cast<const RSEMXDifop2_1Pkt*>(packet));
  }
  else if (size == DIFOP2_2_LEN)
  {
    processDifopPkt(*reinterpret_cast<const RSEMXDifop2_2Pkt*>(packet));
  }
  else if(size ==DIFOP2_3_LEN)
  {
    processDifopPkt(*reinterpret_cast<const RSEMXDifop2_3Pkt*>(packet));
  }
  else if(size ==DIFOP2_4_LEN)
  {
    processDifopPkt(*reinterpret_cast<const RSEMXDifop2_4Pkt*>(packet));
  }
}

template <typename T_PointCloud>
inline bool DecoderRSEMX<T_PointCloud>::decodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RSEMXMsopPkt& pkt = *(RSEMXMsopPkt*)packet;
  bool ret = false;

  constexpr float SCALE_FACTOR = 503.975f / 65536.0f;
  constexpr float KELVIN_OFFSET = 273.15f;
  constexpr float PITCH_OFFSET_SCALE = 1.0f / 32768.0f;

  this->temperature_ = static_cast<float>(pkt.header.temperature) * SCALE_FACTOR - KELVIN_OFFSET;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs(&pkt.header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = getTimeHost() * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs(ts, (RSTimestampUTC*)&pkt.header.timestamp);
    }
  }

  // Convert pkt_seq from 1-based to 0-based index.
  uint16_t pkt_seq = ntohs(pkt.header.pkt_seq) - 1;
  uint8_t surface_index = pkt.header.surface_id;
  if (surface_index >= EMX_SURFACE_NUM)
  {
    RS_WARNING << "surface_id > EMX_SURFACE_NUM, surface_id = " << (int)surface_index
               << ", EMX_SURFACE_NUM = " << EMX_SURFACE_NUM << RS_REND;
    return false;
  }
  if (split_strategy_.newPacket(pkt_seq))
  {
    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    this->first_point_ts_ = pkt_ts;
    ret = true;
  }
  const bool is_dual_mode = (pkt.header.return_mode == 0);  // 0, dual, 1, single(first) , 2, single(second)
  const uint16_t blocks_per_pkt = this->const_param_.BLOCKS_PER_PKT;
  const uint16_t channels_per_block = this->const_param_.CHANNELS_PER_BLOCK;
  const bool dense_points = this->param_.dense_points;
  const float distance_res = this->const_param_.DISTANCE_RES;
  const float yaw_factor = 1000.0f / 512.0f;

  const int yaw_base = static_cast<int>(RS_SWAP_INT16(pkt.header.yaw_angle));
  const float pitch_offset = static_cast<int16_t>(ntohs(pkt.header.pitch_offset)) * PITCH_OFFSET_SCALE;
  uint16_t seq_mod = (is_dual_mode && (pkt_seq % 2 != 0)) ? EMX_PIXELS_PER_COLUMN : 0;

  for (uint16_t blk = 0; blk < blocks_per_pkt; ++blk)
  {
    const RSEMXBlock& block = pkt.blocks[blk];
    const double point_time = pkt_ts;

    uint16_t real_chan;
    if (is_dual_mode)
    {
      const uint16_t real_blk = blk + seq_mod;
      real_chan = this->dual_return_pitch_index_[real_blk];
    }
    else
    {
      real_chan = blk;
    }
    
    int yaw = static_cast<int>(std::round(yaw_base * yaw_factor));
    const int pitch = this->pitch_angle_[real_chan] - static_cast<int>(std::round(pitch_offset * 1000.0f));
    const float cos_pitch = COS_FINE(pitch);
    const float sin_pitch = SIN_FINE(pitch);
    const float cos_yaw = COS_FINE(yaw);
    const float sin_yaw = SIN_FINE(yaw);

    for (uint16_t chan = 0; chan < channels_per_block; ++chan)
    {
      const RSEMXChannel& channel = block.channel[chan];
      const float distance = ntohs(channel.distance) * distance_res;
      const uint8_t feature = channel.point_attribute;

      typename T_PointCloud::PointT point;
      if (this->distance_section_.in(distance))
      {
        float x = distance * cos_pitch * cos_yaw;
        float y = distance * cos_pitch * sin_yaw;
        float z = distance * sin_pitch;
        this->transformPoint(x, y, z);

        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.intensity);
        setRing(point, real_chan);
        setFeature(point,feature);
      }
      else if (!dense_points)
      {
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        setRing(point, real_chan);
        setFeature(point,feature);
      }
      else
      {
        continue;
      }

      setTimestamp(point, point_time);
      this->point_cloud_->points.emplace_back(std::move(point));
    }
  }

  this->prev_point_ts_ = pkt_ts;
  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

}  // namespace lidar
}  // namespace robosense
