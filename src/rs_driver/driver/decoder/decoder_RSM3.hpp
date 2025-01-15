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

#include <rs_driver/driver/decoder/decoder_RSM1.hpp>

namespace robosense
{
namespace lidar
{
#pragma pack(push, 1)

typedef struct
{
  uint16_t distance;
  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t intensity;
  uint8_t point_attribute;
} RSM3Channel;

typedef struct
{
  uint16_t time_offset;
  RSM3Channel channel[28];
} RSM3Block;

typedef struct
{
  RSM1MsopHeader header;
  RSM3Block blocks[5];
  uint8_t tail[2];
  uint8_t crc32[4];
} RSM3MsopPkt;

typedef struct
{
  uint8_t id[8];
  RSSN sn;
  uint8_t reserved1[96];
  RSTimeInfo time_info;
  uint8_t reserved2[390];
} RRSM3DifopPkt;
#pragma pack(pop)
template <typename T_PointCloud>
class DecoderRSM3 :public Decoder<T_PointCloud>
{
public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 2270;
  constexpr static int VECTOR_BASE = 32768; // 2^15
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual ~DecoderRSM3() = default;

  explicit DecoderRSM3(const RSDecoderParam& param);

private:

  static RSDecoderConstParam& getConstParam();
   SplitStrategyBySeq split_strategy_;
};
template <typename T_PointCloud>
inline RSDecoderConstParam& DecoderRSM3<T_PointCloud>::getConstParam()
{
  static RSDecoderConstParam param =
  {
    1448 // msop len
    , 512 // difop len
    , 4 // msop id len
    , 8 // difop id len
    , {0x55, 0xAA, 0x5A, 0xA5} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0x00, 0x00}
    , 28  // laser number
    , 5 // blocks per packet
    , 28 // channels per block
    , 0.1f // distance min
    , 300.0f // distance max
    , 0.005f // distance resolution
    , 80.0f // initial value of temperature 
  };

  return param;
}
template <typename T_PointCloud>
inline DecoderRSM3<T_PointCloud>::DecoderRSM3(const RSDecoderParam& param)
  : Decoder<T_PointCloud>(getConstParam(), param)
{
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  this->angles_ready_ = true;
}

template <typename T_PointCloud>
inline void DecoderRSM3<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
#ifdef ENABLE_DIFOP_PARSE
  const RRSM3DifopPkt& pkt = *(RRSM3DifopPkt*)packet;
  double difop_pkt_ts = parseTimeUTCWithUs(&pkt.time_info.timestamp) * 1e-6;
  if(0)
  {
    RS_DEBUG << "difop_pkt_ts:" << difop_pkt_ts << RS_REND;
  }
#endif
}


template <typename T_PointCloud>
inline bool DecoderRSM3<T_PointCloud>::decodeMsopPkt(const uint8_t* packet, size_t size)
{

  const RSM3MsopPkt& pkt = *(RSM3MsopPkt*)packet;
  bool ret = false;

  this->temperature_ = static_cast<float>((int)pkt.header.temperature - this->const_param_.TEMPERATURE_RES);

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

  uint16_t pkt_seq = ntohs(pkt.header.pkt_seq);
  if (split_strategy_.newPacket(pkt_seq))
  {
    
    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    this->first_point_ts_ = pkt_ts;
    ret = true;
  }

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RSM3Block& block = pkt.blocks[blk];

    double point_time = pkt_ts + ntohs(block.time_offset) * 1e-6;

    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      const RSM3Channel& channel = block.channel[chan];

      float distance = ntohs(channel.distance) * this->const_param_.DISTANCE_RES;

      if (this->distance_section_.in(distance))
      {

        int16_t vector_x = RS_SWAP_INT16(channel.x);
        int16_t vector_y = RS_SWAP_INT16(channel.y);
        int16_t vector_z = RS_SWAP_INT16(channel.z);

        float x = vector_x * distance / VECTOR_BASE;
        float y = vector_y * distance / VECTOR_BASE;
        float z = vector_z * distance / VECTOR_BASE;

        this->transformPoint(x, y, z);

        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.intensity);
        setTimestamp(point, point_time);
        setRing(point, chan);

        this->point_cloud_->points.emplace_back(point);
      }
      else if (!this->param_.dense_points)
      {
        typename T_PointCloud::PointT point;
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        setTimestamp(point, point_time);
        setRing(point, chan);

        this->point_cloud_->points.emplace_back(point);
      }
    }

    this->prev_point_ts_ = point_time;
   
  }

  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

}  // namespace lidar
}  // namespace robosense
