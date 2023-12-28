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
  uint16_t pitch;
  uint16_t yaw;
  uint8_t intensity;
} RSM1_Jumbo_Channel;

typedef struct
{
  uint8_t time_offset;
  uint8_t return_seq;
  RSM1_Jumbo_Channel channel[5];
} RSM1_Jumbo_Block;

typedef struct
{
  RSM1MsopHeader header;
  RSM1_Jumbo_Block blocks[25];
  uint8_t reserved1[11];
  uint8_t reserved2[16];
} RSM1_Jumbo_MsopPkt;

typedef struct
{
  RSM1_Jumbo_MsopPkt pkts[63];
  uint8_t reserved[160];
} RSM1_Jumbo;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderRSM1_Jumbo : public Decoder<T_PointCloud>
{
public:

  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 630;
  constexpr static int ANGLE_OFFSET = 32768;

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual ~DecoderRSM1_Jumbo() = default;

  explicit DecoderRSM1_Jumbo(const RSDecoderParam& param);

private:

  static RSDecoderConstParam& getConstParam();
  RSEchoMode getEchoMode(uint8_t mode);

  bool internDecodeMsopPkt(const uint8_t* pkt, size_t size);
  SplitStrategyBySeq split_strategy_;
};

template <typename T_PointCloud>
inline RSDecoderConstParam& DecoderRSM1_Jumbo<T_PointCloud>::getConstParam()
{
  static RSDecoderConstParam param = 
  {
    62152 // msop len
      , 256 // difop len
      , 4 // msop id len
      , 8 // difop id len
      , {0x55, 0xAA, 0x5A, 0xA5} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0x00, 0x00}
    , 5  // laser number
    , 25 // blocks per packet
      , 5 // channels per block
      , 0.2f // distance min
      , 200.0f // distance max
      , 0.005f // distance resolution
      , 80.0f // initial value of temperature 
  };

  return param;
}

template <typename T_PointCloud>
inline DecoderRSM1_Jumbo<T_PointCloud>::DecoderRSM1_Jumbo(const RSDecoderParam& param)
  : Decoder<T_PointCloud>(getConstParam(), param)
{
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  this->angles_ready_ = true;
}

template <typename T_PointCloud>
inline RSEchoMode DecoderRSM1_Jumbo<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode)
  {
    case 0x00: // dual return
      return RSEchoMode::ECHO_DUAL;
    case 0x04: // strongest return
    case 0x05: // last return
    case 0x06: // first return
    default:
      return RSEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline void DecoderRSM1_Jumbo<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RSM1DifopPkt& pkt = *(RSM1DifopPkt*)packet;
  this->echo_mode_ = this->getEchoMode(pkt.return_mode);
}

template <typename T_PointCloud>
inline bool DecoderRSM1_Jumbo<T_PointCloud>::decodeMsopPkt(const uint8_t* packet, size_t size)
{
  if (size != sizeof(RSM1_Jumbo))
    return false;

  constexpr static int PACKET_NUM = 63;

  const RSM1_Jumbo* jumbo = (const RSM1_Jumbo *)packet;
  bool ret = false;

  for (size_t i = 0; i < PACKET_NUM; i++)
  {
    bool r = internDecodeMsopPkt((const uint8_t*)&(jumbo->pkts[i]), sizeof(RSM1_Jumbo_MsopPkt));
    ret = (ret || r);
  }

  return ret;
}

template <typename T_PointCloud>
inline bool DecoderRSM1_Jumbo<T_PointCloud>::internDecodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RSM1_Jumbo_MsopPkt& pkt = *(RSM1_Jumbo_MsopPkt*)packet;
  bool ret = false;

  if (memcmp(packet, this->const_param_.MSOP_ID, this->const_param_.MSOP_ID_LEN) != 0)
    return false;

  this->temperature_ = static_cast<float>((int)pkt.header.temperature - this->const_param_.TEMPERATURE_RES);
  this->is_get_temperature_ = true;
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
      createTimeUTCWithUs (ts, (RSTimestampUTC*)&pkt.header.timestamp);
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
    const RSM1_Jumbo_Block& block = pkt.blocks[blk];

    double point_time = pkt_ts + block.time_offset * 1e-6;

    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      const RSM1_Jumbo_Channel& channel = block.channel[chan];

      float distance = ntohs(channel.distance) * this->const_param_.DISTANCE_RES;

      if (this->distance_section_.in(distance))
      {
        int pitch = ntohs(channel.pitch) - ANGLE_OFFSET;
        int yaw = ntohs(channel.yaw) - ANGLE_OFFSET;

        float x = distance * COS (pitch) * COS (yaw);
        float y = distance * COS (pitch) * SIN (yaw);
        float z = distance * SIN (pitch);
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
