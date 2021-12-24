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

#include <rs_driver/driver/decoder/decoder.hpp>

namespace robosense
{
namespace lidar
{
#pragma pack(push, 1)

typedef struct
{
  uint8_t id[4];
  uint16_t pkt_seq;
  uint16_t protocol_version;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimestampUTC timestamp;
  uint8_t reserved[10];
  uint8_t lidar_type;
  int8_t temperature;
} RSM1MsopHeader;

typedef struct
{
  uint16_t distance;
  uint16_t pitch;
  uint16_t yaw;
  uint8_t intensity;
  uint8_t point_attribute;
  uint8_t elongation;
} RSM1Channel;

typedef struct
{
  uint8_t time_offset;
  uint8_t return_seq;
  RSM1Channel channel[5];
} RSM1Block;

typedef struct
{
  RSM1MsopHeader header;
  RSM1Block blocks[25];
  uint8_t reserved[3];
} RSM1MsopPkt;

typedef struct
{
  uint8_t ip_local[4];
  uint8_t ip_remote[4];
  uint8_t mac[6];
  uint8_t msop_port[2];
  uint8_t difop_port[2];
} RSM1DifopEther;

typedef struct
{
  uint8_t horizontal_fov_start[2];
  uint8_t horizontal_fov_end[2];
  uint8_t vertical_fov_start[2];
  uint8_t vertical_fov_end[2];
} RSM1DifopFov;

typedef struct
{
  uint8_t pl_ver[5];
  uint8_t ps_ver[5];
} RSM1DifopVerInfo;

typedef struct
{
  uint8_t current_1[3];
  uint8_t current_2[3];
  uint16_t voltage_1;
  uint16_t voltage_2;
  uint8_t reserved[10];
} RSM1DifopRunSts;

typedef struct
{
  uint8_t param_sign;
  uint16_t data;
} RSM1DifopCalibration;

typedef struct
{
  uint8_t id[8];
  uint8_t reserved1[1];
  uint8_t frame_rate;
  RSM1DifopEther ether;
  RSM1DifopFov fov;
  RSM1DifopVerInfo ver_info;
  RSSN sn;
  uint8_t return_mode;
  RSTimeInfo time_info;
  RSM1DifopRunSts status;
  uint8_t reserved2[40];
  RSM1DifopCalibration cali_param[20];
  uint8_t reserved3[71];
} RSM1DifopPkt;

#pragma pack(pop)

const uint32_t SINGLE_PKT_NUM = 630;
const uint32_t DUAL_PKT_NUM = 1260;

template <typename T_PointCloud>
class DecoderRSM1 : public Decoder<T_PointCloud>
{
public:

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual void decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual ~DecoderRSM1() = default;

  explicit DecoderRSM1(const RSDecoderParam& param, 
      const std::function<void(const Error&)>& excb);

private:

  RSDecoderConstParam getConstParam();
  RSEchoMode getEchoMode(uint8_t mode);

  uint16_t max_seq_;
  SplitStrategyBySeq split_;
};

template <typename T_PointCloud>
RSDecoderConstParam DecoderRSM1<T_PointCloud>::getConstParam()
{
  RSDecoderConstParam param = 
  {
      1210 // msop len
    , 256 // difop len
    , 4 // msop id len
    , 8 // difop id len
    , {0x55, 0xAA, 0x5A, 0xA5} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0x00, 0x00}
    , 0 
    , 0
    , 0.2f // distance min
    , 200.0f // distance max
    , 0.005f // distance resolution
  };

  return param;
}

template <typename T_PointCloud>
inline DecoderRSM1<T_PointCloud>::DecoderRSM1(const RSDecoderParam& param, 
      const std::function<void(const Error&)>& excb)
  : Decoder<T_PointCloud>(param, excb, getConstParam())
  , max_seq_(SINGLE_PKT_NUM)
  , split_(&max_seq_)
{
//  this->time_duration_between_blocks_ = 5 * 1e-6;
}

template <typename T_PointCloud>
RSEchoMode DecoderRSM1<T_PointCloud>::getEchoMode(uint8_t mode)
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
void DecoderRSM1<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RSM1DifopPkt& pkt = *(RSM1DifopPkt*)packet;
  this->echo_mode_ = this->getEchoMode(pkt.return_mode);
  max_seq_ = (this->echo_mode_ == ECHO_SINGLE) ? 630 : 1260;
}

template <typename T_PointCloud>
void DecoderRSM1<T_PointCloud>::decodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RSM1MsopPkt& pkt = *(RSM1MsopPkt*)packet;

  this->temperature_ = static_cast<float>(pkt.header.temperature - 80);

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs(&pkt.header.timestamp) * 0.000001;
  }
  else
  {
    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = getTimeHost() * 0.000001 - this->getPacketDuration();
  }
  
  for (size_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RSM1Block& block = pkt.blocks[blk];

    double point_time = pkt_ts + block.time_offset * 1e-6;

    for (size_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      const RSM1Channel& channel = block.channel[chan];

      float distance = ntohs(channel.distance) * this->const_param_.DISTANCE_RES;

      if (this->distance_section_.in(distance))
      {
        static const int ANGLE_OFFSET = 32768;
        int pitch = ntohs(channel.pitch) - ANGLE_OFFSET;
        int yaw = ntohs(channel.yaw) - ANGLE_OFFSET;

        float x = distance * COS (pitch) * COS (yaw);
        float y = distance * COS (pitch) * SIN (yaw);
        float z = distance * SIN (pitch);
        uint8_t intensity = channel.intensity;

        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, intensity);
        setRing(point, chan + 1);
        setTimestamp(point, point_time);
        this->point_cloud_->points.emplace_back(point);
      }
      else if (!this->param_.dense_points)
      {
        typename T_PointCloud::PointT point;
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        setRing(point, chan + 1);
        setTimestamp(point, point_time);
        this->point_cloud_->points.emplace_back(point);
      }
    }
  }

  uint16_t pkt_seq = ntohs(pkt.header.pkt_seq);
  if (split_.newPacket(pkt_seq))
  {
    this->splitFrame();
  }
}

}  // namespace lidar
}  // namespace robosense
