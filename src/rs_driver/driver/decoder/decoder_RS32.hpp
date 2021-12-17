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
#include <rs_driver/utility/dbg.h>

namespace robosense
{
namespace lidar
{

#pragma pack(push, 1)
typedef struct
{
  uint8_t id[2];
  uint16_t azimuth;
  RSChannel channels[32];
} RS32MsopBlock;

typedef struct
{
  RSMsopHeaderV1 header;
  RS32MsopBlock blocks[12];
  unsigned int index;
  uint16_t tail;
} RS32MsopPkt;

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV1 eth;
  RSFOV fov;
  uint16_t reserved0;
  uint16_t phase_lock_angle;
  RSVersionV1 version;
  uint8_t reserved_1[242];
  RSSN sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  uint16_t sw_ver;
  RSTimestampYMD timestamp;
  RSStatusV1 status;
  uint8_t reserved_2[5];
  RSDiagnoV1 diagno;
  uint8_t gprmc[86];
  RSCalibrationAngle ver_angle_cali[32];
  RSCalibrationAngle hori_angle_cali[32];
  uint8_t reserved_3[586];
  uint16_t tail;
} RS32DifopPkt;
#pragma pack(pop)

template <typename T_PointCloud>
class DecoderRS32 : public Decoder<T_PointCloud>
{
public:

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual void decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual ~DecoderRS32() = default;

  explicit DecoderRS32(const RSDecoderParam& param, 
      const std::function<void(const Error&)>& excb);

#ifndef UNIT_TEST
protected:
#endif

  static RSDecoderConstParam getConstParam();
  static RSEchoMode getEchoMode(uint8_t mode);

  template <typename T_BlockDiff>
  void internDecodeMsopPkt(const uint8_t* pkt, size_t size);
};

template <typename T_PointCloud>
RSDecoderConstParam DecoderRS32<T_PointCloud>::getConstParam()
{
  RSDecoderConstParam param = 
  {
      1248 // msop len
    , 1248 // difop len
    , 8 // msop id len
    , 8 // difop id len
    , {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0xFF, 0xEE} // block id
    , 12 // blocks per packet
    , 32 // channels per block
    , 0.4f // distance min
    , 200.0f // distance max
    , 0.005f // distance resolution
    , 0.0625f // temperature resolution

    // lens center
    , 0.03997f // RX
    , -0.01087f // RY
    , 0.0f // RZ
  };

  float blk_ts = 55.52f;
  float firing_tss[] = 
  {
    0.00f,  2.88f,  5.76f,  8.64f, 11.52f, 14.40f, 17.28f, 20.16f, 
    23.04f, 25.92f, 28.80f, 31.68f, 34.56f, 37.44f, 40.32f, 44.64f,
    1.44f,  4.32f,  7.20f, 10.08f, 12.96f, 15.84f, 18.72f, 21.60f,
    24.48f, 27.36f, 30.24f, 33.12f, 36.00f, 38.88f, 41.76f, 46.08f
  };

  param.BLOCK_DURATION = blk_ts / 1000000;
  for (uint16_t i = 0; i < sizeof(firing_tss)/sizeof(firing_tss[0]); i++)
  {
    param.CHAN_TSS[i] = (double)firing_tss[i] / 1000000;
    param.CHAN_AZIS[i] = firing_tss[i] / blk_ts;
  }

  return param;
}

template <typename T_PointCloud>
RSEchoMode DecoderRS32<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode)
  {
    case 0x00: // dual return
      return RSEchoMode::ECHO_DUAL;
    case 0x01: // strongest return
    case 0x02: // last return
    default:
      return RSEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline DecoderRS32<T_PointCloud>::DecoderRS32(const RSDecoderParam& param,
      const std::function<void(const Error&)>& excb)
  : Decoder<T_PointCloud>(param, excb, getConstParam())
{
}

template <typename T_PointCloud>
inline void DecoderRS32<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RS32DifopPkt& pkt = *(const RS32DifopPkt*)(packet);
  this->template decodeDifopCommon<RS32DifopPkt>(pkt);

  this->echo_mode_ = getEchoMode (pkt.return_mode);
  this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
    (this->blks_per_frame_ << 1) : this->blks_per_frame_;

  //
  // RS32's channel angles is of higher resolution than the other lidars. 
  // fix them to the same resolution.
  //
  this->chan_angles_.narrow();
}

template <typename T_PointCloud>
inline void DecoderRS32<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
{
  if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
  {
    internDecodeMsopPkt<SingleReturnBlockDiff<RS32MsopPkt>>(pkt, size);
  }
  else
  {
    internDecodeMsopPkt<DualReturnBlockDiff<RS32MsopPkt>>(pkt, size);
  }
}

template <typename T_PointCloud>
template <typename T_BlockDiff>
inline void DecoderRS32<T_PointCloud>::internDecodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RS32MsopPkt& pkt = *(const RS32MsopPkt*)(packet);

  this->temperature_ = calcTemp(&(pkt.header.temp)) * this->const_param_.TEMPERATURE_RES;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = calcTimeYMD(&pkt.header.timestamp);
  }
  else
  {
    pkt_ts = calcTimeHost();
  }

  T_BlockDiff diff(pkt, this->const_param_.BLOCKS_PER_PKT, this->const_param_.BLOCK_DURATION);

  double block_ts = pkt_ts;
  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RS32MsopBlock& block = pkt.blocks[blk];

    if (memcmp(this->const_param_.BLOCK_ID, block.id, 2) != 0)
    {
      this->excb_(Error(ERRCODE_WRONGPKTHEADER));
      break;
    }

    block_ts += diff.ts(blk);
    int32_t block_az = ntohs(block.azimuth);

    this->newBlock(block_az);

    int32_t block_azi_diff = diff.azimuth(blk);

    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      const RSChannel& channel = block.channels[chan]; 

      double chan_ts = block_ts + this->const_param_.CHAN_TSS[chan];
      int32_t angle_horiz = block_az + 
        (int32_t)((float)block_azi_diff * this->const_param_.CHAN_AZIS[chan]);

      int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, angle_horiz);

      float distance = ntohs(channel.distance) * this->const_param_.DISTANCE_RES;
      uint8_t intensity = channel.intensity;

      if (this->distance_section_.in(distance) && this->scan_section_.in(angle_horiz_final))
      {
        float x =  distance * COS(angle_vert) * COS(angle_horiz_final) + this->const_param_.RX * COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - this->const_param_.RX * SIN(angle_horiz);
        float z =  distance * SIN(angle_vert) + this->const_param_.RZ;

        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, intensity);

        setTimestamp(point, chan_ts);
        setRing(point, this->chan_angles_.toUserChan(chan));

        this->point_cloud_->points.emplace_back(point);
      }
      else if (!this->param_.dense_points)
      {
        typename T_PointCloud::PointT point;
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);

        setTimestamp(point, chan_ts);
        setRing(point, this->chan_angles_.toUserChan(chan));

        this->point_cloud_->points.emplace_back(point);
      }

      this->prev_chan_ts_ = chan_ts;
    }
  }
}

}  // namespace lidar
}  // namespace robosense
