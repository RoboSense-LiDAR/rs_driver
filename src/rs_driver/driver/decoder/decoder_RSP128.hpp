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
#include <rs_driver/driver/decoder/decoder_mech.hpp>

namespace robosense
{
namespace lidar
{
#pragma pack(push, 1)
typedef struct
{
  uint8_t id[1];
  uint8_t ret_id;
  uint16_t azimuth;
  RSChannel channels[128];
} RSP128MsopBlock;

typedef struct
{
  RSMsopHeaderV2 header;
  RSP128MsopBlock blocks[3];
  uint8_t reserved[4];
} RSP128MsopPkt;

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV2 eth;
  RSFOV fov;
  uint8_t reserved1[2];
  uint16_t phase_lock_angle;
  RSVersionV2 version;
  uint8_t reserved2[229];
  RSSN sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  RSTimeInfo time_info;
  RSStatusV1 status;
  uint8_t reserved3[5];
  RSDiagnoV1 diagno;
  uint8_t gprmc[86];
  RSCalibrationAngle vert_angle_cali[128];
  RSCalibrationAngle horiz_angle_cali[128];
  uint8_t reserved4[10];
  uint16_t tail;
} RSP128DifopPkt;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderRSP128 : public DecoderMech<T_PointCloud>
{
public:
  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual ~DecoderRSP128() = default;

  explicit DecoderRSP128(const RSDecoderParam& param);

#ifndef UNIT_TEST
protected:
#endif

  static RSDecoderMechConstParam& getConstParam();
  static RSEchoMode getEchoMode(uint8_t mode);

  template <typename T_BlockIterator>
  bool internDecodeMsopPkt(const uint8_t* pkt, size_t size);
};

template <typename T_PointCloud>
inline RSDecoderMechConstParam& DecoderRSP128<T_PointCloud>::getConstParam()
{
  static RSDecoderMechConstParam param = 
  {
    1248 // msop len
      , 1248 // difop len
      , 4 // msop id len
      , 8 // difop id len
      , {0x55, 0xAA, 0x05, 0x5A} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0xFE} // block id
    , 128 // laser number
    , 3 // blocks per packet
      , 128 // channels per block
      , 0.4f // distance min
      , 250.0f // distance max
      , 0.005f // distance resolution
      , 0.0625f // temperature resolution

      // lens center
      , 0.02892f // RX
      , -0.013f // RY
      , 0.0f // RZ
  };

  INIT_ONLY_ONCE();

  float blk_ts = 55.56f;
  float firing_tss[] = 
  {
    0.0f,    0.0f,    0.0f,    0.0f,    1.13f,   1.13f,   1.13f,   1.13f,
    2.13f,   2.13f,   2.13f,   2.13f,   3.26,    3.26f,   3.26f,   3.26f,
    4.26f,   4.26f,   4.26f,   4.26f,   5.38f,   5.38f,   5.38f,   5.38f,
    6.38f,   6.38f,   6.38f,   6.38f,   7.51f,   7.51f,   7.51f,   7.51f,

    8.51f,   8.51f,   8.51f,   8.51f,   10.01f,  10.01f,  10.01f,  10.01f,
    11.38f,  11.38f,  11.38f,  11.38f,  13.31f,  13.31f,  13.31f,  13.31f,
    15.11f,  15.11f,  15.11f,  15.11f,  17.04f,  17.04f,  17.04f,  17.04f,
    18.85f,  18.85f,  18.85f,  18.85f,  21.14f,  21.14f,  21.14f,  21.14f,

    21.14f,  23.31f,  23.31f,  23.31f,  23.31f,  25.61f,  25.61f,  25.61f,
    25.61f,  27.78f,  27.78f,  27.78f,  27.78f,  30.07f,  30.07f,  30.07f,
    30.07f,  32.24f,  32.24f,  32.24f,  32.24f,  34.54f,  34.54f,  34.54f,
    34.54f,  36.7f,   36.7f,   36.7f,   36.7f,   38.64f,   38.64f,  38.64f,

    38.64f,  40.44f,  40.44f,  40.44f,  40.44f,  41.94f,   41.94f,  41.94f,
    41.94f,  43.3f,   43.3f,   43.3f,   43.3f,   44.8f,    44.8f,   44.8f,
    44.8f,   46.17f,  46.17f,  46.17f,  46.17f,  47.66f,   47.66f,  47.66f,
    47.66f,  49.03f,  49.03f,  49.03f,  49.03f,  50.53f,   50.53f,  53.771f
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
inline RSEchoMode DecoderRSP128<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode)
  {
    case 0x00:
    case 0x01:
    case 0x02:
      return RSEchoMode::ECHO_SINGLE;
    case 0x03:
    case 0x04:
    case 0x05:
    default:
      return RSEchoMode::ECHO_DUAL;
  }
}

template <typename T_PointCloud>
inline void DecoderRSP128<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RSP128DifopPkt& pkt = *(const RSP128DifopPkt*)(packet);
  this->template decodeDifopCommon<RSP128DifopPkt>(pkt);

  this->echo_mode_ = getEchoMode (pkt.return_mode);
  this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
    (this->blks_per_frame_ << 1) : this->blks_per_frame_;
}

template <typename T_PointCloud>
inline DecoderRSP128<T_PointCloud>::DecoderRSP128(const RSDecoderParam& param)
  : DecoderMech<T_PointCloud>(getConstParam(), param)
{
}

template <typename T_PointCloud>
inline bool DecoderRSP128<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
{
  if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
  {
    return internDecodeMsopPkt<SingleReturnBlockIterator<RSP128MsopPkt>>(pkt, size);
  }
  else
  {
    return internDecodeMsopPkt<ABDualReturnBlockIterator<RSP128MsopPkt>>(pkt, size);
  }
}

template <typename T_PointCloud>
template <typename T_BlockIterator>
inline bool DecoderRSP128<T_PointCloud>::internDecodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RSP128MsopPkt& pkt = *(const RSP128MsopPkt*)(packet);
  bool ret = false;

  this->temperature_ = parseTempInBe(&(pkt.header.temp)) * this->const_param_.TEMPERATURE_RES;
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
    pkt_ts = ts * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs (ts, (RSTimestampUTC*)&pkt.header.timestamp);
    }
  }

  T_BlockIterator iter(pkt, this->const_param_.BLOCKS_PER_PKT, this->mech_const_param_.BLOCK_DURATION,
      this->block_az_diff_, this->fov_blind_ts_diff_);

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RSP128MsopBlock& block = pkt.blocks[blk];

    if (memcmp(this->const_param_.BLOCK_ID, block.id, 1) != 0)
    {
      this->cb_excep_(Error(ERRCODE_WRONGMSOPBLKID));
      break;
    }

    int32_t block_az_diff;
    double block_ts_off;
    iter.get(blk, block_az_diff, block_ts_off);

    double block_ts = pkt_ts + block_ts_off;
    int32_t block_az = ntohs(block.azimuth);
    if (this->split_strategy_->newBlock(block_az))
    {
      this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
      this->first_point_ts_ = block_ts;
      ret = true;
    }

    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      const RSChannel& channel = block.channels[chan]; 

      double chan_ts = block_ts + this->mech_const_param_.CHAN_TSS[chan];
      int32_t angle_horiz = block_az + 
        (int32_t)((float)block_az_diff * this->mech_const_param_.CHAN_AZIS[chan]);

      int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, angle_horiz);
      float distance = ntohs(channel.distance) * this->const_param_.DISTANCE_RES;

      if (this->distance_section_.in(distance) && this->scan_section_.in(angle_horiz_final))
      {
        float x =  distance * COS(angle_vert) * COS(angle_horiz_final) + this->mech_const_param_.RX * COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - this->mech_const_param_.RX * SIN(angle_horiz);
        float z =  distance * SIN(angle_vert) + this->mech_const_param_.RZ;
        this->transformPoint(x, y, z);

        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.intensity);
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

      this->prev_point_ts_ = chan_ts;
    }
  }

  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

}  // namespace lidar
}  // namespace robosense
