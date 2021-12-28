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
  uint8_t id[2];
  uint16_t azimuth;
  RSChannel channels[32];
} RS16MsopBlock;

typedef struct
{
  RSMsopHeaderV1 header;
  RS16MsopBlock blocks[12];
  unsigned int index;
  uint16_t tail;
} RS16MsopPkt;

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV1 eth;
  RSFOV fov;
  uint8_t reserved0[2];
  uint16_t phase_lock_angle;
  RSVersionV1 version;
  uint8_t reserved1[242];
  RSSN sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  uint16_t sw_ver;
  RSTimestampYMD timestamp;
  RSStatusV1 status;
  uint8_t reserved2[5];
  RSDiagnoV1 diagno;
  uint8_t gprmc[86];
  uint8_t reserved3[697];
  uint8_t pitch_cali[48];
  uint8_t reserved4[33];
  uint16_t tail;
} RS16DifopPkt;

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV1 eth;
  RSFOV fov;
  uint8_t reserved0[2];
  uint16_t phase_lock_angle;
  RSVersionV1 version;
  uint8_t reserved1[242];
  RSSN sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  uint16_t sw_ver;
  RSTimestampYMD timestamp;
  RSStatusV1 status;
  uint8_t reserved2[5];
  RSDiagnoV1 diagno;
  uint8_t gprmc[86];
  RSCalibrationAngle vert_angle_cali[32];
  RSCalibrationAngle horiz_angle_cali[32];
} AdapterRS16DifopPkt;

#pragma pack(pop)

inline void RS16DifopPkt2Adapter (const uint8_t* difop)
{
  RS16DifopPkt& orig = *(RS16DifopPkt*)difop;
  AdapterRS16DifopPkt& adapter = *(AdapterRS16DifopPkt*)difop;

  for (uint16_t i = 0, j = 16; i < 16; i++, j++)
  {
    uint32_t v = 0;
    v += orig.pitch_cali[i*3];
    v = v << 8;
    v += orig.pitch_cali[i*3 + 1];
    v = v << 8;
    v += orig.pitch_cali[i*3 + 2];

    uint16_t v2 = (uint16_t)(v * 0.01); // higher resolution to lower one.

    adapter.vert_angle_cali[i].sign = (i < 8) ? 1 : 0;
    adapter.vert_angle_cali[i].value = htons(v2);
    adapter.horiz_angle_cali[i].sign = 0;
    adapter.horiz_angle_cali[i].value = 0;

    adapter.vert_angle_cali[j].sign = adapter.vert_angle_cali[i].sign;
    adapter.vert_angle_cali[j].value = adapter.vert_angle_cali[i].value;
    adapter.horiz_angle_cali[j].sign = 0;
    adapter.horiz_angle_cali[j].value = 0;
  }
}

class DecoderRS16 : public DecoderMech
{
public: 

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual void decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual ~DecoderRS16() = default;

  explicit DecoderRS16(const RSDecoderParam& param, 
      const std::function<void(const Error&)>& excb);

#ifndef UNIT_TEST
protected:
#endif

  static RSDecoderMechConstParam& initConstParam();
  static RSEchoMode getEchoMode(uint8_t mode);

  template <typename T_BlockDiff>
  void internDecodeMsopPkt(const uint8_t* pkt, size_t size);
};

inline RSDecoderMechConstParam& DecoderRS16::initConstParam()
{
  static RSDecoderMechConstParam param = 
  {
    1248 // msop len
      , 1248 // difop len
      , 8 // msop id len
      , 8 // difop id len
      , {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0xFF, 0xEE} // block id
    , 12 // blocks per packet
      , 32 // channels per block. how many channels in the msop block.
      , 0.2f // distance min
      , 150.0f // distance max
      , 0.005f // distance resolution
      , 0.0625f // temperature resolution

      // lens center
      , 0.03825f // RX
      , -0.01088f // RY
      , 0.0f // RZ
  };

  INIT_ONLY_ONCE();

  float blk_ts = 55.5f * 2;
  float firing_tss[] = 
  {
    0.00f,  2.80f,  5.60f,  8.40f, 11.20f, 14.00f, 16.80f, 19.60f, 
    22.40f, 25.20f, 28.00f, 30.80f, 33.60f, 36.40f, 39.20f, 42.00f,
    55.50f, 58.30f, 61.10f, 63.90f, 66.70f, 69.50f, 72.30f, 75.10f,
    77.90f, 80.70f, 83.50f, 86.30f, 89.10f, 91.90f, 94.70f, 97.50f
  };

  param.BLOCK_DURATION = blk_ts / 1000000;
  for (uint16_t i = 0; i < sizeof(firing_tss)/sizeof(firing_tss[0]); i++)
  {
    param.CHAN_TSS[i] = (double)firing_tss[i] / 1000000;
    param.CHAN_AZIS[i] = firing_tss[i] / blk_ts;
  }

  return param;
}

inline RSEchoMode DecoderRS16::getEchoMode(uint8_t mode)
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

inline DecoderRS16::DecoderRS16(const RSDecoderParam& param,
      const std::function<void(const Error&)>& excb)
  : DecoderMech(initConstParam(), param, excb)
{
  this->height_ = 16;
}

inline void DecoderRS16::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  RS16DifopPkt2Adapter (packet);

  const AdapterRS16DifopPkt& pkt = *(const AdapterRS16DifopPkt*)(packet);
  this->template decodeDifopCommon<AdapterRS16DifopPkt>(pkt);

  this->echo_mode_ = getEchoMode (pkt.return_mode);
  this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
    (this->blks_per_frame_ << 1) : this->blks_per_frame_;
}

inline void DecoderRS16::decodeMsopPkt(const uint8_t* pkt, size_t size)
{
  if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
  {
    internDecodeMsopPkt<SingleReturnBlockDiff<RS16MsopPkt>>(pkt, size);
  }
  else
  {
    internDecodeMsopPkt<DualReturnBlockDiff<RS16MsopPkt>>(pkt, size);
  }
}

template <typename T_BlockDiff>
inline void DecoderRS16::internDecodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RS16MsopPkt& pkt = *(const RS16MsopPkt*)(packet);

  this->temperature_ = parseTemp(&(pkt.header.temp)) * this->const_param_.TEMPERATURE_RES;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeYMD(&pkt.header.timestamp) * 0.000001;
  }
  else
  {
    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = getTimeHost() * 0.000001 - this->getPacketDuration();
  }

  T_BlockDiff diff(pkt, this->const_param_.BLOCKS_PER_PKT, this->mech_const_param_.BLOCK_DURATION);

  double block_ts = pkt_ts;
  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RS16MsopBlock& block = pkt.blocks[blk];

    if (memcmp(this->const_param_.BLOCK_ID, block.id, 2) != 0)
    {
      this->excb_(Error(ERRCODE_WRONGPKTHEADER));
      break;
    }

    int32_t block_az = ntohs(block.azimuth);
    block_ts += diff.ts(blk);
    int32_t block_azi_diff = diff.azimuth(blk);

    if (this->split_strategy_->newBlock(block_az))
    {
      this->cb_split_(this->height_, this->prev_point_ts_);
    }

    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      const RSChannel& channel = block.channels[chan]; 

      double chan_ts = block_ts + this->mech_const_param_.CHAN_TSS[chan];
      int32_t angle_horiz = block_az + 
        (int32_t)((float)block_azi_diff * this->mech_const_param_.CHAN_AZIS[chan]);

      int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, angle_horiz);
      float distance = ntohs(channel.distance) * this->const_param_.DISTANCE_RES;

      if (this->distance_section_.in(distance) && this->scan_section_.in(angle_horiz_final))
      {
        RSPoint point;
        point.x =  distance * COS(angle_vert) * COS(angle_horiz_final) + this->mech_const_param_.RX * COS(angle_horiz);
        point.y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - this->mech_const_param_.RX * SIN(angle_horiz);
        point.z =  distance * SIN(angle_vert) + this->mech_const_param_.RZ;
        point.intensity = channel.intensity;
        point.timestamp = chan_ts;
        point.ring = (this->chan_angles_.toUserChan(chan) >> 1);

        this->cb_new_point_(point);
      }
      else if (!this->param_.dense_points)
      {
        RSPoint point;
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
        point.intensity = 0;
        point.timestamp = chan_ts;
        point.ring = (this->chan_angles_.toUserChan(chan) >> 1);

        this->cb_new_point_(point);
      }

      this->prev_point_ts_ = chan_ts;
    }

  }
}

}  // namespace lidar
}  // namespace robosense
