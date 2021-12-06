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

template <typename T_Packet>
class SingleReturnPacketTraverser
{
public:

  SingleReturnPacketTraverser(const RSDecoderConstParam const_param, const T_Packet& pkt, double pkt_ts)
    : const_param_(const_param), pkt_(pkt), bi_(0), ci_(0)
  {
    // calc initial values
    blk_ts_ = pkt_ts; 
    blk_azi_ = pkt_.blocks[bi_].azimuth;
    blk_azi_diff_ = pkt.blocks[bi_+1].azimuth - pkt.blocks[bi_].azimuth;
  }

  void toNext()
  {
    ci_++;
    if (ci_ >= const_param_.CHANNELS_PER_BLOCK)
    {
      ci_ = 0;
      bi_++;

      // calc next values
      if (bi_ < const_param_.BLOCKS_PER_PKT)
      {
        blk_ts_ += const_param_.BLOCK_DURATION;
        blk_azi_ = pkt_.blocks[bi_].azimuth;
        if (bi_ < (const_param_.BLOCKS_PER_PKT - 1))
        {
          blk_azi_diff_ = pkt_.blocks[bi_+1].azimuth - pkt_.blocks[bi_].azimuth;
        }
      }
    }
  }

  bool isLast()
  {
    return (bi_ >= const_param_.BLOCKS_PER_PKT);
  }

  bool getValue(int16_t& azi, double& ts)
  {
    if (bi_ >= const_param_.BLOCKS_PER_PKT)
      return false;

    ts = blk_ts_ + const_param_.CHAN_TSS[ci_];
    azi = blk_azi_ + 
      blk_azi_diff_ * (const_param_.CHAN_TSS[ci_] / const_param_.BLOCK_DURATION);
    return true;
  }

  void getPos(uint16_t& blk, uint16_t& chan)
  {
    blk = bi_;
    chan = ci_;
  }

private:

  const RSDecoderConstParam const_param_;
  const T_Packet& pkt_;
  uint16_t bi_;
  uint16_t ci_;
  double blk_ts_;
  int16_t blk_azi_diff_;
  int16_t blk_azi_;
};

template <typename T_Packet>
class DualReturnPacketTraverser
{
public:

  DualReturnPacketTraverser(const RSDecoderConstParam const_param, const T_Packet& pkt, double pkt_ts)
    : const_param_(const_param), pkt_(pkt), bi_(0), ci_(0)
  {
    // calc initial values
    blk_ts_ = pkt_ts; 

    blk_azi_ = pkt_.blocks[bi_].azimuth;
    blk_azi_diff_ = pkt.blocks[bi_+2].azimuth - pkt.blocks[bi_].azimuth;
  }

  void toNext()
  {
    ci_++;
    if (ci_ >= const_param_.CHANNELS_PER_BLOCK)
    {
      ci_ = 0;
      bi_++;

      // calc next values
      if ((bi_ < const_param_.BLOCKS_PER_PKT) && (bi_ % 2 == 0))
      {
        blk_ts_ += const_param_.BLOCK_DURATION;

        blk_azi_ = pkt_.blocks[bi_].azimuth;
        if (bi_ < (const_param_.BLOCKS_PER_PKT-2))
        {
          blk_azi_diff_ = pkt_.blocks[bi_+2].azimuth - pkt_.blocks[bi_].azimuth;
        }
      }
    }
  }

  bool isLast()
  {
    return (bi_ >= const_param_.BLOCKS_PER_PKT);
  }

  bool getValue(int16_t& azi, double& ts)
  {
    if (bi_ >= const_param_.BLOCKS_PER_PKT)
      return false;

    ts = blk_ts_ + const_param_.CHAN_TSS[ci_];
    azi = blk_azi_ + 
      blk_azi_diff_ * (const_param_.CHAN_TSS[ci_] / const_param_.BLOCK_DURATION);
    return true;
  }

  void getPos(uint16_t& blk, uint16_t& chan)
  {
    blk = bi_;
    chan = ci_;
  }

private:

  const RSDecoderConstParam const_param_;
  const T_Packet& pkt_;
  uint16_t bi_;
  uint16_t ci_;
  double blk_ts_;
  int16_t blk_azi_diff_;
  int16_t blk_azi_;
};

}  // namespace lidar
}  // namespace robosense
