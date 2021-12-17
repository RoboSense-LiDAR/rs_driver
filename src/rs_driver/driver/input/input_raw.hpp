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

#include <rs_driver/driver/input/input.hpp>

namespace robosense
{
namespace lidar
{
class InputRaw : public Input
{
public:

  virtual bool init(){return true;}
  virtual bool start(){return true;};
  virtual void stop(){}
  virtual ~InputRaw(){}

  void feedPacket(const uint8_t* data, size_t size);

  InputRaw(const RSInputParam& input_param, const std::function<void(const Error&)>& excb)
    : Input(input_param, excb), pcap_offset_(ETH_HDR_LEN), difop_filter_valid_(false)
  {
  }

private:
  pcap_t* pcap_;
  size_t pcap_offset_;
  std::string msop_filter_str_;
  std::string difop_filter_str_;
  bpf_program msop_filter_;
  bpf_program difop_filter_;
  bool difop_filter_valid_;
  long long msec_to_delay_;
};

inline void InputRaw::feedPacket(const uint8_t* data, size_t size)
{
  std::shared_ptr<Buffer> pkt = cb_get_(MAX_PKT_LEN);
  memcpy(pkt->data(), data, size);
  pkt->setData(0, size);
  pushPacket(pkt);
}

}  // namespace lidar
}  // namespace robosense
