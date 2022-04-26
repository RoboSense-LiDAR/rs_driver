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
#include <rs_driver/driver/input/input_raw.hpp>
#ifdef ENABLE_PCAP_PARSE
#include <rs_driver/driver/input/input_pcap.hpp>
#endif

#ifdef __linux__
#include <rs_driver/driver/input/unix/input_sock.hpp>
#elif _WIN32
#include <rs_driver/driver/input/win/input_sock.hpp>
#endif

namespace robosense
{
namespace lidar
{

class InputFactory
{
public:
  static std::shared_ptr<Input> createInput(InputType type, const RSInputParam& param,
      double sec_to_delay, std::function<void(const uint8_t*, size_t)>& cb_feed_pkt);
};

inline std::shared_ptr<Input> InputFactory::createInput(InputType type, const RSInputParam& param, 
    double sec_to_delay, std::function<void(const uint8_t*, size_t)>& cb_feed_pkt)
{
  std::shared_ptr<Input> input;

  switch(type)
  {
    case InputType::ONLINE_LIDAR:
      {
        input = std::make_shared<InputSock>(param);
      }
      break;

#ifdef ENABLE_PCAP_PARSE
    case InputType::PCAP_FILE:
      {
        input = std::make_shared<InputPcap>(param, sec_to_delay);
      }
      break;
#endif

    case InputType::RAW_PACKET:
      {
        std::shared_ptr<InputRaw> inputRaw = std::make_shared<InputRaw>(param);
        cb_feed_pkt = std::bind(&InputRaw::feedPacket, inputRaw, 
            std::placeholders::_1, std::placeholders::_2);
        input = inputRaw;
      }
      break;

    default:
      RS_ERROR << "Wrong Input Type." << RS_REND;
      exit(-1);
  }

  return input;
}

}  // namespace lidar
}  // namespace robosense
