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
#include <rs_driver/driver/input/sock_input.hpp>
// #include <rs_driver/driver/input/pcap_input.hpp>

namespace robosense
{
namespace lidar
{
inline long long msecToDelay(LidarType type, double replay_rate)
{
  constexpr double RS16_PCAP_SLEEP_DURATION = 1200;     ///< us
  constexpr double RS32_PCAP_SLEEP_DURATION = 530;      ///< us
  constexpr double RSBP_PCAP_SLEEP_DURATION = 530;      ///< us
  constexpr double RS128_PCAP_SLEEP_DURATION = 100;     ///< us
  constexpr double RS80_PCAP_SLEEP_DURATION = 135;      ///< us
  constexpr double RSM1_PCAP_SLEEP_DURATION = 90;       ///< us
  constexpr double RSHELIOS_PCAP_SLEEP_DURATION = 530;  ///< us
  constexpr double RSROCK_PCAP_SLEEP_DURATION = 530;    ///< us TODO

  double duration;
  switch (type)
  {
    case LidarType::RS16:
      duration = RS16_PCAP_SLEEP_DURATION;
      break;
    case LidarType::RS32:
      duration = RS32_PCAP_SLEEP_DURATION;
      break;
    case LidarType::RSBP:
      duration = RSBP_PCAP_SLEEP_DURATION;
      break;
    case LidarType::RS128:
      duration = RS128_PCAP_SLEEP_DURATION;
      break;
    case LidarType::RS80:
      duration = RS80_PCAP_SLEEP_DURATION;
      break;
    case LidarType::RSM1:
      duration = RSM1_PCAP_SLEEP_DURATION;
      break;
    case LidarType::RSHELIOS:
      duration = RSHELIOS_PCAP_SLEEP_DURATION;
      break;
    case LidarType::RSROCK:
    default:
      duration = RSROCK_PCAP_SLEEP_DURATION;
      break;
  }

  return static_cast<long long>(duration / replay_rate);
}

class InputFactory
{
public:
  static std::shared_ptr<Input> createInput(const RSDriverParam& driver_param,
                                            const std::function<void(const Error&)>& excb);
};

inline std::shared_ptr<Input> InputFactory::createInput(const RSDriverParam& driver_param,
                                                        const std::function<void(const Error&)>& excb)
{
  const RSInputParam& input_param = driver_param.input_param;
  std::shared_ptr<Input> input;

  if (input_param.read_pcap)
  {
    long long msec = msecToDelay(driver_param.lidar_type, input_param.pcap_rate);
    // input = std::make_shared<PcapInput>(input_param, excb, msec);
  }
  else
  {
    input = std::make_shared<SockInput>(input_param, excb);
  }

  return input;
}

}  // namespace lidar
}  // namespace robosense
