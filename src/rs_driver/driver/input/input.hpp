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

#include <rs_driver/common/common_header.h>
#include <rs_driver/common/error_code.h>
#include <rs_driver/driver/driver_param.h>
#include <rs_driver/msg/packet.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <memory>
#include <cstdint>
#include <memory>
#include <string>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <array>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>
#include <iostream>
#include <chrono>

#define MAX_PKT_LEN 1500
#define ETH_HDR_LEN 42
#define VLAN_LEN 4
#define SOME_IP_LEN 16

namespace robosense
{
namespace lidar
{
class Input
{
public:
  Input(const RSInputParam& input_param, const std::function<void(const Error&)>& excb);

  inline void regRecvCallback(const std::function<std::shared_ptr<Packet>(size_t)>& cb_get,
                              const std::function<void(std::shared_ptr<Packet>)>& cb_put_msop,
                              const std::function<void(std::shared_ptr<Packet>)>& cb_put_difop);

  virtual bool init() = 0;
  virtual bool start() = 0;
  virtual void stop();
  virtual ~Input()
  {
  }

protected:
  inline void pushPacket(std::shared_ptr<Packet> pkt);

  RSInputParam input_param_;
  std::function<std::shared_ptr<Packet>(size_t size)> cb_get_;
  std::function<void(std::shared_ptr<Packet>)> cb_put_msop_;
  std::function<void(std::shared_ptr<Packet>)> cb_put_difop_;
  std::function<void(const Error&)> excb_;
  std::thread recv_thread_;
  bool to_exit_recv_;
  bool init_flag_;
  bool start_flag_;
};

inline Input::Input(const RSInputParam& input_param, const std::function<void(const Error&)>& excb)
  : input_param_(input_param), excb_(excb), to_exit_recv_(false), init_flag_(false), start_flag_(false)
{
}

inline void Input::regRecvCallback(const std::function<std::shared_ptr<Packet>(size_t)>& cb_get,
                                   const std::function<void(std::shared_ptr<Packet>)>& cb_put_msop,
                                   const std::function<void(std::shared_ptr<Packet>)>& cb_put_difop)
{
  cb_get_ = cb_get;
  cb_put_msop_ = cb_put_msop;
  cb_put_difop_ = cb_put_difop;
}

inline void Input::stop()
{
  if (start_flag_)
  {
    to_exit_recv_ = true;
    recv_thread_.join();

    start_flag_ = false;
  }
}

inline void Input::pushPacket(std::shared_ptr<Packet> pkt)
{
  uint8_t* id = pkt->data();
  if (*id == 0x55)
  {
    cb_put_msop_(pkt);
  }
  else if (*id == 0xA5)
  {
    cb_put_difop_(pkt);
  }
}

}  // namespace lidar
}  // namespace robosense
