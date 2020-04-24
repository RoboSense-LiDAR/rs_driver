/*
 * @Author: your name
 * @Date: 2020-01-20 17:41:33
 * @LastEditTime : 2020-01-20 17:41:41
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /rs_sdk/modules/rs_lidar/include/rs_lidar/input.h
 */
/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#pragma once
#include <rs_common/common.h>
#include <cstdint>
#include <memory>
#include <string>
#include <pcap.h>
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
namespace robosense
{
namespace sensor
{
const int RSLIDAR_PKT_LEN = 1248;

enum InputState
{
  INPUT_OK = 0,
  INPUT_TIMEOUT = 1,
  INPUT_ERROR = 2,
  INPUT_DIFOP = 4,
  INPUT_MSOP = 8,
  INPUT_EXIT = 16
};

class Input : public common::CommonBase
{
public:
  Input(const YAML::Node &yaml_param);
  ~Input();

  InputState getPacket(uint8_t *pkt, uint32_t timeout);

private:
  int setUpSocket(uint16_t port);

  uint16_t msop_port_;
  uint16_t difop_port_;
  int msop_fd_;
  int difop_fd_;

  pcap_t *pcap_;
  bpf_program pcap_msop_filter_;
  bpf_program pcap_difop_filter_;
  double prev_time_;
  double new_time_;

  std::string device_ip_;
  std::string pcap_file_dir_;
  bool read_pcap_;
};
} // namespace sensor
} // namespace robosense
