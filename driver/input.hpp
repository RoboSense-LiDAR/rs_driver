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
#include <sstream>
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

typedef struct RSInput_Param
{
  std::string device_ip = "192.168.1.200";
  uint16_t msop_port = 6699;
  uint16_t difop_port = 7788;
  bool read_pcap = false;
  std::string pcap_file_dir = "";
} RSInput_Param;

class Input
{
public:
  Input(const RSInput_Param &_input_param)
  {
    prev_time_ = getTime();
    new_time_ = getTime();
    input_param_ = _input_param;
    if (input_param_.read_pcap)
    {
      //std::cout << "Opening PCAP file " << this->pcap_file_dir_ << std::endl;
      char errbuf[PCAP_ERRBUF_SIZE];
      if ((this->pcap_ = pcap_open_offline(input_param_.pcap_file_dir.c_str(), errbuf)) == NULL)
      {
        ERROR << "Error opening rslidar pcap file! Abort!" << REND;
        exit(1);
      }
      else
      {
        std::stringstream msop_filter;
        std::stringstream difop_filter;

        msop_filter << "src host " << input_param_.device_ip << " && ";
        difop_filter << "src host " << input_param_.device_ip << " && ";

        msop_filter << "udp dst port " << input_param_.msop_port;
        pcap_compile(pcap_, &this->pcap_msop_filter_, msop_filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
        difop_filter << "udp dst port " << input_param_.difop_port;
        pcap_compile(pcap_, &this->pcap_difop_filter_, difop_filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
      }

      this->msop_fd_ = -1;
      this->difop_fd_ = -1;
    }
    else
    {
      this->msop_fd_ = setSocket(input_param_.msop_port);
      this->difop_fd_ = setSocket(input_param_.difop_port);

      this->pcap_ = NULL;
    }
  }
  ~Input()
  {
    if (!input_param_.read_pcap)
    {
      close(this->msop_fd_);
      close(this->difop_fd_);
      input_param_.msop_port = 0;
      input_param_.difop_port = 0;
    }
    else
    {
      this->input_param_.pcap_file_dir.clear();
      pcap_close(this->pcap_);
    }
  }

  InputState getPacket(uint8_t *pkt, uint32_t timeout)
  {
    InputState res = InputState(0);

    if (pkt == NULL)
    {
      return INPUT_ERROR;
    }

    if (!this->input_param_.read_pcap)
    {
      fd_set rfds;
      struct timeval tmout;

      tmout.tv_sec = timeout / 1000;
      tmout.tv_usec = (timeout % 1000) * 1000;

      FD_ZERO(&rfds);
      FD_SET(this->msop_fd_, &rfds);
      FD_SET(this->difop_fd_, &rfds);

      int max_fd = std::max(this->msop_fd_, this->difop_fd_);
      int retval = select(max_fd + 1, &rfds, NULL, NULL, &tmout);

      if (retval == -1 && errno == EINTR)
      {
        res = INPUT_EXIT;
      }
      else if (retval == -1)
      {
        std::cerr << "select: " << std::strerror(errno) << std::endl;
        res = InputState(res | INPUT_ERROR);
      }
      else if (retval)
      {
        ssize_t n;
        if (FD_ISSET(this->msop_fd_, &rfds))
        {
          res = InputState(res | INPUT_MSOP);
          n = recvfrom(this->msop_fd_, pkt, RSLIDAR_PKT_LEN, 0, NULL, NULL);
        }
        else if (FD_ISSET(this->difop_fd_, &rfds))
        {
          res = InputState(res | INPUT_DIFOP);
          n = recvfrom(this->difop_fd_, pkt, RSLIDAR_PKT_LEN, 0, NULL, NULL);
        }
        else
        {
          return INPUT_ERROR;
        }

        if (n != RSLIDAR_PKT_LEN)
        {
          res = InputState(res | INPUT_ERROR);
        }
      }
    }
    else
    {
      int ret;
      struct pcap_pkthdr *header;
      const u_char *pkt_data;

      if ((ret = pcap_next_ex(this->pcap_, &header, &pkt_data)) >= 0)
      {
        if (!input_param_.device_ip.empty() && (0 != pcap_offline_filter(&pcap_msop_filter_, header, pkt_data)))
        {
          memcpy(pkt, pkt_data + 42, RSLIDAR_PKT_LEN);
          res = INPUT_MSOP;
        }
        else if (!input_param_.device_ip.empty() && (0 != pcap_offline_filter(&pcap_difop_filter_, header, pkt_data)))
        {
          new_time_ = getTime();
          memcpy(pkt, pkt_data + 42, RSLIDAR_PKT_LEN);
          res = INPUT_DIFOP;
          usleep(100000 - (new_time_ - prev_time_) * 1000000);
          prev_time_ = getTime();
        }
        else
        {
          res = INPUT_OK;
          usleep(10);
        }
      }
    }

    return res;
  }

private:
  int setSocket(uint16_t port)
  {
    int sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
      std::cerr << "socket: " << std::strerror(errno) << std::endl;
      return -1;
    }

    struct sockaddr_in my_addr;
    memset((char *)&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(port);
    my_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_fd, (struct sockaddr *)&my_addr, sizeof(my_addr)) < 0)
    {
      std::cerr << "bind: " << std::strerror(errno) << std::endl;
      return -1;
    }

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
      std::cerr << "setsockopt: " << std::strerror(errno) << std::endl;
      return -1;
    }
    return sock_fd;
  }
  RSInput_Param input_param_;

  int msop_fd_;
  int difop_fd_;
  pcap_t *pcap_;
  bpf_program pcap_msop_filter_;
  bpf_program pcap_difop_filter_;
  double prev_time_;
  double new_time_;
};
} // namespace sensor
} // namespace robosense
