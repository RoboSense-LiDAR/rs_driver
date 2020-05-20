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
#ifdef __GNUC__
#include <pcap.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif
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
#include "stdafx.h"
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
using boost::asio::deadline_timer;
using boost::asio::ip::udp;
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
  uint16_t msop_port = 6688;
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
		#if 0
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
		  #endif
        }
        else
        {
          this->msop_fd_ = setSocket(input_param_.msop_port);
         // this->difop_fd_ = setSocket(input_param_.difop_port);

 //     this->pcap_ = NULL;
    }
  }
  ~Input()
  {

    if (!input_param_.read_pcap)
    {
	
   //   close(this->msop_fd_);
   //   close(this->difop_fd_);
      input_param_.msop_port = 0;
      input_param_.difop_port = 0;
	  
    }
    else
    {
	#if 0
      this->input_param_.pcap_file_dir.clear();
      pcap_close(this->pcap_);
	  #endif
    }

  }

      InputState getPacket(uint8_t *pkt, uint32_t timeout)
      {
        deadline_->expires_from_now(boost::posix_time::seconds(1));
        boost::system::error_code ec = boost::asio::error::would_block;
        std::size_t ret = 0;
        char *pRecvBuffer = (char *)malloc(RSLIDAR_PKT_LEN);
        recv_sock_ptr_->async_receive(boost::asio::buffer(pRecvBuffer, RSLIDAR_PKT_LEN),
                                      boost::bind(&Input::handle_receive, _1, _2, &ec, &ret));
        do
        {
          io_service_.run_one();
        } while (ec == boost::asio::error::would_block);
        if (ec)
        {
          free(pRecvBuffer);
          return INPUT_ERROR;
        }
        memcpy(pkt, pRecvBuffer, RSLIDAR_PKT_LEN);
        free(pRecvBuffer);
        return INPUT_MSOP;
      }

    private:
      inline void check_deadline()
      {
        if (deadline_->expires_at() <= deadline_timer::traits_type::now())
        {
          recv_sock_ptr_->cancel();
          deadline_->expires_at(boost::posix_time::pos_infin);
        }
        deadline_->async_wait(boost::bind(&Input::check_deadline, this));
      }
      static void handle_receive(
          const boost::system::error_code &ec, std::size_t length,
          boost::system::error_code *out_ec, std::size_t *out_length)
      {
        *out_ec = ec;
        *out_length = length;
      }
      int setSocket(uint16_t port)
      {
        try
        {
          recv_sock_ptr_.reset(new udp::socket(io_service_, udp::endpoint(udp::v4(), port)));
          deadline_.reset(new deadline_timer(io_service_));
        }
        catch (...)
        {
          ERROR << "Proto Receiver Port is already used! Abort!" << REND;
          exit(-1);
        }
        deadline_->expires_at(boost::posix_time::pos_infin);
        check_deadline();
        return 0;
      }
      RSInput_Param input_param_;

      int msop_fd_;
      int difop_fd_;
	  #if 0
      pcap_t *pcap_;
      bpf_program pcap_msop_filter_;
      bpf_program pcap_difop_filter_;
	  #endif
      double prev_time_;
      double new_time_;
      std::unique_ptr<udp::socket> recv_sock_ptr_;
      std::unique_ptr<deadline_timer> deadline_;
      boost::asio::io_service io_service_;
    };
  } // namespace sensor
} // namespace robosense
