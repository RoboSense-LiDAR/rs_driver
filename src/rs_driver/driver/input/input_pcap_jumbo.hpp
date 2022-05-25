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
#include <rs_driver/utility/dbg.hpp>

#include <linux/ip.h>
#include <linux/udp.h>
#include <sstream>

#ifdef __linux__
#elif _WIN32
#define WIN32
#endif

#include <pcap.h>

namespace robosense
{
namespace lidar
{

class jumbo_ip_packet
{
public:

  jumbo_ip_packet()
    : ip_id_(0), buf_off_(0)
  {
  }

  bool new_fragment(const u_char* pkt_data, size_t pkt_data_size)
  {
    // Is it an ip packet ?
    const u_short* eth_type = (const u_short*)(pkt_data + 12);
    if (ntohs(*eth_type) != 0x0800)
      return false;

    // is it a udp packet?
    const struct iphdr* ip_hdr = (const struct iphdr*)(pkt_data + 14);
    if (ip_hdr->protocol != 0x11)
      return false;

    // ip data
    u_short ip_hdr_size = ip_hdr->ihl * 4;
    u_short ip_len = ntohs(ip_hdr->tot_len);

    const u_char* ip_data = pkt_data + 14 + ip_hdr_size;
    u_short ip_data_len = ip_len - ip_hdr_size;

    // ip fragment
    u_short ip_id = ntohs (ip_hdr->id);
    u_short f_off = ntohs(ip_hdr->frag_off);
    u_short frag_flags  = (f_off >> 13);
    u_short frag_off    = (f_off & 0x1fff) * 8; // 8 octet boudary

    if (ip_id == ip_id_)
    {
      if (frag_off == buf_off_)
      {
        memcpy (buf_ + buf_off_, ip_data, ip_data_len);
        buf_off_ += ip_data_len;

        if ((frag_flags & 0x1) == 0)
        {
          //printf ("--- end new packet. ip_id:0x%x, len:%d\n", ip_id_, buf_off_);

          ip_id_ = 0;
          return true;
        }
      }
    }
    else
    {
      if ((frag_off == 0) && ((frag_flags & 0x1) != 0))
      //if (frag_off == 0)
      {
        ip_id_ = ip_id;
        buf_off_ = 0;

        memcpy (buf_ + buf_off_, ip_data, ip_data_len);
        buf_off_ += ip_data_len;

        //printf ("+++ start packet 0x%x\n", ip_id_);
      }
    }

    return false;
  }

  u_char* buf()
  {
    return buf_;
  }

  u_short buf_len()
  {
    return buf_off_;
  }

private:

  u_short ip_id_;
  u_char buf_[IP_LEN];
  u_short buf_off_;
}; 

class InputPcapJumbo : public Input
{
public:
  InputPcapJumbo(const RSInputParam& input_param, double sec_to_delay)
    : Input(input_param), pcap_(NULL), pcap_offset_(ETH_HDR_LEN), pcap_tail_(0), difop_filter_valid_(false), 
    msec_to_delay_((uint64_t)(sec_to_delay*1000000))
  {
    if (input_param.use_vlan)
    {
      pcap_offset_ += VLAN_HDR_LEN;
    }

    pcap_offset_ += input_param.user_layer_bytes;
    pcap_tail_   += input_param.tail_layer_bytes;

    std::stringstream msop_stream, difop_stream;
    if (input_param_.use_vlan)
    {
      msop_stream << "vlan && ";
      difop_stream << "vlan && ";
    }

    msop_stream << "udp";
    difop_stream << "udp dst port " << input_param_.difop_port;

    msop_filter_str_ = msop_stream.str();
    difop_filter_str_ = difop_stream.str();
  }

  virtual bool init();
  virtual bool start();
  virtual ~InputPcapJumbo();

private:
  void recvPacket();

private:
  pcap_t* pcap_;
  size_t pcap_offset_;
  size_t pcap_tail_;
  std::string msop_filter_str_;
  std::string difop_filter_str_;
  bpf_program msop_filter_;
  bpf_program difop_filter_;
  bool difop_filter_valid_;
  uint64_t msec_to_delay_;

  jumbo_ip_packet jumbo_packet_;
};

inline bool InputPcapJumbo::init()
{
  if (init_flag_)
    return true;

  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
  if (pcap_ == NULL)
  {
    cb_excep_(Error(ERRCODE_PCAPWRONGPATH));
    return false;
  }

  pcap_compile(pcap_, &msop_filter_, msop_filter_str_.c_str(), 1, 0xFFFFFFFF);

  if ((input_param_.difop_port != 0) && (input_param_.difop_port != input_param_.msop_port))
  {
    pcap_compile(pcap_, &difop_filter_, difop_filter_str_.c_str(), 1, 0xFFFFFFFF);
    difop_filter_valid_ = true;
  }

  init_flag_ = true;
  return true;
}

inline bool InputPcapJumbo::start()
{
  if (start_flag_)
    return true;

  if (!init_flag_)
  {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputPcapJumbo::recvPacket, this));

  start_flag_ = true;
  return true;
}

inline InputPcapJumbo::~InputPcapJumbo()
{
  stop();

  if (pcap_ != NULL)
  {
    pcap_close(pcap_);
  }
}

inline void InputPcapJumbo::recvPacket()
{
  while (!to_exit_recv_)
  {
    struct pcap_pkthdr* header;
    const u_char* pkt_data;
    int ret = pcap_next_ex(pcap_, &header, &pkt_data);
    if (ret < 0)  // reach file end.
    {
      pcap_close(pcap_);

      if (input_param_.pcap_repeat)
      {
        cb_excep_(Error(ERRCODE_PCAPREPEAT));

        char errbuf[PCAP_ERRBUF_SIZE];
        pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
        continue;
      }
      else
      {
        cb_excep_(Error(ERRCODE_PCAPEXIT));
        break;
      }
    }

    if (pcap_offline_filter(&msop_filter_, header, pkt_data) != 0)
    {
      bool new_pkt = jumbo_packet_.new_fragment(pkt_data, header->len);
      if (new_pkt)
      {
        const struct udphdr * udp_hdr = (const struct udphdr*)jumbo_packet_.buf();
        u_short dst_port = ntohs(udp_hdr->dest);

        if (dst_port == input_param_.msop_port)
        {
#if 0
          for (uint16_t off = UDP_HDR_LEN; off < jumbo_packet_.buf_len(); off += 984)
          {
            hexdump (jumbo_packet_.buf() + off, 32, "jumbo_buf");
          }
#endif
          std::shared_ptr<Buffer> pkt = cb_get_pkt_(IP_LEN);
          memcpy(pkt->data(), jumbo_packet_.buf() + UDP_HDR_LEN, jumbo_packet_.buf_len() - UDP_HDR_LEN);
          pkt->setData(0, jumbo_packet_.buf_len() - UDP_HDR_LEN);
          pushPacket(pkt);
        }
      }
    }
    else if (difop_filter_valid_ && (pcap_offline_filter(&difop_filter_, header, pkt_data) != 0))
    {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(IP_LEN);
      memcpy(pkt->data(), pkt_data + pcap_offset_, header->len - pcap_offset_ - pcap_tail_);
      pkt->setData(0, header->len - pcap_offset_ - pcap_tail_);
      pushPacket(pkt);
    }
    else
    {
      continue;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(msec_to_delay_));
  }
}

}  // namespace lidar
}  // namespace robosense
