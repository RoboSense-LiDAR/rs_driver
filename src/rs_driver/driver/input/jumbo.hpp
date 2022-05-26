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

#include <rs_driver/utility/dbg.hpp>

namespace robosense
{
namespace lidar
{

#pragma pack(push, 1)

struct iphdr
{
  u_char version;
  u_char tos;
  u_short tot_len;

  u_short id;
  u_short frag_off;

  u_char ttl;
  u_char protocol;
  u_short check;

  u_int saddr;
  u_int daddr;
};

struct udphdr
{
  u_short source;
  u_short dest;
  u_short len;
  u_short check;
};

#pragma pack(pop)

class Jumbo
{
public:

  Jumbo()
    : ip_id_(0), buf_off_(0)
  {
  }

  u_char* buf()
  {
    return buf_;
  }

  u_short buf_len()
  {
    return buf_off_;
  }

  bool new_fragment(const u_char* pkt_data, size_t pkt_data_size);

  u_short dst_port();

private:

  u_short ip_id_;
  u_char buf_[IP_LEN];
  u_short buf_off_;
}; 

inline bool Jumbo::new_fragment(const u_char* pkt_data, size_t pkt_data_size)
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
  //u_short ip_hdr_size = ip_hdr->ihl * 4;
  u_short ip_hdr_size = (ip_hdr->version & 0xf) * 4;
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
#if 0
        printf ("--- end new packet. ip_id:0x%x, len:%d\n", ip_id_, buf_off_);

        for (uint16_t off = UDP_HDR_LEN, i = 0; off < buf_len(); off += 984, i++)
        {
          char title[32];
          sprintf (title, "jumbo %d ", i);

          hexdump (buf() + off, 32, title);
        }
#endif

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

#if 0
      printf ("+++ start packet 0x%x\n", ip_id_);
#endif
    }
  }

  return false;
}

inline u_short Jumbo::dst_port()
{
  const struct udphdr * udp_hdr = (const struct udphdr*)buf_;
  return ntohs(udp_hdr->dest);
}

}  // namespace lidar
}  // namespace robosense
