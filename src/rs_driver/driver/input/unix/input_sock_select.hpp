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

#ifdef __QNX__
#define FD_SETSIZE 1024
#include <pthread.h>
#include <sched.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>
#include <rs_driver/driver/input/unix/qnx_recvmmsg.hpp>
#endif

#include <rs_driver/driver/input/input.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <vector>
#include <memory>
#include <cerrno>
#include <cstring> 

namespace robosense
{
namespace lidar
{
class InputSock : public Input
{
public:
  InputSock(const RSInputParam& input_param) : Input(input_param), pkt_buf_len_(ETH_LEN), sock_offset_(0), sock_tail_(0)
  {
    sock_offset_ += input_param.user_layer_bytes;
    sock_tail_ += input_param.tail_layer_bytes;
  }

  virtual bool init();
  virtual bool start();
  virtual ~InputSock();

private:
  inline void recvPacket();
  inline int createSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp);

protected:
  size_t pkt_buf_len_;
  int fds_[3]{ -1 };
  size_t sock_offset_;
  size_t sock_tail_;
};

inline bool InputSock::init()
{
  if (init_flag_)
  {
    return true;
  }

  int msop_fd = -1, difop_fd = -1, imu_fd = -1;

  msop_fd = createSocket(input_param_.msop_port, input_param_.host_address, input_param_.group_address);
  if (msop_fd < 0)
    goto failMsop;

  if ((input_param_.difop_port != 0) && (input_param_.difop_port != input_param_.msop_port))
  {
    difop_fd = createSocket(input_param_.difop_port, input_param_.host_address, input_param_.group_address);
    if (difop_fd < 0)
      goto failDifop;
  }
  fds_[0] = msop_fd;
  fds_[1] = difop_fd;

  if ((input_param_.imu_port != 0) && (input_param_.imu_port != input_param_.msop_port) &&
      (input_param_.imu_port != input_param_.difop_port))
  {
    imu_fd = createSocket(input_param_.imu_port, input_param_.host_address, input_param_.group_address);
    if (imu_fd < 0)
      goto failImu;
  }

  fds_[2] = imu_fd;

  init_flag_ = true;
  return true;

failImu:
  close(difop_fd);

failDifop:
  close(msop_fd);
failMsop:
  return false;
}

inline bool InputSock::start()
{
  if (start_flag_)
  {
    return true;
  }

  if (!init_flag_)
  {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputSock::recvPacket, this));

  start_flag_ = true;
  return true;
}

inline InputSock::~InputSock()
{
  stop();

  close(fds_[0]);
  if (fds_[1] >= 0)
    close(fds_[1]);

  if (fds_[2] >= 0)
    close(fds_[2]);
}

inline int InputSock::createSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp)
{
  int fd;
  int ret;
  int reuse = 1;

  fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    perror("socket: ");
    goto failSocket;
  }

  ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  if (ret < 0)
  {
    perror("setsockopt(SO_REUSEADDR): ");
    goto failOption;
  }

  struct sockaddr_in host_addr;
  memset(&host_addr, 0, sizeof(host_addr));
  host_addr.sin_family = AF_INET;
  host_addr.sin_port = htons(port);
  host_addr.sin_addr.s_addr = INADDR_ANY;
  if (hostIp != "0.0.0.0" && grpIp == "0.0.0.0")
  {
    inet_pton(AF_INET, hostIp.c_str(), &(host_addr.sin_addr));
  }
  if (hostIp != "0.0.0.0" && grpIp != "0.0.0.0")
  {
    inet_pton(AF_INET, grpIp.c_str(), &(host_addr.sin_addr));
  }

  ret = bind(fd, (struct sockaddr*)&host_addr, sizeof(host_addr));
  if (ret < 0)
  {
    perror("bind: ");
    goto failBind;
  }

  if (grpIp != "0.0.0.0")
  {
    struct ip_mreq ipm;
    inet_pton(AF_INET, grpIp.c_str(), &(ipm.imr_multiaddr));
    inet_pton(AF_INET, hostIp.c_str(), &(ipm.imr_interface));
    ret = setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &ipm, sizeof(ipm));
    if (ret < 0)
    {
      perror("setsockopt(IP_ADD_MEMBERSHIP): ");
      goto failGroup;
    }
  }

  {
    uint32_t opt_val = input_param_.socket_recv_buf;
    uint32_t before_set_val = 0;
    uint32_t after_set_val = 0;
    if (opt_val < 4194304)
    {
      opt_val = 4194304;
    }
    socklen_t opt_len = sizeof(uint32_t);
    // get original value
    if (getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &before_set_val, &opt_len) == -1)
    {
      perror("getsockopt before");
      return -1;
    }
    RS_INFO << "Original receive buffer size: " << before_set_val << " bytes" << RS_REND;
    // set new value
    if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &opt_val, opt_len) == -1)
    {
      perror("setsockopt");
      return -1;
    }
    // get new value
    if (getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &after_set_val, &opt_len) == -1)
    {
      perror("getsockopt after");
      return -1;
    }
    RS_INFO << "After setting: receive buffer size: " << after_set_val << " bytes" << RS_REND;
  }
  {
    int flags = fcntl(fd, F_GETFL, 0);
    ret = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    if (ret < 0)
    {
      perror("fcntl: ");
      goto failNonBlock;
    }
  }

  return fd;

failNonBlock:
failGroup:
failBind:
failOption:
  close(fd);
failSocket:
  return -1;
}

inline void InputSock::recvPacket()
{
#ifdef __QNX__
  struct sched_param param;
  param.sched_priority = 50;
  int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (ret != 0) {
    RS_WARNING << "Recv_thread_ Failed to set high priority (Error: " << ret << "). "
              << "Running with normal priority." << RS_REND;
  }
  // 0100 = 0x04
  unsigned run_mask = 0x04;
  ThreadCtl(_NTO_TCTL_RUNMASK, (void *)run_mask);
#endif
int max_fd = std::max(std::max(fds_[0], fds_[1]), fds_[2]);
  if (max_fd >= FD_SETSIZE) {
    RS_ERROR << "Socket fd exceeds FD_SETSIZE limit: " << FD_SETSIZE << RS_REND;
    return;
  }

  constexpr int VLEN = 16;
  struct mmsghdr msgs[VLEN];
  struct iovec iovs[VLEN];
  std::vector<std::shared_ptr<Buffer>> pkt_pool(VLEN);

  memset(msgs, 0, sizeof(msgs));
  for (int i = 0; i < VLEN; i++) {
    iovs[i].iov_base = nullptr;
    iovs[i].iov_len = 0;
    msgs[i].msg_hdr.msg_name = NULL; 
    msgs[i].msg_hdr.msg_namelen = 0;
    msgs[i].msg_hdr.msg_iov = &iovs[i];
    msgs[i].msg_hdr.msg_iovlen = 1;
    msgs[i].msg_hdr.msg_control = NULL;
    msgs[i].msg_hdr.msg_controllen = 0;
    msgs[i].msg_hdr.msg_flags = 0;
  }

  while (!to_exit_recv_)
  {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fds_[0], &rfds);
    if (fds_[1] >= 0)
      FD_SET(fds_[1], &rfds);

    if (fds_[2] >= 0)
    {
      FD_SET(fds_[2], &rfds);
    }
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);
    if (retval == 0)
    {
      cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
      continue;
    }
    else if (retval < 0)
    {
      if (errno == EINTR)
        continue;
      perror("select: ");
      break;
    }

    for (int i = 0; i < 3; i++)
    {
      if ((fds_[i] >= 0) && FD_ISSET(fds_[i], &rfds))
      {
        for (int k = 0; k < VLEN; ++k) {
          if (pkt_pool[k] == nullptr) {
            pkt_pool[k] = cb_get_pkt_(pkt_buf_len_);
          }
          iovs[k].iov_base = pkt_pool[k]->buf();
          iovs[k].iov_len = pkt_pool[k]->bufSize();
        }
        int n_pkts = recvmmsg(fds_[i], msgs, VLEN, MSG_DONTWAIT, NULL);
        if (n_pkts < 0)
        {
           if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
              perror("recvmmsg: ");
           }
           continue;
        }
        
        for (int k = 0; k < n_pkts; k++) 
        {
          ssize_t data_len = msgs[k].msg_len;
          if (data_len > (ssize_t)(sock_offset_ + sock_tail_)) 
          {
            pkt_pool[k]->setData(sock_offset_, data_len - sock_offset_ - sock_tail_);
            pushPacket(pkt_pool[k]);
            pkt_pool[k] = nullptr; 
          }
        }
      }
    }
  }
}

}  // namespace lidar
}  // namespace robosense