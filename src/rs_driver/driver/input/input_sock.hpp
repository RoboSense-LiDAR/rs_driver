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

#ifdef __linux__

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#elif _WIN32

#include <winsock2.h>
#include <windows.h>

#endif

#include <unistd.h>
#include <fcntl.h>

namespace robosense
{
namespace lidar
{
class InputSock : public Input
{
public:
  InputSock(const RSInputParam& input_param, const std::function<void(const Error&)>& excb)
    : Input(input_param, excb), sock_offset_(0)
  {
    if (input_param.use_someip)
      sock_offset_ += SOME_IP_LEN;
  }

  virtual bool init();
  virtual bool start();
  virtual ~InputSock();

private:
  inline void recvPacket();
  inline void higherThreadPrioty(std::thread::native_handle_type handle);
  inline int createSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp);

private:
  int fds_[2];
  size_t sock_offset_;
};

inline void InputSock::higherThreadPrioty(std::thread::native_handle_type handle)
{
#ifdef ENABLE_HIGH_PRIORITY_THREAD
  int policy;
  sched_param sch;
  pthread_getschedparam(handle, &policy, &sch);

  sch.sched_priority = 63;
  if (pthread_setschedparam(handle, SCHED_RR, &sch))
  {
    std::cout << "setschedparam failed: " << std::strerror(errno) << '\n';
  }
#endif
}

inline bool InputSock::init()
{
  if (init_flag_)
    return true;

  int msop_fd = -1, difop_fd = -1;

  msop_fd = createSocket(input_param_.msop_port, input_param_.host_address, input_param_.multi_cast_address);
  if (msop_fd < 0)
    goto failMsop;

  if ((input_param_.difop_port != 0) && (input_param_.difop_port != input_param_.msop_port))
  {
    difop_fd = createSocket(input_param_.difop_port, input_param_.host_address, input_param_.multi_cast_address);
    if (difop_fd < 0)
      goto failDifop;
  }

  fds_[0] = msop_fd;
  fds_[1] = difop_fd;

  init_flag_ = true;
  return true;

failDifop:
  close(msop_fd);
failMsop:
  return false;
}

inline bool InputSock::start()
{
  if (start_flag_)
    return true;

  if (!init_flag_)
  {
    excb_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  recv_thread_ = std::thread(std::bind(&InputSock::recvPacket, this));

  higherThreadPrioty(recv_thread_.native_handle());

  start_flag_ = true;
  return true;
}

inline InputSock::~InputSock()
{
  stop();

  close(fds_[0]);
  if (fds_[1] >= 0)
    close(fds_[1]);
}

inline int InputSock::createSocket(uint16_t port, const std::string& hostIp, const std::string& grpIp)
{
  int fd;
  int flags;
  int ret;

  fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    std::cerr << "socket: " << std::strerror(errno) << std::endl;
    goto failSocket;
  }

  struct sockaddr_in host_addr;
  memset(&host_addr, 0, sizeof(host_addr));
  host_addr.sin_family = AF_INET;
  host_addr.sin_port = htons(port);
  host_addr.sin_addr.s_addr = INADDR_ANY;
  if (hostIp != "0.0.0.0" && grpIp == "0.0.0.0")
    inet_pton(AF_INET, hostIp.c_str(), &(host_addr.sin_addr));

  ret = bind(fd, (struct sockaddr*)&host_addr, sizeof(host_addr));
  if (ret < 0)
  {
    std::cerr << "bind: " << std::strerror(errno) << std::endl;
    goto failBind;
  }

  if (grpIp != "0.0.0.0")
  {
    struct ip_mreqn ipm;
    memset(&ipm, 0, sizeof(ipm));
    inet_pton(AF_INET, grpIp.c_str(), &(ipm.imr_multiaddr));
    inet_pton(AF_INET, hostIp.c_str(), &(ipm.imr_address));
    ret = setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &ipm, sizeof(ipm));
    if (ret < 0)
    {
      std::cerr << "setsockopt: " << std::strerror(errno) << std::endl;
      goto failGroup;
    }
  }

  flags = fcntl(fd, F_GETFL, 0);
  ret = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  if (ret < 0)
  {
    std::cerr << "setsockopt: " << std::strerror(errno) << std::endl;
    goto failNonBlock;
  }

  return fd;

failNonBlock:
failGroup:
failBind:
  close(fd);
failSocket:
  return -1;
}

inline void InputSock::recvPacket()
{
  fd_set rfds;

  while (!to_exit_recv_)
  {
    FD_ZERO(&rfds);
    FD_SET(fds_[0], &rfds);
    if (fds_[1] >= 0)
      FD_SET(fds_[1], &rfds);
    int max_fd = std::max(fds_[0], fds_[1]);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;
    int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);
    if (retval == 0)
    {
      continue;
    }
    else if (retval == -1)
    {
      if (errno == EINTR)
        continue;

      std::cerr << "select: " << std::strerror(errno) << std::endl;
      break;
    }

    if (FD_ISSET(fds_[0], &rfds))
    {
      std::shared_ptr<Packet> pkt = cb_get_(MAX_PKT_LEN);
      pkt->resetData();
      ssize_t ret = recvfrom(fds_[0], pkt->data(), MAX_PKT_LEN, 0, NULL, NULL);
      if (ret <= 0)
      {
        std::cout << "recv failed" << std::endl;
        break;
      }

      pkt->setData(sock_offset_, ret - sock_offset_);
      pushPacket(pkt);
    }
    else if (FD_ISSET(fds_[1], &rfds))
    {
      std::shared_ptr<Packet> pkt = cb_get_(MAX_PKT_LEN);
      pkt->resetData();
      ssize_t ret = recvfrom(fds_[1], pkt->data(), MAX_PKT_LEN, 0, NULL, NULL);
      if (ret <= 0)
        break;

      pkt->setData(sock_offset_, ret - sock_offset_);
      pushPacket(pkt);
    }
  }
}

}  // namespace lidar
}  // namespace robosense
