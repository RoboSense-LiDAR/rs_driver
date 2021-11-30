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
#include <rs_driver/macro/version.h>
#include <rs_driver/msg/packet.h>
#include <rs_driver/msg/packet_msg.h>
#include <rs_driver/msg/scan_msg.h>
#include <rs_driver/utility/time.h>
#include <rs_driver/common/error_code.h>
#include <rs_driver/utility/sync_queue.h>
#include <rs_driver/driver/input/input_factory.hpp>
#include <rs_driver/driver/decoder/decoder_factory.hpp>

#include <sstream>

namespace robosense
{
namespace lidar
{

template <typename T_PointCloud>
class LidarDriverImpl
{
public:

  static std::string getVersion();

  LidarDriverImpl();
  ~LidarDriverImpl();

  bool init(const RSDriverParam& param);
  bool start();
  void stop();
  void regRecvCallback(const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put, 
      const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get);
  void regRecvCallback(const std::function<void(const uint8_t*, size_t)>& callback);
  void regExceptionCallback(const std::function<void(const Error&)>& callback);
  void decodePacket(const uint8_t* pkt, size_t size);
  bool getLidarTemperature(double& input_temperature);

private:

  void runCallBack(std::shared_ptr<Packet> pkt);
  void reportError(const Error& error);

  std::shared_ptr<Packet> packetGet(size_t size);
  void packetPut(std::shared_ptr<Packet> pkt);

  void processMsop();
  void processDifop();

private:
  RSDriverParam driver_param_;
  std::function<void(const uint8_t*, size_t)> pkt_cb_;
  std::function<void(const Error&)> err_cb_;
  std::function<void(const uint8_t*, size_t)> feed_pkt_cb_;
  std::shared_ptr<Decoder<T_PointCloud>> lidar_decoder_ptr_;
  std::shared_ptr<Input> input_ptr_;
  SyncQueue<std::shared_ptr<Packet>> free_pkt_queue_;
  SyncQueue<std::shared_ptr<Packet>> msop_pkt_queue_;
  SyncQueue<std::shared_ptr<Packet>> difop_pkt_queue_;
  std::thread msop_handle_thread_;
  std::thread difop_handle_thread_;
  bool to_exit_handle_;
  bool init_flag_;
  bool start_flag_;
  uint32_t pkt_seq_;
  uint32_t ndifop_count_;
};

template <typename T_PointCloud>
inline std::string LidarDriverImpl<T_PointCloud>::getVersion()
{
  std::stringstream stream;
  stream << RSLIDAR_VERSION_MAJOR << "."
    << RSLIDAR_VERSION_MINOR << "."
    << RSLIDAR_VERSION_PATCH;

  return stream.str();
}

template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::LidarDriverImpl()
  : init_flag_(false), start_flag_(false), pkt_seq_(0), ndifop_count_(0)
{
}

template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::~LidarDriverImpl()
{
  stop();
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::init(const RSDriverParam& param)
{
  if (init_flag_)
  {
    return true;
  }

  lidar_decoder_ptr_ = DecoderFactory<T_PointCloud>::createDecoder
    (param.lidar_type, param.decoder_param);

  input_ptr_ = InputFactory::createInput(
      param.input_type, param.input_param, std::bind(&LidarDriverImpl<T_PointCloud>::reportError, this, std::placeholders::_1), 0);
  input_ptr_->regRecvCallback(std::bind(&LidarDriverImpl<T_PointCloud>::packetGet, this, std::placeholders::_1),
                              std::bind(&LidarDriverImpl<T_PointCloud>::packetPut, this, std::placeholders::_1));

  if (param.input_type == InputType::RAW_PACKET)
  {
    InputRaw* inputRaw = dynamic_cast<InputRaw*>(input_ptr_.get());
    feed_pkt_cb_ = std::bind(&InputRaw::feedPacket, inputRaw, std::placeholders::_1, std::placeholders::_2);
  }

  if (!input_ptr_->init())
  {
    goto failInputInit;
  }

  driver_param_ = param;
  init_flag_ = true;
  return true;

failInputInit:
  input_ptr_.reset();
  lidar_decoder_ptr_.reset();
  return false;
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::start()
{
  if (start_flag_)
    return true;

  if (!init_flag_)
    return false;

  to_exit_handle_ = false;
  msop_handle_thread_ = std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processMsop, this));
  difop_handle_thread_ = std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processDifop, this));

  input_ptr_->start();

  start_flag_ = true;
  return true;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::stop()
{
  if (input_ptr_ != nullptr)
    input_ptr_->stop();

  if (start_flag_)
  {
    to_exit_handle_ = true;
    msop_handle_thread_.join();
    difop_handle_thread_.join();

    start_flag_ = false;
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regRecvCallback(const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put,
    const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get)
{

  lidar_decoder_ptr_->regRecvCallback(cb_put, cb_get);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regRecvCallback(const std::function<void(const uint8_t*, size_t)>& callback)
{
  pkt_cb_ = callback;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regExceptionCallback(const std::function<void(const Error&)>& callback)
{
  err_cb_ = callback;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::decodePacket(const uint8_t* pkt, size_t size)
{
  feed_pkt_cb_(pkt, size);
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getLidarTemperature(double& input_temperature)
{
  if (lidar_decoder_ptr_ != nullptr)
  {
    input_temperature = lidar_decoder_ptr_->getLidarTemperature();
    return true;
  }
  return false;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runCallBack(std::shared_ptr<Packet> pkt)
{
  if (pkt_cb_)
  {
    pkt_cb_(pkt->data(), pkt->dataSize());
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::reportError(const Error& error)
{
  if (err_cb_)
  {
    err_cb_(error);
  }
}

template <typename T_PointCloud>
inline std::shared_ptr<Packet> LidarDriverImpl<T_PointCloud>::packetGet(size_t size)
{
  std::shared_ptr<Packet> pkt = free_pkt_queue_.pop();
  if (pkt.get() != NULL)
  {
    return pkt;
  }

  return std::make_shared<Packet>(size);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::packetPut(std::shared_ptr<Packet> pkt)
{
  SyncQueue<std::shared_ptr<Packet>>* queue; 

  uint8_t* id = pkt->data();
  if (*id == 0x55)
  {
    queue = &msop_pkt_queue_;
  }
  else if (*id == 0xA5)
  {
    queue = &difop_pkt_queue_;
  }

  static const int PACKET_POOL_MAX = 1024;

  size_t sz = queue->push(pkt);
  if (sz > PACKET_POOL_MAX)
  {
    reportError(Error(ERRCODE_PKTBUFOVERFLOW));
    queue->clear();
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processMsop()
{
  while (!to_exit_handle_)
  {
    std::shared_ptr<Packet> pkt = msop_pkt_queue_.popWait(1000);
    if (pkt.get() == NULL)
      continue;

    lidar_decoder_ptr_->processMsopPkt(pkt->data(), pkt->dataSize());
    runCallBack(pkt);

    free_pkt_queue_.push(pkt);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processDifop()
{
  while (!to_exit_handle_)
  {
    std::shared_ptr<Packet> pkt = difop_pkt_queue_.popWait(500000);
    if (pkt.get() == NULL)
      continue;

    lidar_decoder_ptr_->processDifopPkt(pkt->data(), pkt->dataSize());

    runCallBack(pkt);

    free_pkt_queue_.push(pkt);
  }
}

}  // namespace lidar
}  // namespace robosense
