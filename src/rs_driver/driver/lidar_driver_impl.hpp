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
  void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  void regExceptionCallback(const std::function<void(const Error&)>& callback);
  bool decodeMsopScan(const ScanMsg& scan_msg, T_PointCloud& point_cloud_msg);
  void decodeDifopPkt(const PacketMsg& msg);
  bool getLidarTemperature(double& input_temperature);

private:

  void runCallBack(std::shared_ptr<T_PointCloud> msg);
  void runCallBack(const ScanMsg& msg);
  void runCallBack(const PacketMsg& msg);
  void reportError(const Error& error);
  void msopCallback(std::shared_ptr<Packet> msg);
  void difopCallback(std::shared_ptr<Packet> msg);
  std::shared_ptr<Packet> packetGet(size_t size);
  void packetPut(std::shared_ptr<Packet> pkt);
  void processMsop();
  void processDifop();
#if 0
  void setPointCloudHeader(T_PointCloud& msg, int height);
#endif
  void setScanMsgHeader(ScanMsg& msg);

private:
  RSDriverParam driver_param_;
  std::vector<std::function<void(const Error&)>> excb_;
  std::vector<std::function<void(const ScanMsg&)>> msop_pkt_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> difop_pkt_cb_vec_;
  std::shared_ptr<Input> input_ptr_;
  std::shared_ptr<DecoderBase<T_PointCloud>> lidar_decoder_ptr_;
  std::shared_ptr<T_PointCloud> point_cloud_;
  ScanMsg scan_ptr_;
  SyncQueue<std::shared_ptr<Packet>> free_pkt_queue_;
  SyncQueue<std::shared_ptr<Packet>> msop_pkt_queue_;
  SyncQueue<std::shared_ptr<Packet>> difop_pkt_queue_;
  std::thread msop_handle_thread_;
  std::thread difop_handle_thread_;
  bool to_exit_msop_handle_;
  bool to_exit_difop_handle_;
  bool init_flag_;
  bool start_flag_;
  bool difop_flag_;
  uint32_t scan_seq_;
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
  : init_flag_(false), start_flag_(false), difop_flag_(false), scan_seq_(0), ndifop_count_(0)
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

  driver_param_ = param;

  lidar_decoder_ptr_ = DecoderFactory<T_PointCloud>::createDecoder(driver_param_);

  input_ptr_ = InputFactory::createInput(
      driver_param_, std::bind(&LidarDriverImpl<T_PointCloud>::reportError, this, std::placeholders::_1));
  input_ptr_->regRecvCallback(std::bind(&LidarDriverImpl<T_PointCloud>::packetGet, this, std::placeholders::_1),
                              std::bind(&LidarDriverImpl<T_PointCloud>::msopCallback, this, std::placeholders::_1),
                              std::bind(&LidarDriverImpl<T_PointCloud>::difopCallback, this, std::placeholders::_1));

  if (!input_ptr_->init())
  {
    goto failInputInit;
  }

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

  to_exit_msop_handle_ = false;
  msop_handle_thread_ = std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processMsop, this));
  to_exit_difop_handle_ = false;
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
    to_exit_msop_handle_ = true;
    to_exit_difop_handle_ = true;
    msop_handle_thread_.join();
    difop_handle_thread_.join();

    start_flag_ = false;
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regRecvCallback(const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put,
    const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get)
{
#if 0
  point_cloud_cb_put_vec_.emplace_back(cb_put);
  if (cb_get != nullptr) 
  {
    point_cloud_cb_get_ = cb_get;
  }
#endif
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  msop_pkt_cb_vec_.emplace_back(callback);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  difop_pkt_cb_vec_.emplace_back(callback);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regExceptionCallback(const std::function<void(const Error&)>& callback)
{
  excb_.emplace_back(callback);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::decodeDifopPkt(const PacketMsg& pkt)
{
//  lidar_decoder_ptr_->processDifopPkt(pkt.packet.data(), pkt.packet.size());
//  difop_flag_ = true;
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
inline void LidarDriverImpl<T_PointCloud>::runCallBack(const ScanMsg& msg)
{
  if (msg.seq != 0)
  {
    for (auto& it : msop_pkt_cb_vec_)
    {
      it(msg);
    }
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runCallBack(const PacketMsg& msg)
{
  for (auto& it : difop_pkt_cb_vec_)
  {
    it(msg);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runCallBack(std::shared_ptr<T_PointCloud> msg)
{
#if 0
  if (msg->seq != 0)
  {
    for (auto& it : point_cloud_cb_put_vec_)
    {
      it(msg);
    }
  }
#endif
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::reportError(const Error& error)
{
  for (auto& it : excb_)
  {
    it(error);
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
  free_pkt_queue_.push(pkt);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::msopCallback(std::shared_ptr<Packet> pkt)
{
  static const int PACKET_POOL_MAX = 1024;
  size_t sz = msop_pkt_queue_.push(pkt);
  if (sz > PACKET_POOL_MAX)
  {
    reportError(Error(ERRCODE_PKTBUFOVERFLOW));
    msop_pkt_queue_.clear();
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::difopCallback(std::shared_ptr<Packet> pkt)
{
  difop_pkt_queue_.push(pkt);
}

#if 0
template <typename T_PointCloud>
inline std::shared_ptr<T_PointCloud> LidarDriverImpl<T_PointCloud>::getPointCloud()
{
  std::shared_ptr<T_PointCloud> pc;

  while (1)
  {
    if (point_cloud_cb_get_ != nullptr)
      pc = point_cloud_cb_get_();

    if (pc)
    {
      pc->points.resize(0);
      return pc;
    }

    reportError(Error(ERRCODE_POINTCLOUDNULL));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  };
}
#endif

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processMsop()
{
#if 0
  //point_cloud_ = getPointCloud();

  while (!to_exit_msop_handle_)
  {
    std::shared_ptr<Packet> pkt = msop_pkt_queue_.popWait(1000);
    if (pkt.get() == NULL)
      continue;

    if (!difop_flag_ && driver_param_.wait_for_difop)
    {
      ndifop_count_++;
      if (ndifop_count_ > 240)
      {
        reportError(Error(ERRCODE_NODIFOPRECV));
        ndifop_count_ = 0;
      }

      packetPut(pkt);
      continue;
    }

    int height = 1;
    int ret = lidar_decoder_ptr_->processMsopPkt(pkt->data(), pkt->len(), point_cloud_->points, height);

#ifdef ENABLE_PUBLISH_RAW_MSG
    PacketMsg msg(pkt->len());
    memcpy (msg.packet.data(), pkt->data(), pkt->len());
    scan_ptr_.packets.emplace_back(msg);
#endif

    if (ret >= 0)
    {
      if (ret == FRAME_SPLIT)
      {
        T_PointCloud& msg = *point_cloud_;
        setPointCloudHeader(msg, height);
        if (driver_param_.decoder_param.use_lidar_clock == true)
        {
          msg.timestamp = lidar_decoder_ptr_->getLidarTime(pkt->data());
        }
        else
        {
          msg.timestamp = getTime();
        }

        if (msg.points.size() == 0)
        {
          reportError(Error(ERRCODE_ZEROPOINTS));
        }
        else
        {
          runCallBack(point_cloud_);
        }

#ifdef ENABLE_PUBLISH_RAW_MSG
        setScanMsgHeader(scan_ptr_);
        runCallBack(scan_ptr_);
        scan_ptr_.packets.resize(0);
#endif

        //point_cloud_ = getPointCloud();
      }
    }
    else if (ret < 0)
    {
      if (ret == WRONG_PKT_LENGTH)
        reportError(Error(ERRCODE_WRONGPKTLENGTH));
      else
        reportError(Error(ERRCODE_WRONGPKTHEADER));

      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    packetPut(pkt);
  }
#endif
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processDifop()
{
  while (!to_exit_difop_handle_)
  {
    std::shared_ptr<Packet> pkt = difop_pkt_queue_.popWait(500000);
    if (pkt.get() == NULL)
      continue;

    lidar_decoder_ptr_->processDifopPkt(pkt->data(), pkt->len());
    difop_flag_ = true;

#ifdef ENABLE_PUBLISH_RAW_MSG
    PacketMsg msg(pkt->len());
    memcpy (msg.packet.data(), pkt->data(), pkt->len());
    runCallBack(msg);
#endif

    packetPut(pkt);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::setScanMsgHeader(ScanMsg& msg)
{
  msg.timestamp = getTime();
  if (driver_param_.decoder_param.use_lidar_clock == true)
  {
    msg.timestamp = lidar_decoder_ptr_->getLidarTime(msg.packets.back().packet.data());
  }
  msg.seq = scan_seq_++;
  //msg.frame_id = driver_param_.frame_id;
}

#if 0
template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::setPointCloudHeader(T_PointCloud& msg, int height)
{
  msg.seq = point_cloud_seq_++;
  msg.frame_id = driver_param_.frame_id;
  msg.is_dense = driver_param_.decoder_param.is_dense;
  if (msg.is_dense)
  {
    msg.height = 1;
    msg.width = msg.points.size();
  }
  else
  {
    msg.height = height;
    msg.width = msg.points.size() / msg.height;
  }
}
#endif

}  // namespace lidar
}  // namespace robosense
