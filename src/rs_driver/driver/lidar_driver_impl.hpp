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
#include <rs_driver/driver/driver_param.hpp>
#include <rs_driver/msg/packet.hpp>
#include <rs_driver/common/error_code.hpp>
#include <rs_driver/macro/version.hpp>
#include <rs_driver/utility/sync_queue.hpp>

#include <rs_driver/utility/buffer.hpp>
#include <rs_driver/driver/input/input_factory.hpp>
#include <rs_driver/driver/decoder/decoder_factory.hpp>

#include <sstream>

namespace robosense
{
namespace lidar
{

inline std::string getDriverVersion()
{
  std::stringstream stream;
  stream << RSLIDAR_VERSION_MAJOR << "."
    << RSLIDAR_VERSION_MINOR << "."
    << RSLIDAR_VERSION_PATCH;

  return stream.str();
}

template <typename T_PointCloud>
class LidarDriverImpl
{
public:

  LidarDriverImpl();
  ~LidarDriverImpl();

  void regRecvCallback(const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_pc,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_pc);
  void regRecvCallback(const std::function<void(const Packet&)>& cb_put_pkt);
  void regExceptionCallback(const std::function<void(const Error&)>& cb_excp);
 
  bool init(const RSDriverParam& param);
  bool start();
  void stop();

  void decodePacket(const Packet& pkt);
  bool getTemperature(float& temp);

private:

  void runCallBack(std::shared_ptr<Buffer> pkt, double timestamp, uint8_t is_difop, uint8_t is_frame_begin);
  void reportError(const Error& error);

  std::shared_ptr<Buffer> packetGet(size_t size);
  void packetPut(std::shared_ptr<Buffer> pkt);

  void processMsop();
  void processDifop();

  std::shared_ptr<T_PointCloud> getPointCloud();
  void newPoint(const RSPoint& point);
  void splitFrame(uint16_t height, double ts);
  void setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, uint16_t height, double chan_ts);

  RSDriverParam driver_param_;
  std::function<std::shared_ptr<T_PointCloud>(void)> cb_get_pc_;
  std::function<void(std::shared_ptr<T_PointCloud>)> cb_put_pc_;
  std::function<void(const Packet&)> cb_put_pkt_;
  std::function<void(const Error&)> cb_excp_;
  std::function<void(const uint8_t*, size_t)> cb_feed_pkt_;

  std::shared_ptr<Input> input_ptr_;
  std::shared_ptr<Decoder> lidar_decoder_ptr_;
  SyncQueue<std::shared_ptr<Buffer>> free_pkt_queue_;
  SyncQueue<std::shared_ptr<Buffer>> msop_pkt_queue_;
  SyncQueue<std::shared_ptr<Buffer>> difop_pkt_queue_;
  std::thread msop_handle_thread_;
  std::thread difop_handle_thread_;
  bool to_exit_handle_;
  bool init_flag_;
  bool start_flag_;
  uint32_t pkt_seq_;

  std::shared_ptr<T_PointCloud> point_cloud_;
  uint32_t point_cloud_seq_;
};

template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::LidarDriverImpl()
  : init_flag_(false), start_flag_(false), pkt_seq_(0), point_cloud_seq_(0)
{
}

template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::~LidarDriverImpl()
{
  stop();
}

template <typename T_PointCloud>
std::shared_ptr<T_PointCloud> LidarDriverImpl<T_PointCloud>::getPointCloud()
{
  while (1)
  {
    std::shared_ptr<T_PointCloud> pc = cb_get_pc_();
    if (pc)
    {
      pc->points.resize(0);
      return pc;
    }

    reportError(Error(ERRCODE_POINTCLOUDNULL));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::regRecvCallback(
    const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get,
    const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put) 
{
  cb_get_pc_ = cb_get;
  cb_put_pc_ = cb_put;

  point_cloud_ = getPointCloud();
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regRecvCallback(
    const std::function<void(const Packet&)>& cb_put_pkt)
{
  cb_put_pkt_ = cb_put_pkt;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regExceptionCallback(
    const std::function<void(const Error&)>& cb_excp)
{
  cb_excp_ = cb_excp;
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::init(const RSDriverParam& param)
{
  if (init_flag_)
  {
    return true;
  }

  lidar_decoder_ptr_ = DecoderFactory::createDecoder(
      param.lidar_type, param.decoder_param, 
      std::bind(&LidarDriverImpl<T_PointCloud>::reportError, this, std::placeholders::_1));

  lidar_decoder_ptr_->regRecvCallback( 
      std::bind(&LidarDriverImpl<T_PointCloud>::newPoint, this, std::placeholders::_1), 
      std::bind(&LidarDriverImpl<T_PointCloud>::splitFrame, this, std::placeholders::_1, std::placeholders::_2));
  lidar_decoder_ptr_->enableWritePktTs((cb_put_pkt_ == nullptr) ? false : true);

  double packet_duration = lidar_decoder_ptr_->getPacketDuration();

  input_ptr_ = InputFactory::createInput(param.input_type, param.input_param, 
      std::bind(&LidarDriverImpl<T_PointCloud>::reportError, this, std::placeholders::_1), 
      packet_duration, cb_feed_pkt_);

  input_ptr_->regRecvCallback(
      std::bind(&LidarDriverImpl<T_PointCloud>::packetGet, this, std::placeholders::_1), 
      std::bind(&LidarDriverImpl<T_PointCloud>::packetPut, this, std::placeholders::_1));

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
  {
    return true;
  }

  if (!init_flag_)
  {
    return false;
  }

  to_exit_handle_ = false;
  msop_handle_thread_ = 
    std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processMsop, this));
  difop_handle_thread_ = 
    std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processDifop, this));

  input_ptr_->start();

  start_flag_ = true;
  return true;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::stop()
{
  if (input_ptr_ != nullptr)
  {
    input_ptr_->stop();
  }

  if (start_flag_)
  {
    to_exit_handle_ = true;
    msop_handle_thread_.join();
    difop_handle_thread_.join();

    start_flag_ = false;
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::decodePacket(const Packet& pkt)
{
  cb_feed_pkt_(pkt.buf_.data(), pkt.buf_.size());
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getTemperature(float& temp)
{
  if (lidar_decoder_ptr_ != nullptr)
  {
    temp = lidar_decoder_ptr_->getTemperature();
    return true;
  }
  return false;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runCallBack(std::shared_ptr<Buffer> buf,
    double timestamp, uint8_t is_difop, uint8_t is_frame_begin)
{
  if (cb_put_pkt_)
  {
    Packet pkt;
    pkt.is_difop = is_difop;
    pkt.is_frame_begin = is_frame_begin;
    pkt.seq = pkt_seq_++;

    pkt.buf_.resize(buf->dataSize());
    memcpy (pkt.buf_.data(), buf->data(), buf->dataSize());
    cb_put_pkt_(pkt);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::reportError(const Error& error)
{
  if (cb_excp_)
  {
    cb_excp_(error);
  }
}

template <typename T_PointCloud>
inline std::shared_ptr<Buffer> LidarDriverImpl<T_PointCloud>::packetGet(size_t size)
{
  std::shared_ptr<Buffer> pkt = free_pkt_queue_.pop();
  if (pkt.get() != NULL)
  {
    return pkt;
  }

  return std::make_shared<Buffer>(size);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::packetPut(std::shared_ptr<Buffer> pkt)
{
  SyncQueue<std::shared_ptr<Buffer>>* queue; 

  uint8_t* id = pkt->data();
  if (*id == 0x55)
  {
    queue = &msop_pkt_queue_;
  }
  else if (*id == 0xA5)
  {
    queue = &difop_pkt_queue_;
  }
  else
  {
    free_pkt_queue_.push(pkt);
    return;
  }

  constexpr static int PACKET_POOL_MAX = 1024;

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
    std::shared_ptr<Buffer> pkt = msop_pkt_queue_.popWait(1000);
    if (pkt.get() == NULL)
    {
      continue;
    }

    bool split = lidar_decoder_ptr_->processMsopPkt(pkt->data(), pkt->dataSize());
    runCallBack(pkt, 0, false, split); // msop packet

    free_pkt_queue_.push(pkt);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processDifop()
{
  while (!to_exit_handle_)
  {
    std::shared_ptr<Buffer> pkt = difop_pkt_queue_.popWait(500000);
    if (pkt.get() == NULL)
    {
      continue;
    }

    lidar_decoder_ptr_->processDifopPkt(pkt->data(), pkt->dataSize());
    runCallBack(pkt, 0, true, false); // difop packet

    free_pkt_queue_.push(pkt);
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::newPoint(const RSPoint& pt)
{
  typename T_PointCloud::PointT point;
  setX(point, pt.x);
  setY(point, pt.y);
  setZ(point, pt.z);
  setIntensity(point, pt.intensity);
  setRing(point, pt.ring);
  setTimestamp(point, pt.timestamp);

  point_cloud_->points.emplace_back(point);
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::splitFrame(uint16_t height, double ts)
{
  if (point_cloud_->points.size() > 0)
  {
    setPointCloudHeader(point_cloud_, height, ts);
    cb_put_pc_(point_cloud_);

    point_cloud_ = getPointCloud();
  }
  else
  {
    reportError(Error(ERRCODE_ZEROPOINTS));
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, 
    uint16_t height, double ts)
{
  msg->seq = point_cloud_seq_++;
  msg->timestamp = ts;
  msg->is_dense = driver_param_.decoder_param.dense_points;
  if (msg->is_dense)
  {
    msg->height = 1;
    msg->width = msg->points.size();
  }
  else
  {
    msg->height = height;
    msg->width = msg->points.size() / msg->height;
  }
}

}  // namespace lidar
}  // namespace robosense
