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

  void regPointCloudCallback(
      const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud);
  void regPacketCallback(const std::function<void(const Packet&)>& cb_put_pkt);
  void regImuDataCallback(
    const std::function<std::shared_ptr<ImuData>(void)>& cb_get_imu_data,
    const std::function<void(const std::shared_ptr<ImuData>& msg)>& cb_put_imu_data);
  void regImageDataCallback(
    const std::function<std::shared_ptr<ImageData>(void)>& cb_get_image_data,
    const std::function<void(const std::shared_ptr<ImageData>& msg)>& cb_put_image_data);
  void regExceptionCallback(const std::function<void(const Error&)>& cb_excep);
 
  bool init(const RSDriverParam& param);
  bool start();
  void stop();

  void decodePacket(const Packet& pkt);
  bool getTemperature(float& temp);
  bool getDeviceInfo(DeviceInfo& info);
  bool getDeviceStatus(DeviceStatus& status);

private:
  void runPacketCallBack(uint8_t* data, size_t data_size, double timestamp, uint8_t is_difop, uint8_t is_frame_begin);
  void runExceptionCallback(const Error& error);

  std::shared_ptr<Buffer> packetGet(size_t size);
  void packetPut(std::shared_ptr<Buffer> pkt, bool stuffed);

  std::shared_ptr<Buffer> packet2Get(size_t size);
  void packet2Put(std::shared_ptr<Buffer> pkt, bool stuffed);

  std::shared_ptr<Buffer> packet3Get(size_t size);
  void packet3Put(std::shared_ptr<Buffer> pkt, bool stuffed);

  void processPacket();
  void processPacket2();
  void processPacket3();
  void internalProcessPacket(std::shared_ptr<Buffer> pkt);
  
  std::shared_ptr<ImuData> getImuData();
  void putImuData();

  std::shared_ptr<ImageData> getImageData();
  void putImageData();

  std::shared_ptr<T_PointCloud> getPointCloud();
  void splitFrame(uint16_t height, double ts);
  void setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, uint16_t height, double chan_ts);

  RSDriverParam driver_param_;
  std::function<std::shared_ptr<T_PointCloud>(void)> cb_get_cloud_;
  std::function<void(std::shared_ptr<T_PointCloud>)> cb_put_cloud_;
  std::function<void(const Packet&)> cb_put_pkt_;
  std::function<void(const Packet&)> cb_put_pkt_2_;
  std::function<void(const Packet&)> cb_put_pkt_3_;
  std::function<std::shared_ptr<ImuData>(void)> cb_get_imu_data_;
  std::function<void(const std::shared_ptr<ImuData>& msg)> cb_put_imu_data_;
  std::function<std::shared_ptr<ImageData>(void)> cb_get_image_data_;
  std::function<void(const std::shared_ptr<ImageData>& msg)> cb_put_image_data_;
  std::function<void(const Error&)> cb_excep_;
  std::function<void(const uint8_t*, size_t)> cb_feed_pkt_;

  std::shared_ptr<Input> input_ptr_;
  std::shared_ptr<Decoder<T_PointCloud>> decoder_ptr_;
  SyncQueue<std::shared_ptr<Buffer>> free_pkt_queue_;
  SyncQueue<std::shared_ptr<Buffer>> pkt_queue_;
  SyncQueue<std::shared_ptr<Buffer>> free_pkt_queue_2_;
  SyncQueue<std::shared_ptr<Buffer>> pkt_queue_2_;
  SyncQueue<std::shared_ptr<Buffer>> free_pkt_queue_3_;
  SyncQueue<std::shared_ptr<Buffer>> pkt_queue_3_;
  std::thread handle_thread_;
  std::thread handle_thread_2_;
  std::thread handle_thread_3_;
  uint32_t pkt_seq_;
  uint32_t point_cloud_seq_;
  bool to_exit_handle_;
  bool init_flag_;
  bool start_flag_;
};

template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::LidarDriverImpl()
  : pkt_seq_(0), point_cloud_seq_(0), init_flag_(false), start_flag_(false)
{
}

template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::~LidarDriverImpl()
{
  stop();
}

template <typename T_PointCloud>
std::shared_ptr<ImuData> LidarDriverImpl<T_PointCloud>::getImuData()
{
  while (1)
  {
    std::shared_ptr<ImuData> imuData = cb_get_imu_data_();
    if (imuData)
    {
      imuData->state = false;
      return imuData;
    }
    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_IMUDATANULL)), 1);
  }
}

template <typename T_PointCloud>
std::shared_ptr<ImageData> LidarDriverImpl<T_PointCloud>::getImageData()
{
  while (1)
  {
    std::shared_ptr<ImageData> imageData = cb_get_image_data_();
    if (imageData)
    {
      imageData->state = false;
      return imageData;
    }
    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_IMUDATANULL)), 1);
  }
}

template <typename T_PointCloud>
std::shared_ptr<T_PointCloud> LidarDriverImpl<T_PointCloud>::getPointCloud()
{
  while (1)
  {
    std::shared_ptr<T_PointCloud> cloud = cb_get_cloud_();
    if (cloud)
    {
      cloud->points.resize(0);
      return cloud;
    }

    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_POINTCLOUDNULL)), 1);
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::regPointCloudCallback( 
    const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
    const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud) 
{
  cb_get_cloud_ = cb_get_cloud;
  cb_put_cloud_ = cb_put_cloud;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regPacketCallback(
    const std::function<void(const Packet&)>& cb_put_pkt)
{
  cb_put_pkt_ = cb_put_pkt;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regImuDataCallback(
  const std::function<std::shared_ptr<ImuData>(void)>& cb_get_imu_data,
  const std::function<void(const std::shared_ptr<ImuData>& msg)>& cb_put_imu_data)
{
  cb_get_imu_data_ = cb_get_imu_data;
  cb_put_imu_data_ = cb_put_imu_data;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regImageDataCallback(
  const std::function<std::shared_ptr<ImageData>(void)>& cb_get_image_data, 
  const std::function<void(const std::shared_ptr<ImageData>& msg)>& cb_put_image_data)
{
  cb_get_image_data_ = cb_get_image_data;
  cb_put_image_data_ = cb_put_image_data;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regExceptionCallback(
    const std::function<void(const Error&)>& cb_excep)
{
  cb_excep_ = cb_excep;
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::init(const RSDriverParam& param)
{
  if (init_flag_)
  {
    return true;
  }

  //
  // decoder
  //
  decoder_ptr_ = DecoderFactory<T_PointCloud>::createDecoder(param.lidar_type, param.decoder_param);
  if(decoder_ptr_ == nullptr)
  {
    return false; 
  }
  
  // rewrite pkt timestamp or not ?
  decoder_ptr_->enableWritePktTs((cb_put_pkt_ == nullptr) ? false : true);

  // point cloud related
  decoder_ptr_->point_cloud_ = getPointCloud();
  decoder_ptr_->regCallback( 
      std::bind(&LidarDriverImpl<T_PointCloud>::runExceptionCallback, this, std::placeholders::_1),
      std::bind(&LidarDriverImpl<T_PointCloud>::splitFrame, this, std::placeholders::_1, std::placeholders::_2));

  if(cb_put_imu_data_)
  {
    decoder_ptr_->imuDataPtr_ = getImuData();
    decoder_ptr_->regImuCallback(std::bind(&LidarDriverImpl<T_PointCloud>::putImuData, this));
  }

  if(cb_put_image_data_ && param.input_param.enable_image)
  {
    decoder_ptr_->imageDataPtr_ = getImageData();
    decoder_ptr_->regImageCallback(std::bind(&LidarDriverImpl<T_PointCloud>::putImageData, this));
  }

  double packet_duration = decoder_ptr_->getPacketDuration();
  bool is_jumbo = isJumbo(param.lidar_type);

  //
  // input
  //
  input_ptr_ = InputFactory::createInput(param.input_type, param.input_param, is_jumbo, packet_duration, cb_feed_pkt_);
  if(input_ptr_ == nullptr)
  {
    return false; 
  }

  input_ptr_->regCallback(
      std::bind(&LidarDriverImpl<T_PointCloud>::runExceptionCallback, this, std::placeholders::_1), 
      std::bind(&LidarDriverImpl<T_PointCloud>::packetGet, this, std::placeholders::_1), 
      std::bind(&LidarDriverImpl<T_PointCloud>::packetPut, this, std::placeholders::_1, std::placeholders::_2));

#ifdef ENABLE_USB
  if(param.input_param.enable_image)
  {
    input_ptr_->regCallback2(
        std::bind(&LidarDriverImpl<T_PointCloud>::packet2Get, this, std::placeholders::_1),
        std::bind(&LidarDriverImpl<T_PointCloud>::packet2Put, this, std::placeholders::_1, std::placeholders::_2));
  }

  input_ptr_->regCallback3(
      std::bind(&LidarDriverImpl<T_PointCloud>::packet3Get, this, std::placeholders::_1),
      std::bind(&LidarDriverImpl<T_PointCloud>::packet3Put, this, std::placeholders::_1, std::placeholders::_2));
#endif

  if (!input_ptr_->init())
  {
    goto failInputInit;
  }

  driver_param_ = param;
  init_flag_ = true;
  return true;

failInputInit:
  input_ptr_.reset();
  decoder_ptr_.reset();
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
  handle_thread_ = std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processPacket, this));

#ifdef ENABLE_USB
  if(driver_param_.input_param.enable_image)
  {
    handle_thread_2_ = std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processPacket2, this));
  }
  handle_thread_3_ = std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processPacket3, this));
#endif
  if(!input_ptr_->start())
  {
    return false;
  }

  start_flag_ = true;
  return true;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::stop()
{
  if (!start_flag_)
  {
    return;
  }
  input_ptr_->stop();

  to_exit_handle_ = true;
  handle_thread_.join();

#ifdef ENABLE_USB
  if(driver_param_.input_param.enable_image)
  {
    handle_thread_2_.join();
  }
  handle_thread_3_.join();
#endif

  // clear all points before next session
  if (decoder_ptr_->point_cloud_)
  {
    decoder_ptr_->point_cloud_->points.clear();
  }

  start_flag_ = false;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::decodePacket(const Packet& pkt)
{
  cb_feed_pkt_(pkt.buf_.data(), pkt.buf_.size());
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getTemperature(float& temp)
{
  if (decoder_ptr_ == nullptr )
  {
    return false;
  }
  return decoder_ptr_->getTemperature(temp);
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getDeviceInfo(DeviceInfo& info)
{
  if (decoder_ptr_ == nullptr)
  {
    return false;
  }

  return decoder_ptr_->getDeviceInfo(info);
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getDeviceStatus(DeviceStatus& status)
{
  if (decoder_ptr_ == nullptr)
  {
    return false;
  }

  return decoder_ptr_->getDeviceStatus(status);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runPacketCallBack(uint8_t* data, size_t data_size,
    double timestamp, uint8_t is_difop, uint8_t is_frame_begin)
{
  if (cb_put_pkt_)
  {
    Packet pkt;
    pkt.timestamp = timestamp;
    pkt.is_difop = is_difop;
    pkt.is_frame_begin = is_frame_begin;
    pkt.seq = pkt_seq_++;
    pkt.frame_id = driver_param_.frame_id;

    pkt.buf_.resize(data_size);
    memcpy (pkt.buf_.data(), data, data_size);
    cb_put_pkt_(pkt);
  }
}


template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runExceptionCallback(const Error& error)
{
  if (cb_excep_)
  {
    cb_excep_(error);
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
inline void LidarDriverImpl<T_PointCloud>::packetPut(std::shared_ptr<Buffer> pkt, bool stuffed)
{
  constexpr static int PACKET_POOL_MAX = 1024;

  if (!stuffed)
  {
    free_pkt_queue_.push(pkt);
    return;
  }

  size_t sz = pkt_queue_.push(pkt);
  if (sz > PACKET_POOL_MAX)
  {
    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_PKTBUFOVERFLOW)), 1);
#ifdef ENABLE_USB
    pkt_queue_.pop();
#else
    pkt_queue_.clear();
#endif
  }
}

template <typename T_PointCloud>
inline std::shared_ptr<Buffer> LidarDriverImpl<T_PointCloud>::packet2Get(size_t size)
{
  std::shared_ptr<Buffer> pkt = free_pkt_queue_2_.pop();
  if (pkt.get() != NULL)
  {
    return pkt;
  }

  return std::make_shared<Buffer>(size);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::packet2Put(std::shared_ptr<Buffer> pkt, bool stuffed)
{
  constexpr static int PACKET_POOL_MAX = 512;

  if (!stuffed)
  {
    free_pkt_queue_2_.push(pkt);
    return;
  }

  size_t sz = pkt_queue_2_.push(pkt);
  if (sz > PACKET_POOL_MAX)
  {
    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_PKTBUFOVERFLOW)), 1);
#ifdef ENABLE_USB
    pkt_queue_2_.pop();
#else
    pkt_queue_2_.clear();
#endif
  }
}

template <typename T_PointCloud>
inline std::shared_ptr<Buffer> LidarDriverImpl<T_PointCloud>::packet3Get(size_t size)
{
  std::shared_ptr<Buffer> pkt = free_pkt_queue_3_.pop();
  if (pkt.get() != NULL)
  {
    return pkt;
  }

  return std::make_shared<Buffer>(size);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::packet3Put(std::shared_ptr<Buffer> pkt, bool stuffed)
{
  constexpr static int PACKET_POOL_MAX = 512;

  if (!stuffed)
  {
    free_pkt_queue_3_.push(pkt);
    return;
  }

  size_t sz = pkt_queue_3_.push(pkt);
  if (sz > PACKET_POOL_MAX)
  {
    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_PKTBUFOVERFLOW)), 1);
#ifdef ENABLE_USB
    pkt_queue_3_.pop();
#else
    pkt_queue_3_.clear();
#endif
  }
}


template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::internalProcessPacket(std::shared_ptr<Buffer> pkt)
{
  static const uint8_t msop_id[] = {0x55, 0xAA};
  static const uint8_t difop_id[] = {0xA5, 0xFF};
  static const uint8_t imu_id[] = {0xAA, 0x55};
  static const uint8_t image_id[] = {0xAA, 0x66};
  static const uint8_t depth_id[] = {0xAA, 0x77};

  uint8_t* id = pkt->data();
  if (memcmp(id, msop_id, sizeof(msop_id)) == 0)
  {
    bool pkt_to_split = decoder_ptr_->processMsopPkt(pkt->data(), pkt->dataSize());
    runPacketCallBack(pkt->data(), pkt->dataSize(), decoder_ptr_->prevPktTs(), false, pkt_to_split); // msop packet
  }
  else if(memcmp(id, difop_id, sizeof(difop_id)) == 0)
  {
    decoder_ptr_->processDifopPkt(pkt->data(), pkt->dataSize());
    runPacketCallBack(pkt->data(), pkt->dataSize(), 0, true, false); // difop packet
  }
  else if(memcmp(id, imu_id, sizeof(imu_id)) == 0)
  {
    decoder_ptr_->processImuPkt(pkt->data(), pkt->dataSize()); // imu packet
  }
  else if(memcmp(id, image_id, sizeof(image_id)) == 0)
  {
    decoder_ptr_->processImagePkt(pkt->data(), pkt->dataSize()); // image packet
    free_pkt_queue_2_.push(pkt);
    return;
  }
  else if(memcmp(id, depth_id, sizeof(depth_id)) == 0)
  {
    decoder_ptr_->processPcPkt(pkt->data(), pkt->dataSize()); // depth packet
    free_pkt_queue_3_.push(pkt);
    return;
  }

  free_pkt_queue_.push(pkt);
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processPacket()
{
  while (!to_exit_handle_)
  {
    std::shared_ptr<Buffer> pkt = pkt_queue_.popWait(500000);
    if (pkt.get() == NULL)
    {
      continue;
    }

    internalProcessPacket(pkt);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processPacket2()
{
  while (!to_exit_handle_)
  {
    std::shared_ptr<Buffer> pkt = pkt_queue_2_.popWait(500000);
    if (pkt.get() == NULL)
    {
      continue;
    }

    internalProcessPacket(pkt);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processPacket3()
{
  while (!to_exit_handle_)
  {
    std::shared_ptr<Buffer> pkt = pkt_queue_3_.popWait(500000);
    if (pkt.get() == NULL)
    {
      continue;
    }

    internalProcessPacket(pkt);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::putImuData()
{
  std::shared_ptr<ImuData> imuData = decoder_ptr_->imuDataPtr_;
  if (imuData->state && this->cb_put_imu_data_)
  {
    this->cb_put_imu_data_(imuData);
    decoder_ptr_->imuDataPtr_ = getImuData();
  }
  
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::putImageData()
{
  std::shared_ptr<ImageData> imageData = decoder_ptr_->imageDataPtr_;
  if (imageData->state && this->cb_put_image_data_)
  {
    this->cb_put_image_data_(imageData);
    decoder_ptr_->imageDataPtr_ = getImageData();
  }
  
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::splitFrame(uint16_t height, double ts)
{
  std::shared_ptr<T_PointCloud> cloud = decoder_ptr_->point_cloud_;
  if (cloud->points.size() > 0)
  {
    setPointCloudHeader(cloud, height, ts);
    cb_put_cloud_(cloud);
    decoder_ptr_->point_cloud_ = getPointCloud();
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
    msg->width = (uint32_t)msg->points.size();
  }
  else
  {
    msg->height = height;
    msg->width = (uint32_t)msg->points.size() / msg->height;
  }

  msg->frame_id = driver_param_.frame_id;
}

}  // namespace lidar
}  // namespace robosense
