/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
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
#include <rs_driver/msg/point_cloud_msg.h>
#include <rs_driver/msg/packet_msg.h>
#include <rs_driver/msg/scan_msg.h>
#include <rs_driver/utility/lock_queue.h>
#include <rs_driver/utility/thread_pool.hpp>
#include <rs_driver/utility/time.h>
#include <rs_driver/common/error_code.h>
#include <rs_driver/driver/input.hpp>
#include <rs_driver/driver/decoder/decoder_factory.hpp>
#define MAX_PACKETS_BUFFER_SIZE 100000
namespace robosense
{
namespace lidar
{
template <typename PointT>
class LidarDriverImpl
{
public:
  LidarDriverImpl()
    : init_flag_(false), start_flag_(false), difop_flag_(false), point_cloud_seq_(0), scan_seq_(0), ndifop_count_(0)
  {
  }

  ~LidarDriverImpl()
  {
    stop();
  }

  inline bool init(const RSDriverParam& param)
  {
    if (init_flag_)
    {
      return false;
    }
    driver_param_ = param;
    input_ptr_ = std::make_shared<Input>(driver_param_.input_param,
                                         std::bind(&LidarDriverImpl::reportError, this, std::placeholders::_1));
    input_ptr_->regRecvMsopCallback(std::bind(&LidarDriverImpl::msopCallback, this, std::placeholders::_1));
    input_ptr_->regRecvDifopCallback(std::bind(&LidarDriverImpl::difopCallback, this, std::placeholders::_1));
    if (!input_ptr_->init())
    {
      return false;
    }
    thread_pool_ptr_ = std::make_shared<ThreadPool>();
    point_cloud_ptr_ = typename PointCloudMsg<PointT>::PointCloudPtr(new typename PointCloudMsg<PointT>::PointCloud);
    scan_ptr_ = std::make_shared<ScanMsg>();
    init_flag_ = true;
    return true;
  }

  inline void initDecoderOnly(const RSDriverParam& param)
  {
    if (init_flag_)
    {
      return;
    }
    driver_param_ = param;
    thread_pool_ptr_ = std::make_shared<ThreadPool>();
    point_cloud_ptr_ = typename PointCloudMsg<PointT>::PointCloudPtr(new typename PointCloudMsg<PointT>::PointCloud);
    scan_ptr_ = std::make_shared<ScanMsg>();
    init_flag_ = true;
  }

  inline bool start()
  {
    if (start_flag_ || input_ptr_ == nullptr)
    {
      return false;
    }
    start_flag_ = true;
    return input_ptr_->start();
  }

  inline void stop()
  {
    if (input_ptr_ != nullptr)
    {
      input_ptr_->stop();
    }
    start_flag_ = false;
    msop_pkt_queue_.clear();
    difop_pkt_queue_.clear();
  }

  inline void regRecvCallback(const std::function<void(const PointCloudMsg<PointT>&)>& callback)
  {
    point_cloud_cb_vec_.emplace_back(callback);
  }

  inline void regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
  {
    msop_pkt_cb_vec_.emplace_back(callback);
  }

  inline void regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
  {
    difop_pkt_cb_vec_.emplace_back(callback);
  }

  inline void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
  {
    camera_trigger_cb_vec_.emplace_back(callback);
  }

  inline void regExceptionCallback(const std::function<void(const Error&)>& callback)
  {
    excb_.emplace_back(callback);
  }

  inline bool getLidarTemperature(double& input_temperature)
  {
    if (lidar_decoder_ptr_ != nullptr)
    {
      input_temperature = lidar_decoder_ptr_->getLidarTemperature();
      return true;
    }
    return false;
  }

  inline bool decodeMsopScan(const ScanMsg& scan_msg, PointCloudMsg<PointT>& point_cloud_msg)
  {
    if (lidar_decoder_ptr_ == nullptr)
    {
      lidar_decoder_ptr_ =
          DecoderFactory<PointT>::createDecoder(driver_param_.lidar_type, driver_param_, scan_msg.packets[0]);
      lidar_decoder_ptr_->regRecvCallback(
          std::bind(&LidarDriverImpl::localCameraTriggerCallback, this, std::placeholders::_1));
    }

    typename PointCloudMsg<PointT>::PointCloudPtr output_point_cloud_ptr =
        typename PointCloudMsg<PointT>::PointCloudPtr(new typename PointCloudMsg<PointT>::PointCloud);
    if (!difop_flag_ && driver_param_.wait_for_difop)
    {
      ndifop_count_++;
      if (ndifop_count_ > 20)
      {
        reportError(Error(ErrCode_NoDifopRecv));
        ndifop_count_ = 0;
      }
      point_cloud_msg.point_cloud_ptr = output_point_cloud_ptr;
      usleep(10000);
      return false;
    }

    std::vector<std::vector<PointT>> pointcloud_one_frame;
    int height = 1;
    pointcloud_one_frame.resize(scan_msg.packets.size());
#pragma omp parallel for
    for (uint32_t i = 0; i < scan_msg.packets.size(); i++)
    {
      std::vector<PointT> pointcloud_one_packet;
      RSDecoderResult ret =
          lidar_decoder_ptr_->processMsopPkt(scan_msg.packets[i].packet.data(), pointcloud_one_packet, height);
      switch (ret)
      {
        case RSDecoderResult::DECODE_OK:
        case RSDecoderResult::FRAME_SPLIT:
          pointcloud_one_frame[i] = std::move(pointcloud_one_packet);
          break;
        case RSDecoderResult::WRONG_PKT_HEADER:
          reportError(Error(ErrCode_WrongPktHeader));
          break;
        case RSDecoderResult::PKT_NULL:
          reportError(Error(ErrCode_PktNull));
          break;
        default:
          break;
      }
    }

    for (auto iter : pointcloud_one_frame)
    {
      output_point_cloud_ptr->insert(output_point_cloud_ptr->end(), iter.begin(), iter.end());
    }

    point_cloud_msg.point_cloud_ptr = output_point_cloud_ptr;
    point_cloud_msg.height = height;
    point_cloud_msg.width = point_cloud_msg.point_cloud_ptr->size() / point_cloud_msg.height;
    setPointCloudMsgHeader(point_cloud_msg);
    point_cloud_msg.timestamp = scan_msg.timestamp;
    if (point_cloud_msg.point_cloud_ptr->size() == 0)
    {
      reportError(Error(ErrCode_ZeroPoints));
      return false;
    }
    return true;
  }

  inline void decodeDifopPkt(const PacketMsg& msg)
  {
    if (lidar_decoder_ptr_ == nullptr)
    {
      return;
    }
    lidar_decoder_ptr_->processDifopPkt(msg.packet.data());
    difop_flag_ = true;
  }

private:
  inline void runCallBack(const ScanMsg& msg)
  {
    if (msg.seq != 0)
    {
      for (auto& it : msop_pkt_cb_vec_)
      {
        it(msg);
      }
    }
  }

  inline void runCallBack(const PacketMsg& msg)
  {
    for (auto& it : difop_pkt_cb_vec_)
    {
      it(msg);
    }
  }

  inline void runCallBack(const PointCloudMsg<PointT>& msg)
  {
    if (msg.seq != 0)
    {
      for (auto& it : point_cloud_cb_vec_)
      {
        it(msg);
      }
    }
  }

  inline void reportError(const Error& error)
  {
    for (auto& it : excb_)
    {
      it(error);
    }
  }

  void msopCallback(const PacketMsg& msg)
  {
    if (lidar_decoder_ptr_ == nullptr)
    {
      lidar_decoder_ptr_ =
          DecoderFactory<PointT>::createDecoder(driver_param_.lidar_type, driver_param_, msg, input_ptr_);
      lidar_decoder_ptr_->regRecvCallback(
          std::bind(&LidarDriverImpl::localCameraTriggerCallback, this, std::placeholders::_1));
    }
    if (msop_pkt_queue_.size() > MAX_PACKETS_BUFFER_SIZE)
    {
      reportError(Error(ErrCode_PktBufOverFlow));
      msop_pkt_queue_.clear();
    }
    msop_pkt_queue_.push(msg);
    if (msop_pkt_queue_.is_task_finished_.load())
    {
      msop_pkt_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([this]() { processMsop(); });
    }
  }

  void difopCallback(const PacketMsg& msg)
  {
    difop_pkt_queue_.push(msg);
    if (difop_pkt_queue_.is_task_finished_.load())
    {
      difop_pkt_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([this]() { processDifop(); });
    }
  }

  void processMsop()
  {
    if (!difop_flag_ && driver_param_.wait_for_difop)
    {
      ndifop_count_++;
      if (ndifop_count_ > 120)
      {
        reportError(Error(ErrCode_NoDifopRecv));
        ndifop_count_ = 0;
      }
      usleep(10000);
      msop_pkt_queue_.clear();
      msop_pkt_queue_.is_task_finished_.store(true);
      return;
    }

    while (msop_pkt_queue_.size() > 0)
    {
      PacketMsg pkt = msop_pkt_queue_.popFront();
      std::vector<PointT> pointcloud_one_packet;
      int height = 1;
      int ret = lidar_decoder_ptr_->processMsopPkt(pkt.packet.data(), pointcloud_one_packet, height);
      scan_ptr_->packets.emplace_back(std::move(pkt));
      if (ret == DECODE_OK || ret == FRAME_SPLIT)
      {
        point_cloud_ptr_->insert(point_cloud_ptr_->end(), pointcloud_one_packet.begin(), pointcloud_one_packet.end());
        if (ret == FRAME_SPLIT)
        {
          PointCloudMsg<PointT> msg(point_cloud_ptr_);
          msg.height = height;
          msg.width = point_cloud_ptr_->size() / msg.height;
          setPointCloudMsgHeader(msg);
          if (driver_param_.decoder_param.use_lidar_clock == true)
          {
            msg.timestamp = lidar_decoder_ptr_->getLidarTime(pkt.packet.data());
          }
          else
          {
            msg.timestamp = getTime();
          }
          if (msg.point_cloud_ptr->size() == 0)
          {
            reportError(Error(ErrCode_ZeroPoints));
          }
          else
          {
            runCallBack(msg);
          }
          setScanMsgHeader(*scan_ptr_);
          runCallBack(*scan_ptr_);
          point_cloud_ptr_.reset(new typename PointCloudMsg<PointT>::PointCloud);
          scan_ptr_.reset(new ScanMsg);
        }
      }
      else
      {
        reportError(Error(ErrCode_WrongPktHeader));
        usleep(100000);
      }
    }
    msop_pkt_queue_.is_task_finished_.store(true);
  }

  inline void localCameraTriggerCallback(const CameraTrigger& msg)
  {
    for (auto& it : camera_trigger_cb_vec_)
    {
      it(msg);
    }
  }

  inline void processDifop()
  {
    while (difop_pkt_queue_.size() > 0)
    {
      PacketMsg pkt = difop_pkt_queue_.popFront();
      decodeDifopPkt(pkt);
      runCallBack(pkt);
    }
    difop_pkt_queue_.is_task_finished_.store(true);
  }

  inline void setScanMsgHeader(ScanMsg& msg)
  {
    msg.timestamp = getTime();
    if (driver_param_.decoder_param.use_lidar_clock == true)
    {
      msg.timestamp = lidar_decoder_ptr_->getLidarTime(msg.packets.back().packet.data());
    }
    msg.seq = scan_seq_++;
    msg.frame_id = driver_param_.frame_id;
  }

  inline void setPointCloudMsgHeader(PointCloudMsg<PointT>& msg)
  {
    msg.seq = point_cloud_seq_++;
    msg.frame_id = driver_param_.frame_id;
    msg.is_dense = false;
  }

private:
  Queue<PacketMsg> msop_pkt_queue_;
  Queue<PacketMsg> difop_pkt_queue_;
  std::vector<std::function<void(const ScanMsg&)>> msop_pkt_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> difop_pkt_cb_vec_;
  std::vector<std::function<void(const PointCloudMsg<PointT>&)>> point_cloud_cb_vec_;
  std::vector<std::function<void(const CameraTrigger&)>> camera_trigger_cb_vec_;
  std::vector<std::function<void(const Error&)>> excb_;
  std::shared_ptr<std::thread> lidar_thread_ptr_;
  std::shared_ptr<DecoderBase<PointT>> lidar_decoder_ptr_;
  std::shared_ptr<Input> input_ptr_;
  std::shared_ptr<ThreadPool> thread_pool_ptr_;
  std::shared_ptr<ScanMsg> scan_ptr_;
  bool init_flag_;
  bool start_flag_;
  bool difop_flag_;
  uint32_t point_cloud_seq_;
  uint32_t scan_seq_;
  uint32_t ndifop_count_;
  RSDriverParam driver_param_;
  typename PointCloudMsg<PointT>::PointCloudPtr point_cloud_ptr_;
};

}  // namespace lidar
}  // namespace robosense
