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
#include <rs_driver/msg/pointcloud_msg.h>
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
    : init_flag_(false), start_flag_(false), difop_flag_(false), points_seq_(0), scan_seq_(0), ndifop_count_(0)
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
    pointcloud_ptr_ = typename PointcloudMsg<PointT>::PointCloudPtr(new typename PointcloudMsg<PointT>::PointCloud);
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
    pointcloud_ptr_ = typename PointcloudMsg<PointT>::PointCloudPtr(new typename PointcloudMsg<PointT>::PointCloud);
    scan_ptr_ = std::make_shared<ScanMsg>();
    init_flag_ = true;
  }

  inline bool start()
  {
    if (start_flag_)
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
    msop_pkt_queue_.clear();
    difop_pkt_queue_.clear();
  }

  inline void regRecvCallback(const std::function<void(const PointcloudMsg<PointT>&)> _cb)
  {
    pointscb_.emplace_back(_cb);
  }

  inline void regRecvCallback(const std::function<void(const ScanMsg&)> _cb)
  {
    pkts_msop_cb_.emplace_back(_cb);
  }

  inline void regRecvCallback(const std::function<void(const PacketMsg&)> _cb)
  {
    pkts_difop_cb_.emplace_back(_cb);
  }

  inline void regExceptionCallback(const std::function<void(const Error&)> _excb)
  {
    excb_.emplace_back(_excb);
  }

  inline bool decodeMsopScan(const ScanMsg& pkt_scan_msg, PointcloudMsg<PointT>& point_msg)
  {
    if (lidar_decoder_ptr_ == nullptr)
    {
      lidar_decoder_ptr_ = DecoderFactory<PointT>::createDecoder(driver_param_.lidar_type, driver_param_.decoder_param,
                                                                 pkt_scan_msg.packets[0]);
      lidar_decoder_ptr_->loadCalibrationFile(driver_param_.angle_path);
    }

    typename PointcloudMsg<PointT>::PointCloudPtr output_pointcloud_ptr =
        typename PointcloudMsg<PointT>::PointCloudPtr(new typename PointcloudMsg<PointT>::PointCloud);
    if (!difop_flag_&&driver_param_.wait_for_difop)
    {
      ndifop_count_++;
      if (ndifop_count_ > 20)
      {
        reportError(ErrCode_NoDifopRecv);
        ndifop_count_ = 0;
      }
      point_msg.pointcloud_ptr = output_pointcloud_ptr;
      usleep(10000);
      return false;
    }

    std::vector<std::vector<PointT>> point_vvec;
    int height = 1;
    point_vvec.resize(pkt_scan_msg.packets.size());
#pragma omp parallel for
    for (uint32_t i = 0; i < pkt_scan_msg.packets.size(); i++)
    {
      std::vector<PointT> point_vec;
      int ret = lidar_decoder_ptr_->processMsopPkt(pkt_scan_msg.packets[i].packet.data(), point_vec, height);
      if (ret == DECODE_OK || ret == FRAME_SPLIT)
      {
        point_vvec[i] = std::move(point_vec);
      }
    }

    for (auto iiter : point_vvec)
    {
      for (auto iter = iiter.cbegin(); iter != iiter.cend(); iter++)
      {
        output_pointcloud_ptr->push_back(*iter);
      }
    }

    point_msg.pointcloud_ptr = output_pointcloud_ptr;
    point_msg.height = height;
    point_msg.width = point_msg.pointcloud_ptr->size() / point_msg.height;
    preparePointsMsg(point_msg);
    point_msg.timestamp = pkt_scan_msg.timestamp;
    if (point_msg.pointcloud_ptr->size() == 0)
    {
      reportError(ErrCode_ZeroPoints);
      return false;
    }
    return true;
  }

  void decodeDifopPkt(const PacketMsg& pkt_msg)
  {
    if (lidar_decoder_ptr_ == nullptr)
    {
      return;
    }
    lidar_decoder_ptr_->processDifopPkt(pkt_msg.packet.data());
    difop_flag_ = true;
  }

private:
  inline void runCallBack(const ScanMsg& scan_msg)
  {
    if (scan_msg.seq != 0)
    {
      for (auto& it : pkts_msop_cb_)
      {
        it(scan_msg);
      }
    }
  }

  inline void runCallBack(const PacketMsg& pkts_msg)
  {
    for (auto& it : pkts_difop_cb_)
    {
      it(pkts_msg);
    }
  }

  inline void runCallBack(const PointcloudMsg<PointT>& points_msg)
  {
    if (points_msg.seq != 0)
    {
      for (auto& it : pointscb_)
      {
        it(points_msg);
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
          DecoderFactory<PointT>::createDecoder(driver_param_.lidar_type, driver_param_.decoder_param, msg, input_ptr_);
      lidar_decoder_ptr_->loadCalibrationFile(driver_param_.angle_path);
    }
    if (msop_pkt_queue_.size() > MAX_PACKETS_BUFFER_SIZE)
    {
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
    if (!difop_flag_&&driver_param_.wait_for_difop)
    {
      ndifop_count_++;
      if (ndifop_count_ > 120)
      {
        reportError(ErrCode_NoDifopRecv);
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
      std::vector<PointT> point_vec;
      int height = 1;
      int ret = lidar_decoder_ptr_->processMsopPkt(pkt.packet.data(), point_vec, height);
      scan_ptr_->packets.emplace_back(std::move(pkt));
      if (ret == DECODE_OK || ret == FRAME_SPLIT)
      {
        for (auto iter = point_vec.cbegin(); iter != point_vec.cend(); iter++)
        {
          pointcloud_ptr_->push_back(*iter);
        }
        if (ret == FRAME_SPLIT)
        {
          PointcloudMsg<PointT> msg(pointcloud_ptr_);
          msg.height = height;
          msg.width = pointcloud_ptr_->size() / msg.height;
          preparePointsMsg(msg);
          if (driver_param_.use_lidar_clock == true)
          {
            msg.timestamp = lidar_decoder_ptr_->getLidarTime(pkt.packet.data());
          }
          if (msg.pointcloud_ptr->size() == 0)
          {
            reportError(ErrCode_ZeroPoints);
          }
          else
          {
            runCallBack(msg);
          }
          prepareScanMsg(*scan_ptr_);
          runCallBack(*scan_ptr_);
          pointcloud_ptr_.reset(new typename PointcloudMsg<PointT>::PointCloud);
          scan_ptr_.reset(new ScanMsg);
        }
      }
      else
      {
        reportError(ErrCode_DecodeFail);
        usleep(100000);
      }
    }
    msop_pkt_queue_.is_task_finished_.store(true);
  }

  void processDifop()
  {
    while (difop_pkt_queue_.size() > 0)
    {
      PacketMsg pkt = difop_pkt_queue_.popFront();
      decodeDifopPkt(pkt);
      runCallBack(pkt);
    }
    difop_pkt_queue_.is_task_finished_.store(true);
  }

  void prepareScanMsg(ScanMsg& msg)
  {
    msg.timestamp = getTime();
    if (driver_param_.use_lidar_clock == true)
    {
      msg.timestamp = lidar_decoder_ptr_->getLidarTime(msg.packets.back().packet.data());
    }
    msg.seq = scan_seq_++;
    msg.frame_id = driver_param_.frame_id;
  }

  void preparePointsMsg(PointcloudMsg<PointT>& msg)
  {
    msg.timestamp = getTime();
    msg.seq = points_seq_++;
    msg.frame_id = driver_param_.frame_id;
    msg.is_dense = false;
  }

private:
  Queue<PacketMsg> msop_pkt_queue_;
  Queue<PacketMsg> difop_pkt_queue_;
  std::vector<std::function<void(const ScanMsg&)>> pkts_msop_cb_;
  std::vector<std::function<void(const PacketMsg&)>> pkts_difop_cb_;
  std::vector<std::function<void(const PointcloudMsg<PointT>&)>> pointscb_;
  std::vector<std::function<void(const Error&)>> excb_;
  std::shared_ptr<std::thread> lidar_thread_ptr_;
  std::shared_ptr<DecoderBase<PointT>> lidar_decoder_ptr_;
  std::shared_ptr<Input> input_ptr_;
  std::shared_ptr<ThreadPool> thread_pool_ptr_;
  std::shared_ptr<ScanMsg> scan_ptr_;
  uint32_t scan_seq_;
  uint32_t points_seq_;
  uint32_t ndifop_count_;
  bool init_flag_;
  bool start_flag_;
  bool difop_flag_;
  RSDriverParam driver_param_;
  typename PointcloudMsg<PointT>::PointCloudPtr pointcloud_ptr_;
};

}  // namespace lidar
}  // namespace robosense
