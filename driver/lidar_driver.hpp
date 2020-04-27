/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
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

#include <common/interface/sensor/lidar_packets_interface.h>
#include <common/interface/sensor/lidar_points_interface.h>
#include "driver/input.hpp"
#include "driver/decoder/decoder_factory.hpp"
#include "pcl/impl/point_types.hpp"
using namespace robosense::common;
namespace robosense
{
namespace sensor
{

typedef struct RSLiDAR_Driver_Param
{
  std::string calib_path = "";
  std::string frame_id = "rslidar_points";
  std::string device_type = "RS16";
  bool use_lidar_clock = false;
  uint32_t timeout = 100;
  RSInput_Param input_param;
  RSDecoder_Param decoder_param;
} RSLiDAR_Driver_Param;

class LidarDriver
{
public:
  LidarDriver() = default;
  ~LidarDriver() { stop(); }
  common::ErrCode init(const RSLiDAR_Driver_Param &param)
  {
    driver_param_ = param;
    lidar_decoder_ptr_ = DecoderFactory<pcl::PointXYZI>::createDecoder(driver_param_.device_type, driver_param_.decoder_param);
    lidar_decoder_ptr_->loadCalibrationFile(driver_param_.calib_path);
    lidar_input_ptr_ = std::make_shared<Input>(driver_param_.input_param);
    pointcloud_ptr_ = PointCloudPtr(new PointCloud);
    scan_ptr_ = std::make_shared<LidarScanMsg>();
    thread_flag_ = false;
    points_seq_ = 0;
    scan_seq_ = 0;
    return ErrCode_Success;
  }
  void start()
  {
    if (thread_flag_ == false)
    {
      thread_flag_ = true;
      lidar_thread_ptr_ = std::make_shared<std::thread>([this] { getPackets(); });
    }
  }
  void stop()
  {
    msop_pkt_queue_.clear();
    difop_pkt_queue_.clear();
    if (thread_flag_ == true)
    {
      thread_flag_ = false;
      lidar_thread_ptr_->join();
    }
  }
  inline void regRecvCallback(const std::function<void(const common::LidarPointsMsg &)> callBack)
  {
    pointscb_.emplace_back(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::LidarScanMsg &)> callBack)
  {
    pkts_msop_cb_.emplace_back(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::LidarPacketMsg &)> callBack)
  {
    pkts_difop_cb_.emplace_back(callBack);
  }
  inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack)
  {
    excb_ = excallBack;
  }
  void processMsopScan(const common::LidarScanMsg &pkt_scan_msg)
  {
    std::vector<std::vector<pcl::PointXYZI>> point_vvec;
    int height = 1;
    PointCloudPtr output_pointcloud_ptr = PointCloudPtr(new PointCloud);
    point_vvec.resize(pkt_scan_msg.packets.size());
#pragma omp parallel for
    for (uint32_t i = 0; i < pkt_scan_msg.packets.size(); i++)
    {
      std::vector<pcl::PointXYZI> point_vec;
      int ret = lidar_decoder_ptr_->processMsopPkt(pkt_scan_msg.packets[i].packet.data(), point_vec, height);
      if (ret == E_DECODE_OK || ret == E_FRAME_SPLIT)
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
    output_pointcloud_ptr->height = height;
    output_pointcloud_ptr->width = output_pointcloud_ptr->points.size() / output_pointcloud_ptr->height;
    common::LidarPointsMsg msg(output_pointcloud_ptr);
    preparePointsMsg(msg);
    msg.timestamp = pkt_scan_msg.timestamp;
    if (msg.cloudPtr->points.size() == 0)
    {
      ERROR << "LiDAR driver output points is zero.  Please make sure your lidar type in lidar yaml is right!" << REND;
    }
    else
    {
      runCallBack(msg);
    }
  }
  void processDifopPackets(const common::LidarPacketMsg &pkt_msg)
  {
    lidar_decoder_ptr_->processDifopPkt(pkt_msg.packet.data());
  }

private:
  inline void runCallBack(const common::LidarScanMsg &pkts_msg)
  {
    for (auto &it : pkts_msop_cb_)
    {
      it(pkts_msg);
    }
  }
  inline void runCallBack(const common::LidarPacketMsg &pkts_msg)
  {
    for (auto &it : pkts_difop_cb_)
    {
      it(pkts_msg);
    }
  }
  inline void runCallBack(const common::LidarPointsMsg &points_msg)
  {

    for (auto &it : pointscb_)
    {
      it(points_msg);
    }
  }
  inline void reportError(const common::ErrCode &error)
  {
    if (excb_ != NULL)
    {
      excb_(error);
    }
  }

private:
  void getPackets()
  {
    while (thread_flag_)
    {
      common::LidarPacketMsg pkt_msg;
      InputState ret = lidar_input_ptr_->getPacket(pkt_msg.packet.data(), driver_param_.timeout);
      if (ret == INPUT_MSOP)
      {
        msop_pkt_queue_.push(pkt_msg);
        if (msop_pkt_queue_.is_task_finished.load())
        {
          msop_pkt_queue_.is_task_finished.store(false);
          common::ThreadPool::getInstance()->commit([this]() { processOnlineMsop(); });
        }
      }
      else if (ret == INPUT_DIFOP)
      {
        difop_pkt_queue_.push(pkt_msg);
        if (difop_pkt_queue_.is_task_finished.load())
        {
          difop_pkt_queue_.is_task_finished.store(false);
          common::ThreadPool::getInstance()->commit([this]() { processOnlineDifop(); });
        }
      }
      else if (ret == INPUT_ERROR || ret == INPUT_EXIT || ret == INPUT_TIMEOUT)
      {
        reportError(ErrCode_LidarDriverInterrupt);
      }
    }
  }
  void processOnlineMsop()
  {
    while (msop_pkt_queue_.m_quque.size() > 0 && thread_flag_)
    {
      LidarPacketMsg pkt = msop_pkt_queue_.m_quque.front();
      scan_ptr_->packets.emplace_back(pkt);
      msop_pkt_queue_.pop();
      std::vector<pcl::PointXYZI> point_vec;
      int height = 1;
      int ret = lidar_decoder_ptr_->processMsopPkt(pkt.packet.data(), point_vec, height);
      if (ret == E_DECODE_OK || ret == E_FRAME_SPLIT)
      {
        for (auto iter = point_vec.cbegin(); iter != point_vec.cend(); iter++)
        {
          pointcloud_ptr_->push_back(*iter);
        }
        if (ret == E_FRAME_SPLIT)
        {
          pointcloud_ptr_->height = height;
          pointcloud_ptr_->width = pointcloud_ptr_->points.size() / pointcloud_ptr_->height;
          common::LidarPointsMsg msg(pointcloud_ptr_);
          preparePointsMsg(msg);
          if (driver_param_.use_lidar_clock == true)
          {
            msg.timestamp = lidar_decoder_ptr_->getLidarTime(pkt.packet.data());
          }
          if (msg.cloudPtr->points.size() == 0)
          {
            ERROR << "LiDAR driver output points is zero.  Please make sure your lidar type in lidar yaml is right!" << REND;
          }
          else
          {
            runCallBack(msg);
          }
          pointcloud_ptr_.reset(new PointCloud);
          prepareLidarScanMsg(*scan_ptr_);
          runCallBack(*scan_ptr_);
          scan_ptr_.reset(new LidarScanMsg);
        }
      }
    }
    msop_pkt_queue_.is_task_finished.store(true);
  }
  void processOnlineDifop()
  {
    while (difop_pkt_queue_.m_quque.size() > 0 && thread_flag_)
    {
      LidarPacketMsg pkt = difop_pkt_queue_.m_quque.front();
      difop_pkt_queue_.pop();
      lidar_decoder_ptr_->processDifopPkt(pkt.packet.data());
      runCallBack(pkt);
    }
    difop_pkt_queue_.is_task_finished.store(true);
  }
  void prepareLidarScanMsg(common::LidarScanMsg &msg)
  {
    msg.timestamp = getTime();
    if (driver_param_.use_lidar_clock == true)
    {
      msg.timestamp = lidar_decoder_ptr_->getLidarTime(msg.packets.back().packet.data());
    }
    msg.seq = ++scan_seq_;
    msg.parent_frame_id = driver_param_.frame_id;
    msg.frame_id = driver_param_.frame_id;
  }
  void preparePointsMsg(common::LidarPointsMsg &msg)
  {
    msg.timestamp = getTime();
    msg.seq = ++points_seq_;
    msg.parent_frame_id = driver_param_.frame_id;
    msg.frame_id = driver_param_.frame_id;
    msg.height = msg.cloudPtr->height;
    msg.width = msg.cloudPtr->width;
    msg.is_dense = false;
    msg.is_transform = false;
    msg.is_motion_correct = false;
    msg.lidar_model = driver_param_.device_type;
    msg.points_type = "XYZI";
  }

private:
  common::Queue<common::LidarPacketMsg> msop_pkt_queue_;
  common::Queue<common::LidarPacketMsg> difop_pkt_queue_;
  bool thread_flag_;
  std::vector<std::function<void(const common::LidarScanMsg &)>> pkts_msop_cb_;
  std::vector<std::function<void(const common::LidarPacketMsg &)>> pkts_difop_cb_;
  std::vector<std::function<void(const common::LidarPointsMsg &)>> pointscb_;
  std::function<void(const common::ErrCode &)> excb_;
  std::shared_ptr<std::thread> lidar_thread_ptr_;
  std::shared_ptr<DecoderBase<pcl::PointXYZI>> lidar_decoder_ptr_;
  std::shared_ptr<Input> lidar_input_ptr_;
  PointCloudPtr pointcloud_ptr_;
  std::shared_ptr<common::LidarScanMsg> scan_ptr_;
  uint32_t scan_seq_;
  uint32_t points_seq_;
  RSLiDAR_Driver_Param driver_param_;
};
} // namespace sensor
} // namespace robosense
