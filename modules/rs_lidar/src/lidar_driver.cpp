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
#include "rs_lidar/lidar_driver.h"

namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode LidarBase::init(const YAML::Node &config)
{
  setName("LidarBase");
  ST_Param param;
  int resolution;
  int intensity;
  int echo_mode;
  std::string calib_path;
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  yamlReadAbort<std::string>(driver_config, "frame_id", frame_id_);
  yamlRead<bool>(driver_config, "use_lidar_clock", use_lidar_clock_, false);
  yamlRead<uint32_t>(driver_config, "timeout", timeout_, 100);
  yamlReadAbort<std::string>(driver_config, "device_type", lidar_model_);
  yamlRead<int>(driver_config, "resolution_type", resolution, 0);
  yamlRead<int>(driver_config, "intensity_mode", intensity, 3);
  yamlRead<int>(driver_config, "echo_mode", echo_mode, 1);
  yamlRead<float>(driver_config, "min_distance", param.min_distance, 0.2);
  yamlRead<float>(driver_config, "max_distance", param.max_distance, 200);
  yamlRead<float>(driver_config, "start_angle", param.start_angle, 0);
  yamlRead<float>(driver_config, "end_angle", param.end_angle, 360);
  yamlRead<float>(driver_config, "cut_angle", param.cut_angle, 0);
  yamlRead<std::string>(driver_config, "calib_path", calib_path, "");
  param.resolution = RS_RESOLUTION_TYPE(resolution);
  param.intensity = RS_INTENSITY_TYPE(intensity);
  param.echo = RS_ECHO_MODE(echo_mode);
  lidar_decoder_ptr_ = DecoderFactory<pcl::PointXYZI>::createDecoder(lidar_model_, param);
  lidar_decoder_ptr_->loadCalibrationFile(calib_path);
  lidar_input_ptr_ = std::make_shared<Input>(driver_config);
  pointcloud_ptr_ = PointCloudPtr(new PointCloud);
  scan_ptr_ = std::make_shared<LidarScanMsg>();
  thread_flag_ = false;
  points_seq_ = 0;
  scan_seq_ = 0;
  setinitFlag(true);
  return ErrCode_Success;
}

ErrCode LidarBase::start()
{
  if (thread_flag_ == false)
  {
    thread_flag_ = true;
    lidar_thread_ptr_ = std::make_shared<std::thread>([this] { getPackets(); });
  }
  return ErrCode_Success;
}

ErrCode LidarBase::stop()
{
  msop_pkt_queue_.clear();
  difop_pkt_queue_.clear();
  if (thread_flag_ == true)
  {
    thread_flag_ = false;
    lidar_thread_ptr_->join();
  }
  return ErrCode_Success;
}

void LidarBase::processOnlineMsop()
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
        if (use_lidar_clock_ == true)
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

void LidarBase::processMsopScan(const common::LidarScanMsg &pkt_scan_msg)
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

void LidarBase::processOnlineDifop()
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

void LidarBase::processDifopPackets(const common::LidarPacketMsg &pkt_msg)
{
  lidar_decoder_ptr_->processDifopPkt(pkt_msg.packet.data());
}

void LidarBase::getPackets()
{

  while (thread_flag_)
  {
    common::LidarPacketMsg pkt_msg;
    InputState ret = lidar_input_ptr_->getPacket(pkt_msg.packet.data(), timeout_);
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

void LidarBase::prepareLidarScanMsg(common::LidarScanMsg &msg)
{
  msg.timestamp = getTime();
  if (use_lidar_clock_ == true)
  {
    msg.timestamp = lidar_decoder_ptr_->getLidarTime(msg.packets.back().packet.data());
  }
  msg.seq = ++scan_seq_;
  msg.parent_frame_id = frame_id_;
  msg.frame_id = frame_id_;
}

void LidarBase::preparePointsMsg(common::LidarPointsMsg &msg)
{
  msg.timestamp = getTime();
  msg.seq = ++points_seq_;
  msg.parent_frame_id = frame_id_;
  msg.frame_id = frame_id_;
  msg.height = msg.cloudPtr->height;
  msg.width = msg.cloudPtr->width;
  msg.is_dense = false;
  msg.is_transform = false;
  msg.is_motion_correct = false;
  msg.lidar_model = lidar_model_;
  msg.points_type = "XYZI";
}

} // namespace sensor
} // namespace robosense