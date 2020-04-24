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

#include <rs_common/interface/sensor/lidar_packets_interface.h>
#include <rs_common/interface/sensor/lidar_points_interface.h>
#include "rs_lidar/input.h"
#include "rs_lidar/decoder/decoder_factory.hpp"
#include "pcl/impl/point_types.hpp"
using namespace robosense::common;
namespace robosense
{
namespace sensor
{
class LidarBase : virtual public common::LidarPointsInterface, virtual public common::LidarPacketsInterface
{
public:
  LidarBase() = default;
  ~LidarBase() { stop(); }
  common::ErrCode init(const YAML::Node &config);
  common::ErrCode start();
  common::ErrCode stop();
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
  static uint16_t getApi() { return supported_api_; }
  void processMsopScan(const common::LidarScanMsg &pkt_msg);
  void processDifopPackets(const common::LidarPacketMsg &pkt_msg);
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
  void getPackets();
  void processOnlineMsop();
  void processOnlineDifop();
  void prepareLidarScanMsg(common::LidarScanMsg &msg);
  void preparePacketMsg(common::LidarPacketMsg &msg);
  void preparePointsMsg(common::LidarPointsMsg &msg);

private:
  common::Queue<common::LidarPacketMsg> msop_pkt_queue_;
  common::Queue<common::LidarPacketMsg> difop_pkt_queue_;
  bool thread_flag_;
  bool use_lidar_clock_;
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
  uint32_t timeout_;
  std::string frame_id_;
  std::string config_path_;
  std::string lidar_model_;

private:
  static const uint16_t supported_api_ = 0x0030; // 0000 0000 0011 0000 (support LiDAR points & packets)
};
} // namespace sensor
} // namespace robosense
