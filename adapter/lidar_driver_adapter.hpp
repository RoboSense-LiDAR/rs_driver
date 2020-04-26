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
#include <driver/lidar_driver.hpp>
namespace robosense
{
namespace sensor
{
class LidarDriverAdapter : virtual public common::LidarPointsInterface, virtual public common::LidarPacketsInterface
{
public:
  LidarDriverAdapter()
  {
    driver_ptr_.reset(new LidarDriver());
  }
  ~LidarDriverAdapter()
  {
    driver_ptr_->stop();
  }
  common::ErrCode init(const YAML::Node &config)
  {
    setName("LidarDriverAdapter");
    RSLiDAR_Driver_Param driver_param;
    int echo_mode;
    std::string calib_path;
    YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
    yamlRead<std::string>(driver_config, "frame_id", driver_param.frame_id, "rslidar_points");
    yamlRead<std::string>(driver_config, "calib_path", driver_param.calib_path, "");
    yamlReadAbort<std::string>(driver_config, "device_type", driver_param.device_type);
    yamlRead<bool>(driver_config, "use_lidar_clock", driver_param.use_lidar_clock, false);
    yamlRead<uint32_t>(driver_config, "timeout", driver_param.timeout, 100);
    yamlRead<int>(driver_config, "echo_mode", echo_mode, 1);
    yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0.2);
    yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
    yamlRead<float>(driver_config, "start_angle", driver_param.decoder_param.start_angle, 0);
    yamlRead<float>(driver_config, "end_angle", driver_param.decoder_param.end_angle, 360);
    yamlRead<float>(driver_config, "cut_angle", driver_param.decoder_param.cut_angle, 0);
    yamlRead<std::string>(driver_config, "calib_path", driver_param.calib_path, "");
    yamlRead<std::string>(driver_config, "device_ip", driver_param.input_param.device_ip);
    yamlRead<uint16_t>(driver_config, "msop_port", driver_param.input_param.msop_port);
    yamlRead<uint16_t>(driver_config, "difop_port", driver_param.input_param.difop_port);
    yamlRead<bool>(driver_config, "read_pcap", driver_param.input_param.read_pcap);
    yamlRead<std::string>(driver_config, "pcap", driver_param.input_param.pcap_file_dir);
    driver_param.decoder_param.echo = RS_ECHO_MODE(echo_mode);
    driver_ptr_->init(driver_param);
    setinitFlag(true);
  }
  common::ErrCode start()
  {
    driver_ptr_->start();
  }
  common::ErrCode stop()
  {
    driver_ptr_->stop();
  }
  inline void regRecvCallback(const std::function<void(const common::LidarPointsMsg &)> callBack)
  {
    driver_ptr_->regRecvCallback(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::LidarScanMsg &)> callBack)
  {
    driver_ptr_->regRecvCallback(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::LidarPacketMsg &)> callBack)
  {
    driver_ptr_->regRecvCallback(callBack);
  }
  inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> callBack)
  {
    driver_ptr_->regExceptionCallback(callBack);
  }
  static uint16_t getApi()
  {
    return supported_api_;
  }
  void processMsopScan(const common::LidarScanMsg &pkt_msg)
  {
    driver_ptr_->processMsopScan(pkt_msg);
  }
  void processDifopPackets(const common::LidarPacketMsg &pkt_msg)
  {
    driver_ptr_->processDifopPackets(pkt_msg);
  }

private:
  std::shared_ptr<LidarDriver> driver_ptr_;

private:
  static const uint16_t supported_api_ = 0x0030; // 0000 0000 0011 0000 (support LiDAR points & packets)
};
} // namespace sensor
} // namespace robosense
