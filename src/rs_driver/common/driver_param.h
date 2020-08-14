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
#include <string>

namespace robosense
{
namespace lidar
{
enum LidarType  ///< The lidar type
{
  RSAUTO = 0,  ///< If LidarType is set to RSAUTO, the driver will check the lidar type automatically.(Only support
               ///< the LiDARs of latest version, not support Bpearl & RS80 yet.)
  RS16 = 1,
  RS32 = 2,
  RSBP = 3,
  RS128 = 4,
  RS80 = 5
};

enum SplitFrameMode
{
  SPLIT_BY_ANGLE = 1,
  SPLIT_BY_FIXED_PKTS = 2,
  SPLIT_BY_CUSTOM_PKTS = 3
};

typedef struct RSCameraTriggerParam  ///< The Camera trigger parameters
{
  std::map<double, std::string> trigger_map;  ///< The map stored the trigger angle and camera frame id
  void print() const                          ///< This function is used to print all the parameters for debugging
  {
    std::cout << "\033[1m\033[32m"
              << "------------------------------------------------------"
              << "\033[0m" << std::endl;
    std::cout << "\033[1m\033[32m"
              << "             RoboSense Camera Trigger Parameters "
              << "\033[0m" << std::endl;
    for (auto iter : trigger_map)
    {
      std::cout << "\033[32m"
                << "camera_frame_id : " << iter.first << " trigger_angle : " << iter.second << std::endl;
    }
    std::cout << "\033[0m" << std::endl;
    std::cout << "\033[1m\033[32m"
              << "------------------------------------------------------"
              << "\033[0m" << std::endl;
  }
} RSCameraTriggerParam;

typedef struct RSDecoderParam  ///< The lidar decoder parameter
{
  float max_distance = 200.0f;    ///< The max distance of point cloud range
  float min_distance = 0.2f;      ///< The minimum distance of point cloud range
  float start_angle = 0.0f;       ///< The start angle of point cloud
  float end_angle = 360.0f;       ///< The end angle of point cloud
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  ///< 1: Split frames by cut_angle; 2: Split frames by fixed number of packets; 3:
                                  ///< Split frames by  custom number of packets (num_pkts_split)
  uint32_t num_pkts_split = 1;    ///< The number of packets in one frame, only be used when split_frame_mode=3
  float cut_angle = 0.0f;         ///< The cut angle(degree) used to split frame, only be used when split_frame_mode=1
  bool use_lidar_clock = false;   ///< True: lidar message timestamp is the lidar clock.
                                  ///< False: timestamp is the computer system cloc
  RSCameraTriggerParam trigger_param;  ///< The parameter used to trigger camera
  void print() const                   ///< This function is used to print all the parameters for debugging
  {
    trigger_param.print();
    std::cout << "\033[1m\033[32m"
              << "------------------------------------------------------"
              << "\033[0m" << std::endl;
    std::cout << "\033[1m\033[32m"
              << "             RoboSense Decoder Parameters "
              << "\033[0m" << std::endl;
    std::cout << "\033[32m"
              << "max_distance : " << max_distance << std::endl;
    std::cout << "min_distance : " << min_distance << std::endl;
    std::cout << "start_angle : " << start_angle << std::endl;
    std::cout << "end_angle : " << end_angle << std::endl;
    std::cout << "use_lidar_clock : " << use_lidar_clock << std::endl;
    std::cout << "split_frame_mode : " << split_frame_mode << std::endl;
    std::cout << "num_pkts_split : " << num_pkts_split << std::endl;
    std::cout << "cut_angle : " << cut_angle << "\033[0m" << std::endl;

    std::cout << "\033[1m\033[32m"
              << "------------------------------------------------------"
              << "\033[0m" << std::endl;
  }
} RSDecoderParam;

typedef struct RSInputParam  ///< The lidar input parameter
{
  std::string device_ip = "192.168.1.200";  ///< The ip of lidar
  uint16_t msop_port = 6699;                ///< The msop packet port number
  uint16_t difop_port = 7788;               ///< The difop packet port number
  bool read_pcap = false;   ///< True: The driver will process the pcap through pcap_directory. False: The driver will
                            ///< get data from online lidar
  double pcap_rate = 1;     ///< The rate to read the pcap file
  bool pcap_repeat = true;  ///< True: The pcap bag will repeat play
  std::string pcap_directory = "null";  ///< The absolute path of pcap file
  void print() const                    ///< This function is used to print all the parameters for debugging
  {
    std::cout << "\033[1m\033[32m"
              << "------------------------------------------------------"
              << "\033[0m" << std::endl;
    std::cout << "\033[1m\033[32m"
              << "             RoboSense Input Parameters "
              << "\033[0m" << std::endl;
    std::cout << "\033[32m"
              << "device_ip : " << device_ip << std::endl;
    std::cout << "msop_port : " << msop_port << std::endl;
    std::cout << "difop_port : " << difop_port << std::endl;
    std::cout << "read_pcap : " << read_pcap << std::endl;
    std::cout << "pcap_repeat : " << pcap_repeat << std::endl;
    std::cout << "pcap_directory : " << pcap_directory << "\033[0m" << std::endl;
    std::cout << "\033[1m\033[32m"
              << "------------------------------------------------------"
              << "\033[0m" << std::endl;
  }
} RSInputParam;

typedef struct RSDriverParam  ///< The lidar driver parameter
{
  RSInputParam input_param;                ///< The input parameter
  RSDecoderParam decoder_param;            ///< The decoder parameter
  std::string angle_path = "null";         ///< The path of angle calibration files(angle.csv)
                                           ///< For latest version lidar, this file is not needed
  std::string frame_id = "rslidar";        ///< The frame id of lidar message
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  bool wait_for_difop = true;              ///< True: start sending point cloud until receive difop packet
  void print() const                       ///< This function is used to print all the parameters for debugging
  {
    input_param.print();
    decoder_param.print();
    std::cout << "\033[1m\033[32m"
              << "------------------------------------------------------"
              << "\033[0m" << std::endl;
    std::cout << "\033[1m\033[32m"
              << "             RoboSense Driver Parameters "
              << "\033[0m" << std::endl;
    std::cout << "\033[32m"
              << "angle_path : " << angle_path << std::endl;
    std::cout << "frame_id : " << frame_id << std::endl;

    switch (lidar_type)
    {
      case LidarType::RS16:
        std::cout << "lidar_type : "
                  << "\033[0m";
        std::cout << "\033[1m\033[32m"
                  << "RS16"
                  << "\033[0m" << std::endl;
        break;
      case LidarType::RS32:
        std::cout << "lidar_type : "
                  << "\033[0m";
        std::cout << "\033[1m\033[32m"
                  << "RS32"
                  << "\033[0m" << std::endl;
        break;
      case LidarType::RSBP:
        std::cout << "lidar_type : "
                  << "\033[0m";
        std::cout << "\033[1m\033[32m"
                  << "RSBP"
                  << "\033[0m" << std::endl;
        break;
      case LidarType::RS128:
        std::cout << "lidar_type : "
                  << "\033[0m";
        std::cout << "\033[1m\033[32m"
                  << "RS128"
                  << "\033[0m" << std::endl;
        break;
      case LidarType::RS80:
        std::cout << "lidar_type : "
                  << "\033[0m";
        std::cout << "\033[1m\033[32m"
                  << "RS80"
                  << "\033[0m" << std::endl;
        break;
      case LidarType::RSAUTO:
        std::cout << "lidar_type : "
                  << "\033[0m";
        std::cout << "\033[1m\033[32m"
                  << "RSAUTO"
                  << "\033[0m" << std::endl;
        break;
      default:
        std::cout << "lidar_type : "
                  << "\033[0m";
        std::cout << "\033[1m\033[31m"
                  << "ERROR"
                  << "\033[0m" << std::endl;
    }

    std::cout << "\033[1m\033[32m"
              << "------------------------------------------------------"
              << "\033[0m" << std::endl;
  }
  LidarType strToLidarType(const std::string& type)
  {
    if (type == "RS16")
    {
      return lidar::LidarType::RS16;
    }
    else if (type == "RS32")
    {
      return lidar::LidarType::RS32;
    }
    else if (type == "RSBP")
    {
      return lidar::LidarType::RSBP;
    }
    else if (type == "RS128")
    {
      return lidar::LidarType::RS128;
    }
    else if (type == "RS80")
    {
      return lidar::LidarType::RS80;
    }
    else if (type == "RSAUTO")
    {
      return lidar::LidarType::RSAUTO;
    }
    else
    {
      std::cout << "\033[1m\033[31m"
                << "Wrong lidar type : " << type << std::endl;
      std::cout << "\033[1m\033[31m"
                << "Please setup the correct type: RS16, RS32, RSBP, RS128, RS80, RSAUTO" << std::endl;
      exit(-1);
    }
  }
} RSDriverParam;
}  // namespace lidar
}  // namespace robosense