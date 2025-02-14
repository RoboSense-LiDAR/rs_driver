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

#include <rs_driver/common/rs_log.hpp>
#include "rs_driver/msg/imu_data_msg.hpp"
#include <string>
#include <map>
#include <cstring>
#include <unordered_map>
namespace robosense
{
namespace lidar
{
enum LidarType  ///< LiDAR type
{
  // mechanical
  RS_MECH = 0x01,
  RS16 = RS_MECH,
  RS32,
  RSBP,
  RSAIRY,
  RSHELIOS,
  RSHELIOS_16P,
  RS128,
  RS80,
  RS48,
  RSP128,
  RSP80,
  RSP48,

  // mems
  RS_MEMS = 0x20,
  RSM1 = RS_MEMS,
  RSM2,
  RSM3,
  RSE1,
  RSMX,

  // jumbo
  RS_JUMBO = 0x100,
  RSM1_JUMBO = RS_JUMBO + RSM1,
};

inline bool isMech(LidarType type)
{
  return ((LidarType::RS_MECH <= type) && (type < LidarType::RS_MEMS));
}

inline bool isMems (LidarType type)
{
  return ((LidarType::RS_MEMS <= type) && (type < LidarType::RS_JUMBO));
}

inline bool isJumbo (LidarType type)
{
  return (LidarType::RS_JUMBO <= type);
}

inline std::string lidarTypeToStr(const LidarType& type)
{
   static const std::unordered_map<LidarType, std::string> lidarTypeMap = {
        {LidarType::RS16, "RS16"},
        {LidarType::RS32, "RS32"},
        {LidarType::RSBP, "RSBP"},
        {LidarType::RSAIRY, "RSAIRY"},
        {LidarType::RSHELIOS, "RSHELIOS"},
        {LidarType::RSHELIOS_16P, "RSHELIOS_16P"},
        {LidarType::RS128, "RS128"},
        {LidarType::RS80, "RS80"},
        {LidarType::RS48, "RS48"},
        {LidarType::RSP128, "RSP128"},
        {LidarType::RSP80, "RSP80"},
        {LidarType::RSP48, "RSP48"},
        {LidarType::RSM1, "RSM1"},
        {LidarType::RSM2, "RSM2"},
        {LidarType::RSM3, "RSM3"},
        {LidarType::RSE1, "RSE1"},
        {LidarType::RSMX, "RSMX"},
        {LidarType::RSM1_JUMBO, "RSM1_JUMBO"},
    };

    auto it = lidarTypeMap.find(type);
    if (it != lidarTypeMap.end()) {
        return it->second;
    } else {
        RS_ERROR << "RS_ERROR" << RS_REND;
        std::string str = "ERROR";
        return str;
    }
}

inline LidarType strToLidarType(const std::string& type)
{
   static const std::unordered_map<std::string, LidarType> strLidarTypeMap = {
        {"RS16", LidarType::RS16},
        {"RS32", LidarType::RS32},
        {"RSBP", LidarType::RSBP},
        {"RSHELIOS", LidarType::RSHELIOS},
        {"RSHELIOS_16P", LidarType::RSHELIOS_16P},
        {"RS128", LidarType::RS128},
        {"RS80", LidarType::RS80},
        {"RS48", LidarType::RS48},
        {"RSP128", LidarType::RSP128},
        {"RSP80", LidarType::RSP80},
        {"RSP48", LidarType::RSP48},
        {"RSM1", LidarType::RSM1},
        {"RSM2", LidarType::RSM2},
        {"RSM3", LidarType::RSM3},
        {"RSE1", LidarType::RSE1},
        {"RSMX", LidarType::RSMX},
        {"RSAIRY", LidarType::RSAIRY},
        {"RSM1_JUMBO", LidarType::RSM1_JUMBO},
    };

    auto it = strLidarTypeMap.find(type);
    if (it != strLidarTypeMap.end()) {
        return it->second;
    } else {
      RS_ERROR << "Wrong lidar type: " << type << RS_REND;
      RS_ERROR << "Please give correct type: RS16, RS32, RSBP, RSHELIOS, RSHELIOS_16P, RS48, RS80, RS128, RSP128, RSP80, RSP48, "
              << "RSM1, RSM1_JUMBO, RSM2,RSM3, RSE1, RSMX, RSAIRY." 
              << RS_REND;
      exit(-1);
    }
}

enum InputType
{
  ONLINE_LIDAR = 1,
  PCAP_FILE,
  RAW_PACKET
};

inline std::string inputTypeToStr(const InputType& type)
{
  std::string str = "";
  switch (type)
  {
    case InputType::ONLINE_LIDAR:
      str = "ONLINE_LIDAR";
      break;
    case InputType::PCAP_FILE:
      str = "PCAP_FILE";
      break;
    case InputType::RAW_PACKET:
      str = "RAW_PACKET";
      break;
    default:
      str = "ERROR";
      RS_ERROR << "RS_ERROR" << RS_REND;
  }
  return str;
}

enum SplitFrameMode
{
  SPLIT_BY_ANGLE = 1,
  SPLIT_BY_FIXED_BLKS,
  SPLIT_BY_CUSTOM_BLKS
};

struct RSTransformParam  ///< The Point transform parameter
{
  float x = 0.0f;      ///< unit, m
  float y = 0.0f;      ///< unit, m
  float z = 0.0f;      ///< unit, m
  float roll = 0.0f;   ///< unit, radian
  float pitch = 0.0f;  ///< unit, radian
  float yaw = 0.0f;    ///< unit, radian

  void print() const
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Transform Parameters " << RS_REND;
    RS_INFOL << "x: " << x << RS_REND;
    RS_INFOL << "y: " << y << RS_REND;
    RS_INFOL << "z: " << z << RS_REND;
    RS_INFOL << "roll: " << roll << RS_REND;
    RS_INFOL << "pitch: " << pitch << RS_REND;
    RS_INFOL << "yaw: " << yaw << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
  }
};

struct RSDecoderParam  ///< LiDAR decoder parameter
{
  float min_distance = 0.0f;     ///< min/max distances of point cloud range. valid if min distance or max distance > 0
  float max_distance = 0.0f; 
  bool use_lidar_clock = false;  ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool dense_points = false;     ///< true: discard NAN points; false: reserve NAN points
  bool ts_first_point = false;   ///< true: time-stamp point cloud with the first point; false: with the last point;
  bool wait_for_difop = true;    ///< true: start sending point cloud until receive difop packet
  RSTransformParam transform_param; ///< Used to transform points

  ///< Theses parameters are only for mechanical Lidars.
  bool config_from_file = false; ///< Internal use only for debugging
  std::string angle_path = "";   ///< Internal use only for debugging
  float start_angle = 0.0f;      ///< Start angle of point cloud
  float end_angle = 360.0f;      ///< End angle of point cloud
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  
                                 ///< 1: Split frames by split_angle;
                                 ///< 2: Split frames by fixed number of blocks;
                                 ///< 3: Split frames by custom number of blocks (num_blks_split)
  float split_angle = 0.0f;      ///< Split angle(degree) used to split frame, only be used when split_frame_mode=1
  uint16_t num_blks_split = 1;   ///< Number of packets in one frame, only be used when split_frame_mode=3

  void print() const
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Decoder Parameters " << RS_REND;
    RS_INFOL << "min_distance: " << min_distance << RS_REND;
    RS_INFOL << "max_distance: " << max_distance << RS_REND;
    RS_INFOL << "use_lidar_clock: " << use_lidar_clock << RS_REND;
    RS_INFOL << "dense_points: " << dense_points << RS_REND;
    RS_INFOL << "ts_first_point: " << ts_first_point << RS_REND;
    RS_INFOL << "wait_for_difop: " << wait_for_difop << RS_REND;
    RS_INFOL << "config_from_file: " << config_from_file << RS_REND;
    RS_INFOL << "angle_path: " << angle_path << RS_REND;
    RS_INFOL << "start_angle: " << start_angle << RS_REND;
    RS_INFOL << "end_angle: " << end_angle << RS_REND;
    RS_INFOL << "split_frame_mode: " << split_frame_mode << RS_REND;
    RS_INFOL << "split_angle: " << split_angle << RS_REND;
    RS_INFOL << "num_blks_split: " << num_blks_split << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
    transform_param.print();
  }

};

struct RSInputParam  ///< The LiDAR input parameter
{
  uint16_t msop_port = 6699;                   ///< Msop packet port number
  uint16_t difop_port = 7788;                  ///< Difop packet port number
  uint16_t imu_port = 0;                  ///< IMU packet port number, default disable
  uint16_t user_layer_bytes = 0;    ///< Bytes of user layer. thers is no user layer if it is 0
  uint16_t tail_layer_bytes = 0;    ///< Bytes of tail layer. thers is no tail layer if it is 0

  ///< These parameters are valid when the input type is online lidar
  std::string host_address = "0.0.0.0";        ///< Address of host
  std::string group_address = "0.0.0.0";       ///< Address of multicast group
  uint32_t socket_recv_buf = 106496;   //  <Bytes of socket receive buffer. 

  ///< These parameters are valid when the input type is pcap file
  std::string pcap_path = "";                  ///< Absolute path of pcap file
  bool pcap_repeat = true;                     ///< true: The pcap bag will repeat play
  float pcap_rate = 1.0f;                      ///< Rate to read the pcap file
  bool use_vlan = false;                       ///< Vlan on-off

  void print() const
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Input Parameters " << RS_REND;
    RS_INFOL << "msop_port: " << msop_port << RS_REND;
    RS_INFOL << "difop_port: " << difop_port << RS_REND;
    RS_INFOL << "imu_port: " << imu_port << RS_REND;
    RS_INFOL << "user_layer_bytes: " << user_layer_bytes << RS_REND;
    RS_INFOL << "tail_layer_bytes: " << tail_layer_bytes << RS_REND;
    RS_INFOL << "host_address: " << host_address << RS_REND;
    RS_INFOL << "group_address: " << group_address << RS_REND;
    RS_INFOL << "socket_recv_buf: " << socket_recv_buf << RS_REND;
    RS_INFOL << "pcap_path: " << pcap_path << RS_REND;
    RS_INFOL << "pcap_rate: " << pcap_rate << RS_REND;
    RS_INFOL << "pcap_repeat: " << pcap_repeat << RS_REND;
    RS_INFOL << "use_vlan: " << use_vlan << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
  }

};

struct RSDriverParam  ///< The LiDAR driver parameter
{
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; ///< Input type
  std::string frame_id = "rslidar";  ///< The frame id of LiDAR mesage
  RSInputParam input_param;          ///< Input parameter
  RSDecoderParam decoder_param;      ///< Decoder parameter

  void print() const
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Driver Parameters " << RS_REND;
    RS_INFOL << "input type: " << inputTypeToStr(input_type) << RS_REND;
    RS_INFOL << "lidar_type: " << lidarTypeToStr(lidar_type) << RS_REND;
    RS_INFOL << "frame_id: "   << frame_id << RS_REND;
    RS_INFOL << "------------------------------------------------------" << RS_REND;

    input_param.print();
    decoder_param.print();
  }

};


struct DeviceInfo
{
  DeviceInfo()
  {
      init();
  }
  bool state;
  uint8_t sn[6];
  uint8_t mac[6];
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];

  float qx{0.0f};
  float qy{0.0f};
  float qz{0.0f};
  float qw{1.0f};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  
  void init()
  {
    memset(sn, 0, sizeof(sn));
    memset(mac, 0, sizeof(mac));
    memset(top_ver, 0, sizeof(top_ver));
    memset(bottom_ver, 0, sizeof(bottom_ver));
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
    qw = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    state = false;
  }

  DeviceInfo& operator=(const DeviceInfo& other)
  {
    if (this != &other) 
    {
      memcpy(sn, other.sn, sizeof(sn));
      memcpy(mac, other.mac, sizeof(mac));
      memcpy(top_ver, other.top_ver, sizeof(top_ver));
      memcpy(bottom_ver, other.bottom_ver, sizeof(bottom_ver));
      qx = other.qx;
      qy = other.qy;
      qz = other.qz;
      qw = other.qw;
      x = other.x;
      y = other.y;
      z = other.z;
      state = other.state;
    }
    return *this;
  }
};


struct DeviceStatus
{
  DeviceStatus()
  {
      init();
  }
  float voltage = 0.0f;
  bool state;
  void init()
  {
    voltage = 0.0f;
    state = false;
  }

  DeviceStatus& operator=(const DeviceStatus& other)
  {
    if (this != &other) 
    {
      voltage = other.voltage;
      state = other.state;
    }
    return *this;
  }
};

}  // namespace lidar
}  // namespace robosense
