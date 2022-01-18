# 参数介绍
## 1 参数定义
参数文件`rs_driver/src/rs_driver/driver_param.h`里面详细定义了每一个参数，如下。

```
typedef struct RSTransformParam  ///< The Point transform parameter
{
  float x = 0.0f;      ///< unit, m
  float y = 0.0f;      ///< unit, m
  float z = 0.0f;      ///< unit, m
  float roll = 0.0f;   ///< unit, radian
  float pitch = 0.0f;  ///< unit, radian
  float yaw = 0.0f;    ///< unit, radian
} RSTransformParam;

typedef struct RSDecoderParam  ///< LiDAR decoder parameter
{
  bool config_from_file = false;///< Internal use only for debugging
  std::string angle_path = "";  ///< Internal use only for debugging
  bool wait_for_difop = true;   ///< true: start sending point cloud until receive difop packet
  float min_distance = 0.2f;    ///< Minimum distance of point cloud range
  float max_distance = 200.0f;  ///< Max distance of point cloud range
  float start_angle = 0.0f;     ///< Start angle of point cloud
  float end_angle = 360.0f;     ///< End angle of point cloud
  SplitFrameMode split_frame_mode =
      SplitFrameMode::SPLIT_BY_ANGLE;  ///< 1: Split frames by split_angle;
                                       ///< 2: Split frames by fixed number of blocks;
                                       ///< 3: Split frames by custom number of blocks (num_blks_split)
  float split_angle = 0.0f;      ///< Split angle(degree) used to split frame, only be used when split_frame_mode=1
  uint16_t num_blks_split = 1;   ///< Number of packets in one frame, only be used when split_frame_mode=3
  bool use_lidar_clock = false;  ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool dense_points = false;     ///< true: discard NAN points; false: reserve NAN points
  RSTransformParam transform_param;    ///< Used to transform points
} RSDecoderParam;

typedef struct RSInputParam  ///< The LiDAR input parameter
{
  uint16_t msop_port = 6699;       ///< MSOP packet port number
  uint16_t difop_port = 7788;      ///< DIFOP packet port number
  std::string host_address = "0.0.0.0";   ///< Address of host  
  std::string group_address = "0.0.0.0";  ///< Address of multicast group
  std::string pcap_path = "";      ///< Absolute path of pcap file
  bool pcap_repeat = true;         ///< true: The pcap bag will repeat play
  float pcap_rate = 1.0;           ///< Rate to read the pcap file
  bool use_vlan = false;           ///< Vlan on-off
  bool use_someip = false;         ///< SOME/IP layer on-off
} RSInputParam;

typedef struct RSDriverParam  ///< The LiDAR driver parameter
{
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; ///< Input type
  RSInputParam input_param;          ///< Input parameter
  RSDecoderParam decoder_param;      ///< Decoder parameter
} RSDriverParam;
```
