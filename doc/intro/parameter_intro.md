# Parameters Introduction

#### 1 RSDecoderParam
```c++
typedef struct RSDecoderParam ///< The lidar decoder parameter
{
  float max_distance = 200.0f;   ///< The max distance of point cloud range
  float min_distance = 0.2f;     ///< The minimum distance of point cloud range
  float start_angle = 0.0f;      ///< The start angle of point cloud
  float end_angle = 360.0f;      ///< The end angle of point cloud
  SplitFrameMode split_frame_mode = 1; ///< 1: Split frames by cut_angle; 2: Split frames by fixed number of packets; 3: Split frames by  custom number of packets (num_pkts_split)
  uint32_t num_pkts_split = 0;   ///< The number of packets in one frame, only be used when split_frame_mode=3
  float cut_angle = 0.0f;        ///< The cut angle used to split frame, only be used when split_frame_mode=1
  bool use_lidar_clock = false;  ///< true: lidar message timestamp is the lidar clock. false: timestamp is the computer system clock
  RSCameraTriggerParam trigger_param;  ///< The parameter used to trigger camera
} RSDecoderParam;
```



#### 2 RSInputParam

```c++
typedef struct RSInputParam ///< The lidar input parameter
{
  std::string device_ip = "192.168.1.200"; ///< The ip of lidar
  uint16_t msop_port = 6699;               ///< The msop packet port number
  uint16_t difop_port = 7788;              ///< The difop packet port number
  bool read_pcap = false;                  ///< true: The driver will process the pcap through pcap_path. false: The driver will get data from online lidar
  double pcap_rate = 1;                    ///< The rate to read the pcap file
  bool pcap_repeat = true;                 ///< true: The pcap bag will repeat play
  std::string pcap_path = "null";     ///< The absolute path of pcap file
} RSInputParam;
```




#### 3 RSDriverParam
```c++
typedef struct RSDriverParam ///< The lidar driver parameter
{
  RSInputParam input_param;                 ///< The input parameter
  RSDecoderParam decoder_param;             ///< The decoder parameter
  std::string angle_path = "null";          ///< The path of angle calibration files(angle.csv)(for latest version lidar, this file is not needed)
  std::string frame_id = "rslidar";         ///< The frame id of lidar message
  LidarType lidar_type = LidarType::RS16;   ///< Lidar type
  bool wait_for_difop = true;               ///< true: start sending point cloud until receive difop packet
} RSDriverParam;
```



