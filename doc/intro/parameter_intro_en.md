# Parameters Intorduction
## 1 Parameter Definition
The detail parameters are defined in `rs_driver/src/rs_driver/driver_param.h` file.

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
  float max_distance = 200.0f;  ///< Max distance of point cloud range
  float min_distance = 0.2f;    ///< Minimum distance of point cloud range
  float start_angle = 0.0f;     ///< Start angle of point cloud
  float end_angle = 360.0f;     ///< End angle of point cloud
  SplitFrameMode split_frame_mode =
      SplitFrameMode::SPLIT_BY_ANGLE;  ///< 1: Split frames by cut_angle;
                                       ///< 2: Split frames by fixed number of packets;
                                       ///< 3: Split frames by custom number of packets (num_pkts_split)
  uint32_t num_pkts_split = 1;         ///< Number of packets in one frame, only be used when split_frame_mode=3
  float cut_angle = 0.0f;              ///< Cut angle(degree) used to split frame, only be used when split_frame_mode=1
  bool use_lidar_clock = false;        ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  RSTransformParam transform_param;    ///< Used to transform points
  RSCameraTriggerParam trigger_param;  ///< Used to trigger camera
} RSDecoderParam;

typedef struct RSInputParam  ///< The LiDAR input parameter
{
  std::string device_ip = "192.168.1.200";     ///< Ip of LiDAR
  std::string multi_cast_address = "0.0.0.0";  ///< Address of multicast
  uint16_t msop_port = 6699;                   ///< Msop packet port number
  uint16_t difop_port = 7788;                  ///< Difop packet port number
  bool read_pcap = false;          ///< true: The driver will process the pcap through pcap_path. false: The driver will
                                   ///< Get data from online LiDAR
  double pcap_rate = 1;            ///< Rate to read the pcap file
  bool pcap_repeat = true;         ///< true: The pcap bag will repeat play
  std::string pcap_path = "null";  ///< Absolute path of pcap file
  bool use_vlan = false;                       ///< Vlan on-off
  bool use_someip = false;                     ///< Someip on-off
} RSInputParam;

typedef struct RSDriverParam  ///< The LiDAR driver parameter
{
  RSInputParam input_param;          ///< Input parameter
  RSDecoderParam decoder_param;      ///< Decoder parameter
  std::string angle_path = "null";   ///< Path of angle calibration files(angle.csv).Only used for internal debugging.
  std::string frame_id = "rslidar";  ///< The frame id of LiDAR message
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  bool wait_for_difop = true;              ///< true: start sending point cloud until receive difop packet
  bool saved_by_rows = false;  ///< true: the output point cloud will be saved by rows (default is saved by columns)
} RSDriverParam;
```
