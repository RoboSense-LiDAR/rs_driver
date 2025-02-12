# 4 Introduction to rs_driver's Parameters 



## 4.1 Parameter File

The parameters are defined in the file `rs_driver/src/rs_driver/driver_param.h`.

Basically, there are 3 structures: 

+ RSDriverParam 
+ RSDecoderParam 
+ RSInputParam



## 4.2 RSDriverParam

RSDriverParam contains RSDecoderParam and RSInputParam.

```c++
typedef struct RSDriverParam
{
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; ///< Input type
  RSInputParam input_param;
  RSDecoderParam decoder_param;
} RSDriverParam;
```

+ lidar_type - Lidar Type. Some LiDARs are mechanical, and some are MEMS. Some parameters of RSDecoderParam is only for mechanical LiDARs.

```c++
enum LidarType
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
```

+ input_type - What source the Lidar packets is from.
  + ONLINE_LIDAR means from online LiDAR; PCAP_FILE means from PCAP file, which is captured with 3rd party tool; RAW_PACKET is user's own data captured with the `rs_driver` API.

```c++
enum InputType
{
  ONLINE_LIDAR = 1,
  PCAP_FILE,
  RAW_PACKET
};
```



## 4.3 RSInputParam

RSInputParam specifies the detail paramters of packet source.

The following parameters are for `ONLINE_LIDAR` and `PCAP_FILE`.
+ msop_port - The UDP port on the host, to receive MSOP packets.
+ difop_port - The UDP port on the host, to receive DIFOP Packets.
+ user_layer_bytes - Bytes of user layer. thers is no user layer if it is 0
+ tail_layer_bytes - Bytes of tail layer. thers is no tail layer if it is 0

The following parameters are only for ONLINE_LIDAR.
+ host_address - The host's IP, to receive MSOP/DIFOP Packets
+ group_address - A multicast group to receive MSOP/DIFOP packts. `rs_driver` make `host_address` join it.
+ socket_recv_buf - Bytes of socket receive buffer.

The following parameters are only for PCAP_FILE.
+ pcap_path - Full path of the PCAP file.
+ pcap_repeat - Whether to replay PCAP file repeatly
+ pcap_rate - `rs_driver` replay the PCAP file by the theological frame rate. `pcap_rate` gives a rate to it, so as to speed up or slow down.
+ use_vlan - If the PCAP file contains VLAN layer, use `use_vlan`=`true` to skip it.

```c++
typedef struct RSInputParam
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
} RSInputParam;

```



## 4.4 RSDecoderParam

RSDecoderParam specifies how rs_driver decode LiDAR's packets.

```c++
typedef struct RSDecoderParam
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
} RSDecoderParam;
```

The following parameters are for all LiDARs。

+ min_distance、max_distance - Set measurement range. Internal use only.
+ use_lidar_clock - Where the point cloud's timestamp is from. From the LiDAR, or from `rs_driver` on the host? 
  + If `use_lidar_clock`=`true`，use the LiDAR timestamp, else use the host one.
+ dense_points - Whether the point cloud is dense.
  + If `dense_points`=`false`, then point cloud contains NAN points, else discard them.
+ ts_first_point - Whether to stamp the point cloud with the first point, or the last point.
  + If `ts_first_point`=`false`, then stamp it with the last point, else with the first point。
+ wait_for_difop - Whether wait for DIFOP Packet before parse MSOP packets.
  + DIFOP Packet contains angle calibration parameters. If it is unavailable, the point cloud is flat.
  + If you get no point cloud, try `wait_for_difop`=`false`. It might help to locate the problem.
+ transform_param - paramters of coordinate transformation. It is only valid when the CMake option `ENABLE_TRANSFORM`=`ON`.

```c++
typedef struct RSTransformParam
{
  float x = 0.0f;      ///< unit, m
  float y = 0.0f;      ///< unit, m
  float z = 0.0f;      ///< unit, m
  float roll = 0.0f;   ///< unit, radian
  float pitch = 0.0f;  ///< unit, radian
  float yaw = 0.0f;    ///< unit, radian
} RSTransformParam;
```

The following parameters are only for mechanical LiDARs.

+ config_from_file - Where to get LiDAR configuration parameters, from extern files, or from DIFOP packet. Internal use only.
+ angle_path - File of angle calibration parameters. Internal use only.
+ start_angle、end_angle - Generally, mechanical LiDARs's point cloud's azimuths are in the range of [`0`, `360`]. Here you may assign a smaller range of [`start_angle`, `end_angle`).

+ split_frame_mode - How to split frame.
  + `SPLIT_BY_ANGLE` is by a user requested angle. User can specify it. This is default and suggested.
  + `SPLIT_BY_FIXED_BLKS` is by blocks theologically; 
  + `SPLIT_BY_CUSTOM_BLKS` is by user requested blocks. 

```c++
enum SplitFrameMode
{
  SPLIT_BY_ANGLE = 1,
  SPLIT_BY_FIXED_BLKS,
  SPLIT_BY_CUSTOM_BLKS
};
```
+ split_angle - If `split_frame_mode`=`SPLIT_BY_ANGLE`, then `split_angle` is the requested angle to split.
+ num_blks_split - If `split_frame_mode`=`SPLIT_BY_CUSTOM_BLKS`，then `num_blks_split` is blocks.


