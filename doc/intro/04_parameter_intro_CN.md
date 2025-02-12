# 4 **配置参数介绍**



## 4.1 概述

文件`rs_driver/src/rs_driver/driver_param.h`中， 定义了`rs_driver`的配置选项。

这些配置选项定义在如下结构中。

+ RSDriverParam
+ RSDecoderParam
+ RSInputParam



## 4.2 RSDriverParam

RSDriverParam包括RSDecoderParam、RSInputParam、和其他选项。

```c++
typedef struct RSDriverParam
{
  LidarType lidar_type = LidarType::RS16;         ///< Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; ///< Input type
  RSInputParam input_param;
  RSDecoderParam decoder_param;
} RSDriverParam;
```

+ 成员`lidar_type` - 指定雷达的类型。这些雷达部分是机械式雷达，部分是MEMS雷达。RSDecoderParam中的部分选项只针对机械式雷达。

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

+ 成员`input_type` - 指定雷达的数据源类型
  + ONLINE_LIDAR是在线雷达；PCAP_FILE是包含MSOP/DIFOP Packet的PCAP文件；RAW_PACKET是使用者调用`rs_driver`的函数接口获得MSOP/DIFOP Packet，自己保存的数据。

```c++
enum InputType
{
  ONLINE_LIDAR = 1,
  PCAP_FILE,
  RAW_PACKET
};
```



## 4.3 RSInputParam

RSInputParam指定`rs_driver`的网络配置选项。

如下参数针对`ONLINE_LIDAR`和`PCAP_FILE`。
+ msop_port - 指定主机的本地端口，接收MSOP Packet
+ difop_port - 指定主机的本地端口，接收DIFOP Packet
+ imu_port - 指定主机的本地端口，接收IMU Packet
+ user_layer_bytes - 指定用户层数据的字节数。 用户层数据是MSOP Packet和DIFOP Packet中的一部分，这部分数据由用户自己定义。
+ tail_layer_bytes - 指定尾部数据的字节数。尾部数据是MSOP Packet的最后一部分，这部分数据由用户自己定义。

如下参数仅针对 `ONLINE_LIDAR`。

+ host_address - 指定主机网卡的IP地址，接收MSOP/DIFOP Packet
+ group_address - 指定一个组播组的IP地址。`rs_driver`将 `host_address`指定的网卡加入这个组播组，以便接收MSOP/DIFOP Packet。
+ socket_recv_buf - 指定socket的接收缓冲区大小。 `rs_driver`会接收MSOP Packet，因此需要足够大的缓冲区。

如下参数仅针对`PCAP_FILE`。
+ pcap_path - PCAP文件的全路径
+ pcap_repeat - 指定是否重复播放PCAP文件
+ pcap_rate - `rs_driver`按理论上的MSOP Packet时间间隔，模拟播放PCAP文件。`pcap_rate`可以在这个速度上指定一个比例值，加快或放慢播放速度。
+ use_vlan - 如果PCAP文件中的MSOP/DIFOP Packet包含VLAN层，可以指定`use_vlan`=`true`，跳过这一层。

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

RSDecoderParam指定雷达解码器的配置选项。

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

如下参数针对所有雷达。
+ min_distance、max_distance - 重置测距的最大、最小值。仅内部调试使用。
+ use_lidar_clock - 指定点云的时间采用MSOP Packet中的时间（雷达根据自身时间设置），还是主机的系统时间。
  + 如果`use_lidar_clock`=`true`，则采用MSOP Packet的，否则采用主机的。
+ dense_points - 指定点云是否是dense的。
  + 如果`dense_points`=`false`, 则点云中包含NAN点，否则去除点云中的NAN点。
+ ts_first_point - 指定点云的时间戳来自它的第一个点，还是最后第一个点。
  + 如果`ts_first_point`=`true`, 则第一个点的时间作为点云的时间戳，否则最后一个点的时间作为点云的时间戳。
+ wait_for_difop - 解析MSOP Packet之前，是否等待DIFOP Packet。
  + DIFOP Packet中包含垂直角等标定参数。如果没有这些参数，`rs_driver`输出的点云将是扁平的。
  + 在`rs_driver`不输出点云时，设置`wait_for_difop=false`，可以帮助定位问题。
+ transform_param - 指定点的坐标转换参数。这个选项只有在CMake编译宏`ENABLE_TRANSFORM=ON`时才有效。

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

如下参数仅针对机械式雷达。

+ config_from_file - 指定雷达本身的配置参数是从文件中读入，还是从DIFOP Packet中得到。这个选项仅内部调试使用。
+ angle_path - 雷达的角度标定参数文件。仅内部调试使用。
+ start_angle、end_angle - 机械式雷达一般输出的点云的水平角在[`0`, `360`]之间，这里可以指定一个更小的范围[`start_angle`, `end_angle`)。
+ split_frame_mode - 指定分帧模式
  + `SPLIT_BY_ANGLE`是按 `用户指定的角度`分帧；`SPLIT_BY_FIXED_BLKS`是按 `理论上的每圈BLOCK数`分帧；`SPLIT_BY_CUSTOM_BLKS`按 `用户指定的BLOCK数`分帧。默认值是 `SPLIT_BY_ANGLE`。一般不建议使用其他两种模式。

```c++
enum SplitFrameMode
{
  SPLIT_BY_ANGLE = 1,
  SPLIT_BY_FIXED_BLKS,
  SPLIT_BY_CUSTOM_BLKS
};
```
+ split_angle - 如果`split_frame_mode`=`SPLIT_BY_ANGLE`, 则`split_angle`指定分帧的角度
+ num_blks_split - 如果`split_frame_mode`=`SPLIT_BY_CUSTOM_BLKS`，则`num_blks_split`指定每帧的BLOCK数。

