# rs_driver 配置参数介绍

## 1 参数文件

文件`rs_driver/src/rs_driver/driver_param.h`中， 定义了rs_driver的配置参数。

rs_driver的参数主要保存在RSDriverParam、RSDecoderParam、和RSInputParam三个结构中。

## 2 RSDriverParam

RSDriverParam中包括RSDecoderParam和RSInputParam。

```c++
typedef struct RSDriverParam
{
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; ///< Input type
  RSInputParam input_param;
  RSDecoderParam decoder_param;
} RSDriverParam;
```

+ 成员lidar_type - 指定雷达的类型。
  + 雷达有两个大类，一是机械式雷达，二是MEMS雷达。RSDecoderParam中的部分参数只针对机械式雷达使用。

```c++
enum LidarType
{
  RS16 = 1,
  RS32,
  RSBP,
  RS128,
  RS80,
  RSHELIOS,
  RSROCK,
  RSM1
};
```

+ 成员input_type - 指定雷达的数据从哪里来。
  + ONLINE_LIDAR是在线雷达；PCAP_FILE是PCAP文件，使用第三方工具抓包得到；RAW_PACKET用于Packet录制与回放。

```c++
enum InputType
{
  ONLINE_LIDAR = 1,
  PCAP_FILE,
  RAW_PACKET
};
```


## 3 RSDecoderParam

RSDecoderParam指定雷达的Packet解析器如何工作。

```c++
typedef struct RSDecoderParam
{
  bool use_lidar_clock = false;
  bool dense_points = false;
  bool ts_first_point = false;
  bool wait_for_difop = true;
  RSTransformParam transform_param;
  bool config_from_file = false;
  std::string angle_path = "";
  float min_distance = 0.2f;
  float max_distance = 200.0f;

  // The following parameters are only for mechanical Lidars.
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;
  float split_angle = 0.0f;
  uint16_t num_blks_split = 1;
  float start_angle = 0.0f;
  float end_angle = 360.0f;
} RSDecoderParam;
```

如下参数针对所有雷达。
+ use_lidar_clock - 指定使用雷达还是主机的时间戳。
  + 如果`use_lidar_clock`=`true`，则使用雷达写入Packet中的时间戳；如果`use_lidar_clock`=`false`,则使用rs_driver在主机端产生的时间戳。
+ dense_points - 指定点云是否是dense的。
  + 如果`dense_points`=`false`, 则点云中包含NAN点；如果`dense_points`=`true`,则去除点云中的NAN点。
+ ts_first_point - 指定点云的时间戳来自点云的第一个点，还是最后第一个点。
  + 如果`ts_first_point`=`true`, 则第一个点的时间作为点云的时间戳；如果`ts_first_point`=`false`,则最后一个点的时间作为点云的时间戳。
+ wait_for_difop - 解析MSOP Packet之前，是否等待DIFOP Packet。
  + DIFOP Packet中包含角度校准等参数数据。如果没有这个数据，rs_driver输出的点云将是扁平的。
  + 在rs_driver不输出点云时，设置`wait_for_difop`=`false`，有助于定位问题的位置。
+ transform_param - 指定点的坐标转换参数。这个选项只有在CMake选项`ENABLE_TRANSFORM`=`ON`时才有效。

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

+ config_from_file - 指定雷达本身的配置参数是从文件中读入，还是从DIFOP Packet中得到。仅内部调试使用。
+ angle_path - 雷达的角度校准参数文件。仅内部调试使用。
+ min_distance、max_distance - 测距的最大、最小值。仅内部调试使用。

如下参数仅针对机械式雷达。
+ split_frame_mode - 指定分帧模式. 
  + `SPLIT_BY_ANGLE`是按`用户指定的角度`分帧；`SPLIT_BY_FIXED_BLKS`是按`理论上的每圈BLOCK数`分帧；`SPLIT_BY_CUSTOM_BLKS`按`用户指定的BLOCK数`分帧。默认值是`SPLIT_BY_ANGLE`。一般不建议使用其他两种模式。

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

+ start_angle、end_angle - 机械式雷达一般输出的点云的水平角在[`0`, `360`]之间，这里可以指定一个更小的范围[`start_angle`, `end_angle`)。

## 4 RSInputParam

RSInputParam指定rs_driver得到有关雷达的Packet源的参数。

如下参数针对`ONLINE_LIDAR`和`PCAP_FILE`。
+ msop_port - 指定主机上的本地UDP端口，接收MSOP Packet
+ difop_port - 指定主机上的本地UDP端口，接收DIFOP Packet

如下参数仅针对ONLINE_LIDAR。
+ host_address - 指定本地的IP地址，接收MSOP/DIFOP Packet
+ group_address - 指定一个组播组。rs_driver将`host_address`指定的网卡加入这个组播组，以便接收MSOP/DIFOP Packet。

如下参数仅针对PCAP_FILE。
+ pcap_path - PCAP文件的全路径
+ pcap_repeat - 指定是否重复播放PCAP文件
+ pcap_rate - rs_driver按`理论上的雷达帧率`播放PCAP文件。`pcap_rate`可以在这个速度上指定一个比例值，加快/放慢播放速度。
+ use_vlan - 如果PCAP文件中的Packet包含VLAN层，可以指定`use_vlan`=`true`，跳过这一层。

```c++
typedef struct RSInputParam
{
  uint16_t msop_port = 6699;
  uint16_t difop_port = 7788;
  std::string host_address = "0.0.0.0";
  std::string group_address = "0.0.0.0";

  // The following parameters are only for PCAP_FILE
  std::string pcap_path = "";
  bool pcap_repeat = true;
  float pcap_rate = 1.0;
  bool use_vlan = false;
} RSInputParam;

```
