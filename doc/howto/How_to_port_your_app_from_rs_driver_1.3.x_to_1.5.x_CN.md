# 如何将基于rs_driver v1.3.x的应用程序移植到v1.5.x

## 1 为什么要移植？

本文说明了如何将基于rs_driver v1.3.x的应用程序移植到v1.5.x上。

rs_driver v1.5.x是对v1.3.x较大重构后的版本。它主要做了如下改进。
+ 明显减少CPU资源的占用率
+ 增加可选的编译选项，解决在某些平台上丢包的问题
+ 去除对Boost库的依赖，更容易移植到除x86以外的其他平台上。
+ 重构网络部分和解码部分的代码，完整支持不同场景下的配置
+ 完善帮助文档，尤其是提供了[源代码解析](../src_intro/rs_driver_intro_CN.md)文档，帮助用户深入理解驱动的实现，以方便功能裁剪。

## 2 如何移植？

rs_driver的接口包括了两个部分：RSDriverParam和LidarDriver。

### 2.1 RSDriverParam

#### 2.1.1 RSDriverParam

RSDriverParam包括了一些可以改变rs_driver行为的参数。

在v1.3.2中，RSDriverParam定义如下。

```c++
typedef struct RSDriverParam  ///< The LiDAR driver parameter
{
  RSInputParam input_param;          ///< Input parameter
  RSDecoderParam decoder_param;      ///< Decoder parameter
  std::string angle_path = "null";   ///< Path of angle calibration files(angle.csv).Only used for internal debugging.
  std::string frame_id = "rslidar";  ///< The frame id of LiDAR message
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  bool wait_for_difop = true;              ///< true: start sending point cloud until receive difop packet
  bool saved_by_rows = false;        ///< true: the output point cloud will be saved by rows (default is saved by columns)
};
```

在v1.5.x中，RSDriverParam定义如下。

```c++
typedef struct RSDriverParam  ///< The LiDAR driver parameter
{
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; ///< Input type
  RSInputParam input_param;                ///< Input parameter
  RSDecoderParam decoder_param;            ///< Decoder parameter
};
```

变动如下：
+ 加入新成员`input_type`。这个成员指定数据源的类型：`ONLINE_LIDAR`, `PCAP_FILE`, or `RAW_PACKET`.
+ 成员`wait_for_difop` 指定在处理MSOP包之前是否先等待DIFOP包。对于机械式雷达，如果没有DIFOP包中的垂直角数据，得到的点云就是扁平的。这个选项是与解码相关的，所以把它移到RSDecoderParam中。
+ 删除成员`save_by_rows`。默认情况下，点在点云中按照扫描的通道次序保存在vector中，一轮扫描的点在竖直的线上，也就是按照列保存。成员`saved_by_rows` 可以指定按照水平的线保存点，也就是按照行保存。删除这个成员有两个原因：
  + 在按照列保存点的vector中，按照行的顺序检索点，也是容易的，跳过通道数就可以了。
  + 这个选项的实现方式是将点云重排一遍，但是复制点云的代码太大了。
+ 成员`angle_path`指定垂直角从指定的文件读取，而不是解析DIFOP包得到。这个选项是关于数据源的，所以移到RSInputParam。
+ 成员`frame_id` 来自ROS的概念，rs_driver本身和rs_driver的其他使用者并不需要它，所以删除它。适配ROS的驱动 `rslidar_sdk`会创建和处理它。

#### 2.1.2 RSInputParam

在v1.3.x中，RSInputParam定义如下。

```c++
typedef struct RSInputParam  ///< The LiDAR input parameter
{
  std::string device_ip = "192.168.1.200";     ///< Ip of LiDAR
  std::string multi_cast_address = "0.0.0.0";  ///< Address of multicast
  std::string host_address = "0.0.0.0";        ///< Address of host
  uint16_t msop_port = 6699;                   ///< Msop packet port number
  uint16_t difop_port = 7788;                  ///< Difop packet port number
  bool read_pcap = false;          ///< true: The driver will process the pcap through pcap_path. false: The driver will
                                   ///< Get data from online LiDAR
  double pcap_rate = 1;            ///< Rate to read the pcap file
  bool pcap_repeat = true;         ///< true: The pcap bag will repeat play
  std::string pcap_path = "null";  ///< Absolute path of pcap file
  bool use_vlan = false;           ///< Vlan on-off
  bool use_someip = false;         ///< Someip on-off
  bool use_custom_proto = false;   ///< Customer Protocol on-off
}；
```

在v1.5.x中，RSInputParam定义如下。

```c++
typedef struct RSInputParam  ///< The LiDAR input parameter
{
  uint16_t msop_port = 6699;                   ///< Msop packet port number
  uint16_t difop_port = 7788;                  ///< Difop packet port number
  std::string host_address = "0.0.0.0";        ///< Address of host
  std::string group_address = "0.0.0.0";       ///< Address of multicast group
  std::string pcap_path = "";                  ///< Absolute path of pcap file
  bool pcap_repeat = true;                     ///< true: The pcap bag will repeat play
  float pcap_rate = 1.0f;                      ///< Rate to read the pcap file
  bool use_vlan = false;                       ///< Vlan on-off
  uint16_t user_layer_bytes = 0;    ///< Bytes of user layer. thers is no user layer if it is 0
  uint16_t tail_layer_bytes = 0;    ///< Bytes of tail layer. thers is no tail layer if it is 0
};
```

变动如下：
+ 删除成员`device_ip`。rs_driver接收MSOP/DIFOP包时，其实不需要这个值。
+ 成员`multi_cast_address`改名为`group_address`。这里想表达的含义是，在组播模式下，rs_driver会将主机加入`group_address`指定的组播组。
+ 删除成员`read_pcap`。 既然`RSDriverParam.input_type` 已经指定了数据源类型，`read_pcap`就不需要了。
+ 删除成员`use_someip` and `use_custom_proto`。它们的功能已经被新的选项 `RSInputParam.user_layer_bytes`所取代。

#### 2.1.3 RSDecoderParam

在v1.3.x中，RSDecoderParam定义如下。

```c++
typedef struct RSDecoderParam  ///< LiDAR decoder parameter
{
  float max_distance = 200.0f;                                       ///< Max distance of point cloud range
  float min_distance = 0.2f;                                         ///< Minimum distance of point cloud range
  float start_angle = 0.0f;                                          ///< Start angle of point cloud
  float end_angle = 360.0f;                                          ///< End angle of point cloud
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  ///< 1: Split frames by cut_angle;
                                                                     ///< 2: Split frames by fixed number of packets;
                                                             ///< 3: Split frames by custom number of packets (num_pkts_split)
  uint32_t num_pkts_split = 1;         ///< Number of packets in one frame, only be used when split_frame_mode=3
  float cut_angle = 0.0f;              ///< Cut angle(degree) used to split frame, only be used when split_frame_mode=1
  bool use_lidar_clock = false;        ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  RSTransformParam transform_param;    ///< Used to transform points
  RSCameraTriggerParam trigger_param;  ///< Used to trigger camera
};
```

在v1.5.x中，RSDecoderParam定义如下。

```c++
typedef struct RSDecoderParam  ///< LiDAR decoder parameter
{
  bool config_from_file = false; ///< Internal use only for debugging
  std::string angle_path = "";   ///< Internal use only for debugging
  bool wait_for_difop = true;    ///< true: start sending point cloud until receive difop packet
  float min_distance = 0.2f;     ///< Minimum distance of point cloud range
  float max_distance = 200.0f;   ///< Max distance of point cloud range
  float start_angle = 0.0f;      ///< Start angle of point cloud
  float end_angle = 360.0f;      ///< End angle of point cloud
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  
                                 ///< 1: Split frames by split_angle;
                                 ///< 2: Split frames by fixed number of blocks;
                                 ///< 3: Split frames by custom number of blocks (num_blks_split)
  float split_angle = 0.0f;      ///< Split angle(degree) used to split frame, only be used when split_frame_mode=1
  uint16_t num_blks_split = 1;   ///< Number of packets in one frame, only be used when split_frame_mode=3
  bool use_lidar_clock = false;  ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool dense_points = false;     ///< true: discard NAN points; false: reserve NAN points
  RSTransformParam transform_param; ///< Used to transform points
};
```

变动如下：
+ 成员`cut_angle`改名为`split_angle`, 以便与成员 `split_frame_mode`的名字保持一致。
+ 增加成员`config_from_file`。只有这个成员为true，成员`angle_path`才有效。
+ 成员`num_pkts_split`改名为 `number_blks_split`. 因为在v1.3.x中，分帧的最小单位是packet，而在v1.5.x中，分帧单位改成了更小的block。

### 2.2 LidarDriver

创建LidarDriver实例，并调用它的成员函数，可以运行rs_driver。

在v1.3.x中，LidarDriver定义如下。

```c++
template <typename PointT>
class LidarDriver
{
public:

  inline void regRecvCallback(const std::function<void(const PointCloudMsg<PointT>&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }
  ...
};
```


在v1.5.x中，LidarDriver定义如下。

```c++
template <typename T_PointCloud>
class LidarDriver
{
public:

 inline void regPointCloudCallback(const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud)
  {
    driver_ptr_->regPointCloudCallback(cb_get_cloud, cb_put_cloud);
  }
  ...
};
```

变动如下：
+ LidarDriver类的模板参数从点 `PointT`改成了点云 `T_PointCloud`。
+ LidarDriver在v1.3.x时需要一个点云回调函数，用于rs_driver把构建好的点云返回给调用者，在v1.5.x中则需要两个回调函数，除了返回构建好的点云，还需要给rs_driver提供空闲的点云实例。
关于这两点，可以参考[Decode online Lidar](./how_to_decode_online_lidar_CN.md)，得到更详细的说明。



