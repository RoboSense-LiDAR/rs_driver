# 如何对点云作坐标转换

## 1 简介

阅读本文前，请先阅读 [连接在线雷达](./how_to_decode_online_lidar_CN.md) or [解码PCAP文件](./how_to_decode_pcap_file_CN.md)。

本文说明如何使用坐标转换功能，将点云变换到不同位置，不同角度上去。

进行旋转的顺序依次是**yaw - pitch - row**。x, y, z的单位是```米```, roll, pitch, yaw的单位是```度```。

警告：

对点做坐标转化，需要消耗大量的CPU资源。提供这个功能，仅用于测试目的，所以***一定不要在你的产品中使能它***。

## 2 步骤

### 2.1 编译

要使用坐标转换功能，需要使能编译选项```ENABLE_TRANSFORM=ON```。

```bash
cmake -DENABLE_TRANSFORM=ON ..
```

### 2.2 配置参数

配置坐标转换的参数，这些参数的默认值是```0```。

如下的例子中，x=1, y=0, z=2.5, roll=0.1, pitch=0.2, yaw=1.57. 

```c++
RSDriverParam param;                            ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;     /// get packet from online lidar
param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct

param.decoder_param.transform_param.x = 1;	  ///< unit: m
param.decoder_param.transform_param.y = 0;	  ///< unit: m
param.decoder_param.transform_param.z = 2.5;	  ///< unit: m
param.decoder_param.transform_param.roll = 0.1; ///< unit: radian
param.decoder_param.transform_param.pitch = 0.2;///< unit: radian
param.decoder_param.transform_param.yaw = 1.57; ///< unit: radian

```

