# PCAP文件 - 高级主题

## 1 简介

RoboSense雷达可以工作在单播/组播/广播模式下，也可以工作在VLAN环境下，也可以加入用户自己的层。

本文说明了在每种场景下如何配置rs_driver的参数。

阅读本文之前，请先阅读 [在线雷达-高级主题](./online_lidar_advanced_topics_CN.md).

## 2 一般场景

在下列场景下，使用如下配置代码解码PCAP文件。
+ 广播/组播/单播模式
+ PCAP文件中有多个雷达

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::PCAP_FILE;          ///< get packet from online lidar
param.input_param.pcap_path = "/home/robosense/lidar.pcap";  ///< Set the pcap file path
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

一个例外是：PCAP文件中有多个雷达数据，但这些雷达目的端口相同，使用不同的目的IP地址来区分。这种情况不支持。

## 3 VLAN

有些场景下，雷达工作在VLAN环境下。这时MSOP/DIFOP包带VLAN层，如下图。

![](./img/12_vlan_layer.png)

rs_driver使用libpcap库解析PCAP文件，可以得到完整的、包括VLAN层的MSOP/DIFOP包。

要剥除VLAN层，只需要设置`use_vlan=true`。

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::PCAP_FILE;          ///< get packet from online lidar
param.input_param.pcap_path = "/home/robosense/lidar.pcap";  ///< Set the pcap file path
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.input_param.use_vlan = true;                ///< Whether to use VLAN layer.
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

## 4 User Layer, Tail Layer 

某些场景下，用户可能在MSOP/DIFOP数据前后加入自己的层。
+ USER_LAYER 在MSOP/DIFOP数据之前，TAIL_LAYER在MSOP/DIFOP数据之后。

![](./img/12_user_layer.png)

这些层是UDP数据的一部分，所以rs_driver可以自己剥除他们。只需要告诉它每个层的字节数就可以。

如下的例子中，指定USER_LAYER为8字节，TAIL_LAYER为4字节。

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::PCAP_FILE;          ///< get packet from online lidar
param.input_param.pcap_path = "/home/robosense/lidar.pcap";  ///< Set the pcap file path
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.input_param.user_layer_bytes = 8;           ///< user layer bytes. there is no user layer if it is 0
param.input_param.tail_layer_bytes = 4;           ///< tail layer bytes. there is no user layer if it is 0
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```















