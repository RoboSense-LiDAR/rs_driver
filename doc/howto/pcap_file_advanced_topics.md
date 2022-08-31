# PCAP file - Advanced Topics

## 1 Introduction

 The RoboSense LiDAR may work in unicast/multicast/broadcast mode, with VLAN layer and with user layers.

This document illustrates how to configure the driver in each case.

Before reading this document, please be sure that you have read [Online LiDAR - Advanced Topics](./online_lidar_advanced_topics.md).

## 2 General Case

Generally, below code is for decoding a PCAP file in these cases.
+ Broadcast/multicast/unicast mode
+ There are multiple LiDars in a file.

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::PCAP_FILE;          ///< get packet from online lidar
param.input_param.pcap_path = "/home/robosense/lidar.pcap";  ///< Set the pcap file path
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

The only exception is "Multiple Lidars with same ports but different IPs", which is not supported now.

## 3 VLAN

In some user cases, The LiDar may work on VLAN.  Its packets have a VLAN layer.

![](./img/12_vlan_layer.png)

rs_driver decodes PCAP file and gets all parts of MSOP packets, including the VLAN layer. 

To strip the VLAN layer, just set `use_vlan=true`.

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

In some user cases, User may add extra layers before or/and after the MSOP/DIFOP packet.
+ USER_LAYER is before the packet and TAIL_LAYER is after it.

![](./img/12_user_layer.png)

These extra layers are parts of UDP data. The driver can strip them. 

To strip them, just give their lengths in bytes. 

In the following example, USER_LAYER is 8 bytes, and TAIL_LAYER is 4 bytes.

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















