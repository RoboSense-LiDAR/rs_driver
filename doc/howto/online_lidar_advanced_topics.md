# Online Lidar - Advanced Topics

## 1 Introduction

 The RoboSense Lidar may work in unicast/multicast/broadcast mode, with VLAN layer and with user layers.

This document illustrates how to configure the driver in each case.

Before reading this document, please be sure that you have read [Decode online Lidar](./how_to_decode_online_lidar.md).

## 2 Unicast, Multicast and Broadcast

### 2.1 Broadcast mode

The simplest way is broadcast mode. 

The Lidar sends MSOP/DIFOP packets to the host machine (The driver runs on it). For simplicity, the DIFOP port is ommited here.
+ The Lidar sends to `255.255.255.255` : `6699`, and the host binds to port `6699`.

![](../img/12_broadcast.png)

Below is how to configure RSDriverParam variable.

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

### 2.2 Unicast mode

To reduce the network load, the Lidar is suggested to work in unicast mode.
+ The Lidar sends to `192.168.1.102` : `6699`, and the host binds to port `6699`.

![](../img/12_unicast.png)

Below is how to configure the RSDriverParam variable. In fact, it is same with the broadcast case.

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```


### 2.3 Multicast mode

The Lidar may also works in multicast mode.
+ The lidar sends to `224.1.1.1`:`6699` 
+ The host binds to port `6699`. And it makes local NIC (Network Interface Card) join the multicast group `224.1.1.1`. The local NIC's IP is `192.168.1.102`.

![](../img/12_multicast.png)

Below is how to configure the RSDriverParam variable.

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.group_address = "224.1.1.1";    ///< Set the multicast group address.
param.input_param.host_address = "192.168.1.102"; ///< Set the host address.
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type. Make sure this type is correct 
```

## 3 Multiple Lidars

### 3.1 Different remote ports

If you have two Lidars, it is suggested to set different remote ports.
+ First Lidar sends to `192.168.1.102`:`6699`, and the first driver instance binds to `6699`.
+ Second Lidar sends to `192.168.1.102`:`5599`, and the second driver instance binds to `5599`.

![](../img/12_multi_lidars_port.png)

Below is how to configure the RSDriverParam variables.

```c++
RSDriverParam param1;                              ///< Create a parameter object for Lidar 192.168.1.200
param1.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param1.input_param.msop_port = 6699;               ///< Set the lidar msop port number
param1.input_param.difop_port = 7788;              ///< Set the lidar difop port number
param1.lidar_type = LidarType::RS32;               ///< Set the lidar type.

RSDriverParam param2;                              ///< Create a parameter object for Lidar 192.168.1.201
param2.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param2.input_param.msop_port = 5599;               ///< Set the lidar msop port number
param2.input_param.difop_port = 6688;              ///< Set the lidar difop port number
param2.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

### 3.2 Different remote IPs

An alternate way is to set different remote IPs. 
+ The host has two NICs: `192.168.1.102` and `192.168.1.103`.
+ First Lidar sends to `192.168.1.102`:`6699`, and the first driver instance binds to `192.168.1.102:6699`.
+ Second Lidar sends to `192.168.1.103`:`6699`, and the second driver instance binds to `192.168.1.103:6699`.

![](../img/12_multi_lidars_ip.png)

Below is how to configure the RSDriverParam variables.

```c++
RSDriverParam param1;                              ///< Create a parameter object for Lidar 192.168.1.200
param1.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param1.input_param.host_address = "192.168.1.102"; ///< Set the host address.
param1.input_param.msop_port = 6699;               ///< Set the lidar msop port number
param1.input_param.difop_port = 7788;              ///< Set the lidar difop port number
param1.lidar_type = LidarType::RS32;               ///< Set the lidar type.

RSDriverParam param2;                              ///< Create a parameter object for Lidar 192.168.1.201
param2.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param2.input_param.host_address = "192.168.1.103"; ///< Set the host address.
param2.input_param.msop_port = 6699;               ///< Set the lidar msop port number
param2.input_param.difop_port = 7788;              ///< Set the lidar difop port number
param2.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

## 4 VLAN

In some user cases, The Lidar may work on VLAN.  Its packets have a VLAN layer.

![](../img/12_vlan_layer.png)

The driver cannot parse this packet. Instead, it depends on a virtual NIC to strip the VLAN layer.

Below is an example.
+ The Lidar works on VLAN `80`. It sends packets to `192.168.1.102` : `6699`. The packet has a VLAN layer.
+ Suppose there is a physical NIC `eno1` on the host.  It receives packets with VLAN layer.

![](../img/12_vlan.png)

To strip the VLAN layer, create a virtual NIC `eno1.80` on `eno1`, and assign IP `192.168.1.102` to it.

```
sudo apt-get install vlan -y
sudo modprobe 8021q

sudo vconfig add eno1 80
sudo ifconfig eno1.80 192.168.1.102 up
```

Now the driver may take `eno1.80` as a general NIC, and receives packets without VLAN layer.

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

## 5 User Layer, Tail Layer 

In some user cases, User may add extra layers before or after the MSOP/DIFOP packet.
+ USER_LAYER is before the packet and TAIL_LAYER is after it.

![](../img/12_user_layer.png)

These extra layers are parts of UDP data. The driver can strip them. 

To strip them, just give their lengths in bytes. 

In the following example, USER_LAYER is 8 bytes, and TAIL_LAYER is 4 bytes.

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.input_param.user_layer_bytes = 8;           ///< user layer bytes. there is no user layer if it is 0
param.input_param.tail_layer_bytes = 4;           ///< tail layer bytes. there is no user layer if it is 0
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```















