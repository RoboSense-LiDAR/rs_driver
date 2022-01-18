# How to use multicast group

## 1 Introduction

This document illustrates how to receive point cloud in the multicast group mode. This is only for the Linux platform.

## 2 Steps

Suppose the multicast group address is ```224.1.1.1```, and the host addres is ```192.168.1.102```.

The Lidar sends packets to the multicat group address. The NIC with the host address should be on the same network with the Lidar. 

The driver will make the NIC join to the multicast group address, and then get the packets from it.

### 2.1 Set up parameters

Set the  parameter ```group_address``` and ```host_address```. 

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.input_param.host_address = "192.168.1.102"; ///< Set the host address.
param.input_param.group_address = "224.1.1.1";    ///< Set the multicast group address.
param.lidar_type = LidarType::RS16;               ///< Set the lidar type. Make sure this type is correct 
```

### 2.4 Run

Run the demo program. 











