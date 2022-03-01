# How to use multi-cast function

## 1 Introduction

This document will show you how to use rs_driver to receive point cloud from the LiDAR working in multi-cast mode.

## 2 Steps (Linux)

Suppose the multi-cast address of LiDAR is ```224.1.1.1```, and the host address is ```192.168.1.102```. 

The host address should be in the same network with the Lidar. In the other words, it can ping to the Lidar.

### 2.1 Set up parameters

Set up the  parameter ```multi_cast_address``` and ```host_address```. Here is an example for online lidar.

```c++
RSDriverParam param;                                             ///< Create a parameter object
param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
param.input_param.host_address = "192.168.1.102"; ///< Set the host address.
param.input_param.multi_cast_address = "224.1.1.1"; ///< Set the multi-cast address.
param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct 

```

### 2.4 Run

Run the program. 











