# How to visualize the point cloud with rs_driver_viewer

## 1 Introduction

The rs_driver_viewer is a visualization tool for the point cloud. This document illustrates how to use it.

## 2 Compile and Run

Compile the driver with the option COMPILE_TOOLS=ON. 

```bash
cmake -DCOMPILE_TOOS=ON ..
```

Run the tool.

```bash
./tool/rs_driver_viewer 
```

### 2.1 Help Menu

- -h/--help

   print the argument menu 

- -msop

   MSOP port number of LiDAR, the default value is *6699*

- -difop

   DIFOP port number of LiDAR, the default value is *7788*
   
- -host

   the host address

- -group

   the multicast group address

- -pcap

   Full path of the PCAP file. If this argument is empty, the driver read packets from online Lidar, else from the PCAP file. 

- -type

   LiDAR type, the default value is *RS16*

- -x

   Transformation parameter, default is 0, unit: m

- -y

   Transformation parameter, default is 0, unit: m

- -z

   Transformation parameter, default is 0, unit: m

- -roll

   Transformation parameter, default is 0, unit: radian

- -pitch

   Transformation parameter, default is 0, unit: radian

- -yaw

   Transformation parameter, default is 0, unit: radian

Note:

**The point cloud transformation function is available only if the CMake option ENABLE_TRANSFORM is ON.**

## 3 Examples

- Decode from an online RS128 LiDAR. Its MSOP port is ```9966```, and difop port is ```8877```

  ```bash
  rs_driver_viewer -msop 9966 -difop 8877 -type RS128 
  ```

- Decode from a PCAP file with RSHELIOS LiDAR data.

  ```bash
  rs_driver_viewer -pcap /home/robosense/helios.pcap -type RSHELIOS
  ```

- Decode with the coordinate transformation parameters: x=1.5, y=2, z=0, roll=1.57, pitch=0, yaw=0

  ```bash
  rs_driver_viewer -type RS16 -x 1.5 -y 2 -roll 1.57 
  ```

  

