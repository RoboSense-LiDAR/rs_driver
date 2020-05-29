## rs_driver

### 1. 工程简介
  **rs_driver**为速腾聚创雷达驱动内核。支持**RS-LiDAR-16**、**RS-LiDAR-32**、**RS-Bpearl**和**RS-Ruby**的点云数据解析，方便用户二次开发使用。



### 2. 依赖介绍

- Boost
- pthread
- pcap

Boost 与 pthread 均为系统库，可直接链接使用。 Pcap则需使用以下指令安装:

```
sudo apt-get install -y  libpcap-dev
```



### 3. 驱动安装

```sh
    cd rs_driver
    mkdir build && cd build
    cmake .. && make -j4
    sudo make install
```

-----------------------------------------------------------------
### 1. Introduction
  **rs_driver** is a driver code for RoboSense LiDAR,  include **RS-LiDAR-16**、**RS-LiDAR-32**、**RS-Bpearl** and **RS-Ruby** now. It can be used to extract packets of lidar to pointcloud, and is convenient for users to do secondary development.



### 2. Dependency 

- Boost
- pthread
- pcap

Boost and pthread are libraries of system, that can be linked directly. pcap need to install as follow:

```
sudo apt-get install -y  libpcap-dev
```

### 3. Install

```sh
    cd rs_driver
    mkdir build && cd build
    cmake .. && make -j4
    sudo make install
```