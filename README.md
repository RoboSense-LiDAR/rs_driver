# **rs_driver** 

### 1 工程简介
  **rs_driver**为速腾聚创雷达驱动内核。支持**RS-LiDAR-16**、**RS-LiDAR-32**、**RS-Bpearl**和**RS-128**的点云数据解析，方便用户二次开发使用。



### 2 依赖介绍

- Boost
- pthread
- pcap

Boost 与 pthread 均为系统库，可直接链接使用。 

Pcap则需使用以下指令安装:

```sh
sudo apt-get install -y  libpcap-dev
```



### 3 使用方式

#### 3.1 安装使用

首先安装驱动

```sh
cd rs_driver
mkdir build && cd build
cmake .. && make -j4
sudo make install
```

使用时，需要在CMakeLists文件中链接雷达驱动

```cmake
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(project ${rs_driver_LIBRARIES})
```

#### 3.2 嵌入使用

直接将rs_driver放入用户工程内，在CMakeLists中链接即可

```cmake
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/rs_driver/cmake)
include(rs_driver)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(project ${rs_driver_LIBRARIES})
```



### 4 快速上手

[在线连接雷达](doc/howto/how_to_online_use_driver.md)

[离线解析pcap包](doc/howto/how_to_offline_decode_pcap.md)



### 5 其他资料

[参数简介](doc/intro/parameter_intro.md)

[消息简介](doc/intro/message_intro.md)

[异常简介](doc/intro/errcode_intro.md)

[接口简介](doc/intro/api_intro.md)







---



### 1 Introduction

  **rs_driver** is the driver code for RoboSense LiDAR,  include **RS-LiDAR-16**、**RS-LiDAR-32**、**RS-Bpearl** and **RS-128** . It can be used to extract packets from lidar to point cloud, and it is convenient for users to do advanced development.



### 2 Dependency 

- Boost
- pthread
- pcap

Boost and pthread are libraries of system, that can be linked directly. 

Pcap need to be installed as follow:

```sh
sudo apt-get install -y  libpcap-dev
```



### 3 Usage

#### 3.1 Install driver to use

Install the driver.

```sh
cd rs_driver
mkdir build && cd build
cmake .. && make -j4
sudo make install
```

Then find the driver package and link it in the project CMakeLists.

```cmake
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(project ${rs_driver_LIBRARIES})
```

#### 3.2 Embed driver in project

Put the rs_driver in the project and link it in the CMakeLists.

```cmake
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/rs_driver/cmake)
include(rs_driver)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(project ${rs_driver_LIBRARIES})
```



### 4 Quick Start

[Online connect LiDAR](doc/howto/how_to_online_use_driver.md)

[Offline decode pcap bag](doc/howto/how_to_offline_decode_pcap.md)



### 5 Others

[Intro to parameters](doc/intro/parameter_intro.md)

[Intro to message types](doc/intro/message_intro.md)

[Intro to error codes](doc/intro/errcode_intro.md)

[Intro to api](doc/intro/api_intro.md)









