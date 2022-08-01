# **rs_driver**  

[中文介绍](README_CN.md) 

## 1 Introduction

**rs_driver** is the driver kernel for the RoboSense LiDARs.

## 2 Supported LiDARs

Below are the supported LiDARS.

- RS-LiDAR-16
- RS-LiDAR-32
- RS-Bpearl
- RS-Helios
- RS-Helios-16P
- RS-Ruby-128
- RS-Ruby-80
- RS-Ruby-48
- RS-Ruby-Plus-128
- RS-Ruby-Plus-80
- RS-Ruby-Plus-48
- RS-LiDAR-M1

## 3 Supported Platforms

**rs_driver** is compatible with the following platforms and compilers. Note the compiler should support C++ 14.

- Ubuntu (16.04, 18.04)
  - gcc (4.8+)

- Windows
  - MSVC ( tested with Win10 / VS2019)

## 4 Dependency Libraries

**rs_driver** depends on the following third-party libraries. Please install them before compiling **rs_driver**.

- libpcap (optional, needed to parse PCAP file)
- Eigen3 (optional, needed to use the internal transformation function)
- PCL (optional, needed to build the visualization tool)
- Boost (optional, needed to build the visualization tool)

## 5 Compile On Ubuntu

### 5.1 Dependency Libraries

```sh
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
```

### 5.2 Compilation

```bash
cd rs_driver
mkdir build && cd build
cmake .. && make -j4
```

### 5.3 Installation

```bash
sudo make install
```

### 5.4 Use rs_driver as a third party library

In your ```CMakeLists.txt```, find the **rs_driver** package and link to it .

```cmake
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${rs_driver_LIBRARIES})
```

### 5.5 Use rs_driver as a submodule

Add **rs_driver** into your project as a submodule. 

In your ```CMakeLists.txt```, find the **rs_driver** package and link to it .

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${rs_driver_LIBRARIES})
```

## 6 Compile On Windows

### 6.1 Dependency Libraries

#### libpcap

Install [libpcap runtime library](https://www.winpcap.org/install/bin/WinPcap_4_1_3.exe).

Unzip [libpcap's developer's pack](https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip) to your favorite location, and add the path to the folder ```WpdPack_4_1_2/WpdPack``` to the environment variable ```PATH``` . 

#### PCL

To compile with VS2019, please use the official installation package [PCL All-in-one installer](https://github.com/PointCloudLibrary/pcl/releases).

Select the "Add PCL to the system PATH for xxx" option during the installation.

![](./doc/img/01_install_pcl.PNG)

### 6.2 Installation

Installation is not supported on Windows.

## 7 Quick Start

**rs_driver** offers two demo programs in ```rs_driver/demo```.

- demo_online.cpp
- demo_pcap.cpp

Please refer to them for usage of the rs_driver API. 

`demo_pcap` is based on libpcap.

To build `demo_online` and `demo_pcap`, enable the option COMPILE_DEMOS when configure the project.

```bash
cmake -DCOMPILE_DEMOS=ON ..
```

For more info about how to decode an online Lidar, Please refer to [Decode online LiDAR](doc/howto/how_to_decode_online_lidar.md)

For more info about how to decode a PCAP file, Please refer to [Decode pcap bag](doc/howto/how_to_decode_pcap_file.md)

## 8 Visualization of Point Cloud

**rs_driver** offers a visualization tool `rs_driver_viwer` in ```rs_driver/tool``` , which is based on PCL.

To build it, enable the option CMOPILE_TOOLS when configuring the project.

```bash
cmake -DCOMPILE_TOOLS=ON ..
```

For more info about how to use the `rs_driver_viewer`, please refer to [Visualization tool guide](doc/howto/how_to_use_rs_driver_viewer.md) 

## 9 More Topics

For more topics, Please refer to:

Trasformation function: [Transformation guide](doc/howto/how_to_transform_pointcloud.md) 
Network configuration advanced topics: [Advanced Topics](doc/howto/online_lidar_advanced_topics.md) 

For more info about the `rs_driver` API, Please refer to:
- **Point Cloud message definition**: ```rs_driver/src/rs_driver/msg/point_cloud_msg.hpp```, ```rs_driver/src/rs_driver/msg/pcl_point_cloud_msg.hpp```
- **API definition**: ```rs_driver/src/rs_driver/api/lidar_driver.hpp```
- **Parameters definition**: ```rs_driver/src/rs_driver/driver/driver_param.hpp```, 
- **Error code definition**: ```rs_driver/src/rs_driver/common/error_code.hpp```

