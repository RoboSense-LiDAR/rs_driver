# **rs_driver**  

[中文介绍](README_CN.md) 

## 1 Introduction

**rs_driver** is the driver kernel for the RoboSense LiDARs.

## 2 Supported LiDARs

- RS-LiDAR-16
- RS-LiDAR-32
- RS-Bpearl
- RS-Ruby
- RS-Ruby Lite
- RS-LiDAR-M1
- RS-Helios

## 3 Supported Platforms

**rs_driver** is compatible with the following platforms and compilers: 

- Ubuntu (16.04, 18.04)
  - gcc (4.8+)

- Windows
  - MSVC ( tested with VC2017 and VC2019)
  - Mingw-w64 (tested with x86_64-8.1.0-posix-seh-rt_v6-rev0 )

## 4 Dependency Libraries

**rs_driver** depends on the following third-party libraries. Please install them before compiling **rs_driver**.

- libpcap (optional, needed if parsing PCAP file)
- PCL (optional, needed if building the visualization tool)
- Eigen3 (optional, needed if use the internal transformation function)

## 5 Compilation and Installation

### 5.1 On Ubuntu

#### 5.1.1 Dependency Libraries

```sh
sudo apt-get install libpcap-dev libpcl-dev libeigen3-dev
```

#### 5.1.2 Compilation

```bash
cd rs_driver
mkdir build && cd build
cmake .. && make -j4
```

#### 5.1.3 Installation

```bash
sudo make install
```

#### 5.1.4 Use rs_driver as a third party library

In your ```CMakeLists.txt```, find the **rs_driver** package and link to it .

```cmake
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${rs_driver_LIBRARIES})
```

#### 5.1.5 Use rs_driver as a submodule

Add **rs_driver** into your project as a submodule. 

In your ```CMakeLists.txt```, find the **rs_driver** package and link to it .

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${rs_driver_LIBRARIES})
```

### 5.2 On Windows

#### 5.2.1 Dependency Libraries

##### libpcap

Install [libpcap runtime library](https://www.winpcap.org/install/bin/WinPcap_4_1_3.exe).

Download [libpcap's developer's pack](https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip) to your favorite location, and add the path to ```WpdPack_4_1_2/WpdPack``` folder to the environment variable ```PATH``` . 

##### PCL

+ MSVC

Please use the official installation package [All-in-one installer](https://github.com/PointCloudLibrary/pcl/releases).

Select the "Add PCL to the system PATH for xxx" option during the installation.

![](./doc/img/01_install_pcl.PNG)

+ Mingw-w64

Since there'are no installers for mingw-w64 compiler available, Please compile PCL out from source as instructed in this [tutorial](https://pointclouds.org/documentation/tutorials/compiling_pcl_windows.html). 

#### 5.2.2 Installation

Installation is not supported on Windows.

## 6 Quick Start

**rs_driver** offers two demo programs in ```rs_driver/demo```.

- demo_online.cpp
- demo_pcap.cpp

Please refer to them for usage of the rs_driver API. 

`demo_pcap` is based on libpcap.

To build `demo_online` and `demo_pcap`, enable the option COMPILE_DEMOS when configure the project.

```bash
cmake -DCOMPILE_DEMOS=ON ..
```

For more info about how to decode an online Lidar, Please refer to [Online connect LiDAR](doc/howto/how_to_online_use_driver.md)

For more info about how to decode a PCAP file, Please refer to [Decode pcap bag](doc/howto/how_to_decode_pcap.md)

## 7 Visualization of Point Cloud

**rs_driver** offers a visualization tool `rs_driver_viwer` in ```rs_driver/tool``` , which is based on PCL.

To build it, enable the option CMOPILE_TOOLS when configuring the project.

```bash
cmake -DCOMPILE_TOOLS=ON ..
```

For more info about how to use the `rs_driver_viewer`, please refer to [Visualization tool guide](doc/howto/how_to_use_rs_driver_viewer.md) 

## 8 More Topics

For more topics, Please refer to:

Multi-Cast function: [Multi-Cast](doc/howto/how_to_use_multi_cast_function.md) 
Trasformation function: [Transformation guide](doc/howto/ow_to_transform_pointcloud.md) 

For more info about the `rs_driver` API, Please refer to:
- **Parameters definition**: ```rs_driver/src/rs_driver/driver/driver_param.h```
- **Point Cloud message definition**: ```rs_driver/src/rs_driver/msg/point_cloud_msg.h```
- **API definition**: ```rs_driver/src/rs_driver/api/lidar_driver.h```
- **Error code definition**: ```rs_driver/src/rs_driver/common/error_code.h```

