# **rs_driver** 

[English Version](README.md) 

## 1 简介

**rs_driver**为RoboSense的雷达驱动。

## 2 支持的雷达型号

支持的雷达型号如下。
- RS-LiDAR-16
- RS-LiDAR-32
- RS-Bpearl
- RS-Helios
- RS-Ruby
- RS-Ruby Lite
- RS-LiDAR-M1

## 3 支持的操作系统

支持的操作系统及编译器如下。
- Ubuntu (16.04, 18.04, 20.04)
  - gcc (4.8+)

- Windows
  - MSVC  (VS2017 & VS2019 已测试)
  - Mingw-w64 (x86_64-8.1.0-posix-seh-rt_v6-rev0 已测试)

## 4 依赖的第三方库

依赖的第三方库如下。
- libpcap (可选。如不需要解析PCAP文件，可忽略)
- PCL (可选。如不需要可视化工具，可忽略)
- Eigen3 (可选。如不需要内置坐标变换，可忽略)

## 5 编译与安装
### 5.1 Ubuntu下的编译与安装
#### 5.1.1 安装第三方库

```bash
sudo apt-get install libpcap-dev libpcl-dev libeigen3-dev
```
#### 5.1.2 编译

```bash
cd rs_driver
mkdir build && cd build
cmake .. && make -j4
```

#### 5.1.3 安装

```bash
sudo make install
```

#### 5.1.4 作为第三方库使用

配置您的```CMakeLists```文件，使用find_package()指令找到**rs_driver**库，并链接。

```cmake
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${rs_driver_LIBRARIES})
```

#### 5.1.5 作为子模块使用

将**rs_driver**作为子模块添加到您的工程，相应配置您的```CMakeLists```文件。

使用find_package()指令找到**rs_driver**库，并链接。

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(project ${rs_driver_LIBRARIES})
```

### 5.2 Windows下的编译与安装

#### 5.2.1 安装第三方库

##### libpcap

  安装[libpcap运行库](https://www.winpcap.org/install/bin/WinPcap_4_1_3.exe)。

  下载[libpcap开发者包](https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip)到任意位置，并将```WpdPack_4_1_2/WpdPack``` 的路径添加到环境变量```PATH```。

##### PCL

+ MSVC

​     如果使用MSVC编译器，可使用PCL官方提供的[PCL安装包](https://github.com/PointCloudLibrary/pcl/releases)安装。

​     安装过程中选择 “Add PCL to the system PATH for xxx”:

![](./doc/img/01_install_pcl.PNG)

+ Mingw-w64

​     PCL官方没有提供mingw编译的库，所以需要按照[PCL官方教程](https://pointclouds.org/documentation/tutorials/compiling_pcl_windows.html), 从源码编译PCL并安装。

#### 5.2.2 安装

Windows下，**rs_driver** 暂不支持安装。

## 6 快速上手

**rs_driver**在目录```rs_driver/demo``` 下，提供了两个使用示例程序。

- demo_online.cpp
- demo_pcap.cpp

`demo_online`解析在线雷达的数据，输出点云， `demo_pcap`解析PCAP文件，输出点云。

`demo_pcap`基于libpcap库。

要编译这两个程序，需使能COMPILE_DEMOS选项。

```bash
cmake -DCOMPILE_DEMOS=ON ..
```

关于`demo_online`的更多说明，可以参考[在线连接雷达](doc/howto/how_to_online_use_driver.md)

关于`demo_pcap`的更多说明，可以参考[解析pcap包](doc/howto/how_to_decode_pcap.md)

## 7 可视化工具

**rs_driver**在目录```rs_driver/tool``` 下，提供了一个点云可视化工具`rs_driver_viewer`。它基于PCL库。

要编译这个工具，需使能COMPILE_TOOS选项。

```bash
cmake -DCOMPILE_TOOLS=ON ..
```

关于`rs_driver_viewer`的使用方法，请参考[可视化工具操作指南](doc/howto/how_to_use_rs_driver_viewer.md) 

## 8 更多主题

关于**rs_driver**的其他主题，请参考如下链接。

组播模式:  [组播模式](doc/howto/how_to_use_multi_cast_function.md) 
坐标变换：[坐标变换](doc/howto/how_to_transform_pointcloud.md) 

**rs_driver**的主要接口文件如下。

- 点云消息定义: ```rs_driver/src/rs_driver/msg/point_cloud_msg.h```
- 接口定义: ```rs_driver/src/rs_driver/api/lidar_driver.h```
- 参数定义: ```rs_driver/src/rs_driver/driver/driver_param.h```
- 错误码定义: ```rs_driver/src/rs_driver/common/error_code.h```

