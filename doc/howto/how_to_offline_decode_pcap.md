# How to offline decode pcap bag



## 1 Introduction

This document will show you how to use the API to decode pcap bag.



## 2 Steps

Please follow the steps below to do advanced development.



#### 2.1 Define a point type

Now the driver will automatically detect and assign value to the following six variables.

- x ------ The x coordinate of point.
- y ------ The y coordinate of point.
- z ------ The z coordinate of point.
- intensity ------ The intensity of point.
- timestamp ------ The timestamp of point (If use_lidar_clock is set to true, this timestamp will be lidar time, otherwise will be system time).
- ring ------ The ring ID of the point, which represents the row number. e.g. For RS80, the range of ring ID is 0~79 (from bottom to top).



Here are some examples: 

- The point type contains **x, y, z** 

  ```c++
  struct PointXYZ ///< user defined point type
  {
    float x;
    float y;
    float z;
    ...
  };
  ```

- The point type contains **x, y, z, intensity**

  ```c++
  struct PointXYZI ///< user defined point type
  {
    float x;
    float y;
    float z;
    float intensity;
    ...
  };
  ```

  Or you can use **pcl::PointXYZI** directly

- The point type contains **x, y, z, intensity, timestamp, ring**

  ```c++
  struct PointXYZIRT ///< user defined point type
  {
    float x;
    float y;
    float z;
    float intensity;
    float timestamp;
    uint16_t ring;
    ...
  };
  ```

The only thing user need to pay attention to is that the variable name must obey the rules -- **x, y, z, intensity, timestamp, ring**, otherwise the variable can not be assigned as expected.



#### 2.2 Define a point cloud callback function

Define the point cloud callback function. The template argument PointXYZI is the point type we defined in 2.1. When point cloud message is ready, this function will be called by driver. **Note! Please don't add any time-consuming operations in this function!** User can make a copy of the message and process it in another thread.  Or user can add some quick operations such like ros publish in the callback function.

```c++
void pointCloudCallback(const PointCloudMsg<PointXYZI> &msg)
{
  MSG << "msg: " << msg.seq << " point cloud size: " << msg.point_cloud_ptr->size() << REND;
}
```

#### 2.3 Define a exception callback function

Define the exception callback function. When driver want to send out infos or error codes, this function will be called. Same as the previous callback function, please **do not add any time-consuming operations in this callback function!**

```c++
void exceptionCallback(const Error &code)
{
  WARNING << "Error code : " << code.toString() << REND;
}
```

#### 2.4 Instantiate the driver object

Instanciate the driver object.

```c++
LidarDriver<PointXYZI> driver;          ///< Declare the driver object
```

#### 2.5 Define the parameter, configure the parameter

Define a parameter object and config it. Since we want to decode pcap bag, please set the read_pcap to true and set up the correct pcap file directory. The ip address, msop port and difop port number of lidar can be got from **wireshark(a network socket capture software)**. The default value is ip: 192.168.1.200, msop: 6699, difop: 7788. User also need to make sure the **lidar type** is set correctly.

```c++
RSDriverParam param;                                             ///< Create a parameter object
param.input_param.read_pcap = true;                              ///< Set read_pcap to true
param.input_param.pcap_path = "/home/robosense/rs16.pcap";  ///< Set the pcap file directory
param.input_param.device_ip = "192.168.1.200";  ///< Set the lidar ip address, the default is 192.168.1.200
param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct
```

#### 2.6 Register the point cloud callback and exception callback

Register the callback functions we defined in 2.2 and 2.3. **Note:The exception callback function must be registered before the init() function is called because  error may occur during the initialization**.

```c++
driver.regRecvCallback(pointCloudCallback); ///< Register the point cloud callback function into the driver
driver.regExceptionCallback(exceptionCallback);  ///<Register the exception callback function into the driver
```

#### 2.7 Call the driver initialization function

Call the initialization function and pass the parameter into the driver. Since we need to get packets from pcap bag, we call init() function instead of initDecoderOnly(). Remember to check whether the initialization is successful, if not, please check the error code, and modify parameters.

```c++
if (!driver.init(param))                         ///< Call the init function and pass the parameter
{
  ERROR << "Driver Initialize Error..." << REND;
  return -1;
}
```

#### 2.8 Start the driver

Call the start function to start the driver.

```c++
driver.start();                                  ///< Call the start function. The driver thread will start
```



## 3 Hint for compile

If you  choose to embed rs_driver into your project instead of installing it as mentioned in README, you need to setup the include_directories in CMakeLists

```cmake
include_directories(your_project_directory/rs_driver/src)
```

Since the driver core only has header files, user need to link the related libraries when compile. You need to link to boost, pcap & pthread. 

```cmake
find_package(Boost COMPONENTS system REQUIRED)
add_executable(demo
              demo/demo.cpp
              )
target_link_libraries(demo
                    ${Boost_LIBRARIES}       
                      pcap
                      pthread
)
```



### *Congratulations! You have finished the first simple demo tutorial of RoboSense lidar driver! You can find the complete demo code in the demo folder under the project directory. Feel free to connect us if you have any question about the driver.*