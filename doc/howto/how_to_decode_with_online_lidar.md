# How to use driver with online Lidar

## 1 Introduction

This document illustrates how to decode the packets from an online lidar with the driver's API.

## 2 Steps

### 2.1 Define Point

Now the driver will automatically assign values these member variables.

- x ------ float type. The x coordinate of point.
- y ------ float type. The y coordinate of point.
- z ------ float type. The z coordinate of point.
- intensity ------ uint8_t type. The intensity of point.
- timestamp ------ double type. The timestamp of point. If ```use_lidar_clock``` is ```true```, lidar clock time in MSOP packet is used, otherwise regenerate the timestamp on host.
- ring ------ The ring ID of the point, which represents the row number. e.g. For RS80, the range of ring ID is 0~79 (from bottom to top).

Below are some examples: 

- The point type contains **x, y, z, intensity**

  ```c++
  struct PointXYZI ///< user defined point type
  {
    float x;
    float y;
    float z;
    uint8_t intensity;
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
    uint8_t intensity;
    double timestamp;
    uint16_t ring;
    ...
  };
  ```

User may add new member variables, remove member variables, or change the order of them, but should not change names or types of them -- **x, y, z, intensity, timestamp, ring**, otherwise the behaviors might be unexpected.

### 2.2 Define Point Cloud

  The driver keep a point cloud instance internally, and accumulates points into it. When a frame of point cloud is ready, it informs the caller.
  
  The point cloud is like below:
  
  ```c++
  template <typename T_Point>
  class PointCloudT
  {
  public:
    typedef T_Point PointT;
    typedef std::vector<PointT> VectorT;

    uint32_t height = 0;    ///< Height of point cloud
    uint32_t width = 0;     ///< Width of point cloud
    bool is_dense = false;  ///< If is_dense is true, the point cloud does not contain NAN points,
    double timestamp = 0.0;
    uint32_t seq = 0;           ///< Sequence number of message

    VectorT points;
  };
  
  typedef PointXYZI PointT;
  typedef PointCloudT<PointT> PointCloudMsg;

  ```

  User may add new members to PointCloudT, change the order of its members, but should not remove member variables.

  The template argument of PointCloudT is the point type, which is defined in the section 2.1. 

### 2.3 Define point cloud callback functions

The user is supposed to managed point cloud pools. The driver gets free point cloud from user, and returns prepared point clouds to user.

User provides two callback functions. The driver call one of them to get a free point cloud, before constructing it.

```c++
std::shared_ptr<PointCloudMsg> pointCloudGetCallback(void)
{
  return std::make_shared<PointCloudMsg>();
}
```

The driver call the other to return the prepared point cloud.

```c++
void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
  RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;
}
```

Note: 

+ Above examples always allocates a new point cloud instance for simplicity. This is not suggested, especially if the efficiency is sensitive.

+ The callback functions run in the main handling loop of the driver, so **Do not add any time-consuming operations in the callback functions**. 

### 2.4 Define exception callback functions

When an error is reported, the driver will inform user. User should provide an exception callback function. 

Once again, **Do not add any time-consuming operations in the callback functions**.

```c++
void exceptionCallback(const Error &code)
{
  RS_WARNING << "Error code : " << code.toString() << RS_REND;
}
```

### 2.5 Instantiate the driver object

Instantiate a driver object.

```c++
LidarDriver<PointCloudMsg> driver;          ///< Declare the driver object
```

### 2.6 Configure the parameter

Define a parameter object and configure it. 

To decode pcap bag, please set the ```input_type``` to ```InputType::ONLINE_LIDAR```. 

Read the pcap file with a 3rd party tool, such as Wireshark, to get the MSOP/DIFOP ports. Also give the correct lidar type.

```c++
RSDriverParam param;                             ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;      /// get packet from online lidar
param.input_param.pcap_path = "/home/robosense/rs16.pcap";  ///< Set the pcap file path
param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct
```

### 2.7 Register callback functions

Register above callback functions. 

Please **register the callback functions before call the init() function**, because errors may occur during the initialization.

```c++
driver.regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback); ///< Register the point cloud callback functions
driver.regExceptionCallback(exceptionCallback);  ///<Register the exception callback function
```

### 2.8 Call the driver initialization function

Call the initialization function with the the parameter. 

```c++
if (!driver.init(param))                         ///< Call the init function and pass the parameter
{
  RS_ERROR << "Driver Initialize Error..." << RS_REND;
  return -1;
}
```

### 2.9 Start the driver

Start the driver.

```c++
driver.start();                                  ///< Call the start function. The driver thread will start
```



## 3 About Point cloud storage order

The prepared point cloud stores the points in **column major order**. In other words, if a point is msg.point_cloud_ptr->at(i) , the next point on the same ring will be msg.point_cloud_ptr->at(i+msg.height). 

## 4 Congratulations

**You have finished the demo tutorial of the RoboSense LiDAR driver! Please find the complete demo code in the project directory. Feel free to contact us if you have any questions.**

### *Congratulations! You have finished the demo tutorial of RoboSense LiDAR driver! You can find the complete demo code in the demo folder under the project directory. Feel free to connect us if you have any question about the driver.*
