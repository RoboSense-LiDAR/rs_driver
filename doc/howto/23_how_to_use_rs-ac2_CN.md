#  23 **如何使用RS-AC2**

## 23.1 概述

本文描述如何连接RS-AC2，得到点云,图像，以及IMU数据。

这个例子连接单个RS-AC2，完整的代码可以参考示例程序`rs_driver/demo/demo_usb.cpp`。

这里的点和点云，图像，IMU定义均引用了`rs_driver`工程的消息文件。

 ```rs_driver/src/rs_driver/msg/point_cloud_msg.hpp```, 
 ```rs_driver/src/rs_driver/msg/pcl_point_cloud_msg.hpp```,
 ```rs_driver/src/rs_driver/msg/image_msg.hpp```,
 ```rs_driver/src/rs_driver/msg/imu_data_msg.hpp```


## 23.2 步骤

### 23.2.1 定义点

点是点云的基本组成单元。`rs_driver`支持点的如下成员。
- x -- 坐标X，类型float
- y -- 坐标Y，类型float
- z -- 坐标Y，类型float
- intensity -- 反射率，类型uint8_t
- timestamp -- 时间戳，类型double
- ring -- 通道编号。。

如下是几个例子。

- 点的成员包括 **x, y, z, intensity**

  ```c++
  struct PointXYZI
  {
    float x;
    float y;
    float z;
    uint8_t intensity;
  };
  ```
  
- 如果使用PCL库，也可以简单使用PCL的点定义 **pcl::PointXYZI**

  ```c++
  typedef pcl::PointXYZI PointXYZI; 
  ```

- 点的成员包括 **x, y, z, intensity, timestamp, ring**

  ```c++
  struct PointXYZIRT
  {
    float x;
    float y;
    float z;
    uint8_t intensity;
    double timestamp;
    uint16_t ring;
  };
  ```

这些点的定义，使用者可以加入新成员，删除成员，改变成员顺序，但是不可以改变成员的类型。

### 23.2.2 定义点云

如下是点云的定义。

  ```c++
  template <typename T_Point>
  class PointCloudT
  {
  public:
    typedef T_Point PointT;
    typedef std::vector<PointT> VectorT;

    uint32_t height = 0;    ///< Height of point cloud
    uint32_t width = 0;     ///< Width of point cloud
    bool is_dense = false;  ///< If is_dense is true, the point cloud does not contain NAN points
    double timestamp = 0.0; ///< Timestamp of point cloud
    uint32_t seq = 0;       ///< Sequence number of message

    VectorT points;
  };
  
  ```
这个点云的定义，使用者可以加入新成员，改变成员顺序，但是不可以删除成员，或者改变成员的类型。

这个点云定义是一个模板类，还需要指定一个点类型作为模板参数。

  ```c++
  typedef PointXYZI PointT;
  typedef PointCloudT<PointT> PointCloudMsg;
  ```

### 23.2.3 定义LidarDriver对象

LidarDriver类是`rs_driver`的接口类。这里定义一个LidarDriver的实例。

```c++
int main()
{
  LidarDriver<PointCloudMsg> driver;          ///< Declare the driver object
  ...
}
```

### 23.2.4 配置LidarDriver的参数

RSDriverParam定义LidarDriver的参数。这里定义RSDriverParam变量，并配置它。

+ `InputType::USB`意味着设备使用USB进行连接。
+ `LidarType::RS_AC2`是雷达类型

```c++
int main()
{
  ...
  RSDriverParam param;                            ///< Create a parameter object
  param.input_type = InputType::USB;              ///< Get packet from online lidar
  param.lidar_type = LidarType::RS_AC2;           ///< Set the lidar type. Ensure the type is correct

  // param.input_param.device_uuid = "0x12345678";   ///< Set the device UUID
  param.decoder_param.ts_first_point = true;      ///< Use the timestamp of the first point
  param.input_param.image_width = 1616;  ///< Set the image width. Make sure this value is correct
  param.input_param.image_height = 2636;  ///< Set the image height. Make sure this value is correct
  param.input_param.image_format = FRAME_FORMAT_XR24;  ///< Set the image format. Make sure this value is correct
  param.input_param.image_fps = 15;  ///< Set the image fps. Make sure this value is correct
  ...
}
```

### 23.2.5 定义和注册点云回调函数

+ `rs_driver`需要调用者通过回调函数，提供空闲的点云实例。这里定义这第一个点云回调函数。

```c++
SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;

std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}
```

+ `rs_driver`通过回调函数，将填充好的点云返回给调用者。这里定义这第二个点云回调函数。

```c++
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);

  RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;
}
```

注意这两个回调函数都运行在`rs_driver`的Points Packet处理线程中，所以它们不可以做太耗时的任务，否则会导致PointsPacket不能及时处理。。

+ 使用者在自己的线程中，处理点云。

```c++
void processCloud(void)
{
  while (1)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the point cloud msg, even it is time-consuming.
    RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

    free_cloud_queue.push(msg);
  }
}

```

+ 注册两个点云回调函数。

```c++
int main()
{
  ...
  driver.regPointCloudCallback(driverReturnPointCloudToCallerCallback, 
                               driverReturnPointCloudToCallerCallback);
  ...
}
```

### 23.2.6 定义和注册IMU回调函数

+ 和获取点云类似， `rs_driver`需要调用者通过回调函数，提供空闲的IMU实例。这里定义这第一个IMU回调函数。

```c++
SyncQueue<std::shared_ptr<ImuData>> free_imu_data_queue;

std::shared_ptr<ImuData> driverGetIMUDataFromCallerCallback(void)
{
  std::shared_ptr<ImuData> msg = free_imu_data_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<ImuData>();
}

```

+ `rs_driver`通过回调函数，将填充好的IMU数据返回给调用者。这里定义这第二个IMU回调函数。

```c++
SyncQueue<std::shared_ptr<ImuData>> stuffed_imu_data_queue;

void driverReturnImuDataToCallerCallback(const std::shared_ptr<ImuData>& msg)
{
  stuffed_imu_data_queue.push(msg);
}

```

注意这两个回调函数都运行在`rs_driver`的IMU Packet的处理线程中，所以它们不可以做太耗时的任务，否则会导致IMU Packet不能及时处理。

+ 使用者在自己的线程中，处理IMU数据。

```c++
void processImuData(void)
{
  uint32_t imu_cnt = 0;
  while (!to_exit_process)
  {
    std::shared_ptr<ImuData> msg = stuffed_imu_data_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the IMU data msg, even it is time-consuming.
    RS_MSG << "msg: " << imu_cnt << " imu data ts: " <<std::dec<<std::to_string(msg->timestamp) << RS_REND;

    imu_cnt++;
    free_imu_data_queue.push(msg);
  }
}

```

+ 注册两个IMU回调函数。

```c++
int main()
{
  ...
#if ENABLE_IMU_PARSE
  driver.regImuDataCallback(driverGetIMUDataFromCallerCallback, driverReturnImuDataToCallerCallback);
#endif
  ...
}
```

### 23.2.7 定义和注册图像回调函数

+ 和获取点云类似， `rs_driver`需要调用者通过回调函数，提供空闲的Image实例。这里定义这第一个Image回调函数，根据`lidar_type`来创建单目图像还是双目图像。

```c++
SyncQueue<std::shared_ptr<ImageMsg>> free_image_data_queue;;

std::function<std::shared_ptr<ImageMsg>()> createImageDataCallback(LidarType lidar_type)
{
  return [lidar_type]() -> std::shared_ptr<ImageMsg> {
    std::shared_ptr<ImageMsg> msg = free_image_data_queue.pop();
    if (msg)
    {
      return msg;
    }

    switch (lidar_type)
    {
      case LidarType::RS_AC1:
        return std::make_shared<MonoImageMsg>();
      default:
        return std::make_shared<StereoImageMsg>();
    }
  };
}

```

+ `rs_driver`通过回调函数，将填充好的Image数据返回给调用者。这里定义这第二个Image回调函数。

```c++
SyncQueue<std::shared_ptr<ImageMsg>> stuffed_image_data_queue;

void driverReturnImageDataToCallerCallback(const std::shared_ptr<ImageMsg>& msg)
{
  stuffed_image_data_queue.push(msg);
}
```

注意这两个回调函数都运行在`rs_driver`的Image Packet的处理线程中，所以它们不可以做太耗时的任务，否则会导致Image Packet不能及时处理。

+ 使用者在自己的线程中，处理Image数据，根据`camera_mode`判断是单目图像还是双目图像，分别处理。

```c++
void processImageData(void)
{
  uint32_t image_cnt = 0;
  while (!to_exit_process)
  {
    std::shared_ptr<ImageMsg> msg = stuffed_image_data_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the Image data msg, even it is time-consuming.

    image_cnt++;
    if (msg->camera_mode == CameraMode::STEREO)
    {
      auto stereo = std::dynamic_pointer_cast<StereoImageMsg>(msg);
      if (stereo->left_data != NULL)
      {
      }
      if (stereo->right_data != NULL)
      {
      }
    }
    else
    {
      auto mono = std::dynamic_pointer_cast<MonoImageMsg>(msg);
    }

#if 1
    RS_MSG << "msg: " << image_cnt << " image data ts: " <<std::dec<<std::to_string(msg->timestamp) << RS_REND;
#endif

    free_image_data_queue.push(msg);
  }
}
```

+ 注册两个Image回调函数。

```c++
int main()
{
  ...
#if ENABLE_IMAGE_PARSE
    driver.regImageDataCallback(createImageDataCallback(param.lidar_type), driverReturnImageDataToCallerCallback);
#endif
  ...
}
```



### 23.2.8 定义和注册异常回调函数

+ `rs_driver`检测到异常发生时，通过回调函数通知调用者。这里定义异常回调函数。

```c++
void exceptionCallback(const Error &code)
{
  RS_WARNING << "Error code : " << code.toString() << RS_REND;
}
```

再一次提醒，这个回调函数运行在`rs_driver`的线程中，所以不可以做太耗时的任务，否则可能导致MSOP/DIFOP包不能及时接收和处理。

+ 在主函数中，注册异常回调函数。

```c++
int main()
{
  ...
  driver.regExceptionCallback(exceptionCallback);  ///<Register the exception callback function
  ...
}
```

### 23.2.9 初始化LidarDriver对象

按照RSDriverParam指定的配置，初始化LidarDriver对象。

```c++
int main()
{
  ...
  if (!driver.init(param))  ///< Call the init function with the parameter
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }
  ...
}
```

### 23.2.10 启动LidarDriver

启动LidarDriver对象。

```c++
int main()
{
  ...
  driver.start();  ///< Call the start function. The driver thread will start
  ...
}
```

### 23.2.11 祝贺

编译`demo_usb`并运行。您应该可以看到类似如下点云打印了。

```c++
RoboSense Lidar-Driver Linux usb demo start......
msg: 1 imu data ts: 1741941730.782551
msg: 2 imu data ts: 1741941730.787492
msg: 3 imu data ts: 1741941730.792433
msg: 4 imu data ts: 1741941730.797375
msg: 5 imu data ts: 1741941730.802217
msg: 6 imu data ts: 1741941730.807259
msg: 7 imu data ts: 1741941730.812200
msg: 8 imu data ts: 1741941730.817142
msg: 9 imu data ts: 1741941730.822084
msg: 10 imu data ts: 1741941730.827025
msg: 11 imu data ts: 1741941730.831972
msg: 12 imu data ts: 1741941730.836911
msg: 13 imu data ts: 1741941730.841850
msg: 1 image data ts: 8.599935
msg: 14 imu data ts: 1741941730.846792
msg: 15 imu data ts: 1741941730.851734
msg: 16 imu data ts: 1741941730.856688
msg: 1 point cloud size: 27648
msg: 17 imu data ts: 1741941730.861617
msg: 2 image data ts: 8.599935
msg: 18 imu data ts: 1741941730.866559
msg: 19 imu data ts: 1741941730.871505
msg: 20 imu data ts: 1741941730.876460
msg: 3 image data ts: 8.666600
msg: 21 imu data ts: 1741941730.881384
msg: 22 imu data ts: 1741941730.886326
msg: 23 imu data ts: 1741941730.891266
```

