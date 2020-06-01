# Interface Introduction

### **namespace**

  ```C++
      robosene::lidar
  ```

### **constructor**

  ```C++
      LidarDriverInterface() 
  ```

### **member functions**

```c++
      /**
       * @description: The initialize function, used to set the realated parameters and instance objects, used when get packets from online lidar or pcap.
       * @param The struct->RSLiDAR_Driver_Param 
       * @return: Null
       */
      inline void init(const RSLiDAR_Driver_Param &param)
      {
        driver_ptr_->init(param);
      }
```

```c++
      /**
       * @description: The initialize function, only initilize decoder(not include input module), only be used when not get packets from ROS or other ways excluding online lidar and pcap.
       * @param The struct->RSLiDAR_Driver_Param 
       * @return: Null
       */
      inline void initDecoderOnly(const RSLiDAR_Driver_Param &param)
      {
        driver_ptr_->initDecoderOnly(param);
      }
```



```c++
      /**
       * @description: Used to start the thread to receive packets, and decode packets
       * @param Null
       * @return: Null
       */
      inline void start()
      {
        driver_ptr_->start();
      }
```

```c++
      /**
       * @description: Used to stop all threads
       * @param Null 
       * @return: Null
       */
      inline void stop()
      {
        driver_ptr_->stop();
      }
```

```c++
      /**
       * @description: Used to register the lidar point cloud callback function.
       *  When pointcloud is prepared, this function will be called.
       * @param callBack the callback funtion  
       * @return: Null
       */
      inline void regPointRecvCallback(const std::function<void(const PointcloudMsg<PointT> &)> callBack)
      {
        driver_ptr_->regPointRecvCallback(callBack);
      }
```

```c++
      /**
       * @description: Used to register the lidar scan message callback funtion. 
       * When lidar scan message is ready, this function will be called.
       * @param callBack the callback funtion  
       * @return: Null
       */
      inline void regRecvCallback(const std::function<void(const ScanMsg &)> callBack)
      {
        driver_ptr_->regRecvCallback(callBack);
      }
```

```c++
      /**
       * @description: Used to register the lidar difop packet message callback funtion. 
       * When lidar difop packet message is ready, this function will be called.
       * @param callBack the callback funtion  
       * @return: Null
       */
      inline void regRecvCallback(const std::function<void(const PacketMsg &)> callBack)
      {
        driver_ptr_->regRecvCallback(callBack);
      }
```

```c++
      /**
       * @description: Used to register the exception message callback funtion. 
       * When error occurs, this function will be called.
       * @param excallBack The callback funtion  
       * @return: Null
       */
      inline void regExceptionCallback(const std::function<void(const Error &)> excallBack)
      {
        driver_ptr_->regExceptionCallback(excallBack);
      }
```

```c++
      /**
       * @description: Used to decode the scan message. Can be called when processing offline lidar message.
       **NOTE** This function will only work after decodeDifopPkt is called,
                because the driver need difop packet to help to decode scan message.
       * @param pkt_scan_msg The lidar scan message used to be decode
       * @param point_msg The output point cloud message
       * @return: Null
       */
      inline void decodeMsopScan(const ScanMsg &pkt_scan_msg, PointcloudMsg<PointT> &point_msg)
      {
        driver_ptr_->decodeMsopScan(pkt_scan_msg, point_msg);
      }
```

```c++
      /**
       * @description: Used to decode the lidar difop message. **Must** be called when processing offline lidar message.
       * @param pkt_msg The lidar difop packet 
       * @return: Null
       */
      inline void decodeDifopPkt(const PacketMsg &pkt_msg)
      {
        driver_ptr_->decodeDifopPkt(pkt_msg);
      }
```

