# Api Introduction

### **namespace**

  ```C++
      robosene::lidar
  ```

### **constructor**

  ```C++
      LidarDriver() 
  ```

### **member functions**

```c++
      /**
       * @description: The initialize function, used to set the realated parameters and instance objects, 
       *               used when get packets from online lidar or pcap.
       * @param The struct->RSDriverParam 
       * @return: bool
       */
      inline bool init(const RSDriverParam &param);
```

```c++
      /**
       * @description: The initialize function, only initilize decoder(not include input module), only be used when not get packets from ROS or other ways excluding online lidar and pcap.
       * @param The struct->RSDriverParam 
       */
      inline void initDecoderOnly(const RSDriverParam &param);
```



```c++
      /**
       * @description: Start the thread to receive packets, and decode packets
       * @return: bool
       */
      inline bool start();
```

```c++
      /**
       * @description: Stop all threads
       */
      inline void stop();
```

```c++
      /**
       * @description: Register the lidar point cloud callback function.
       *  When pointcloud is prepared, this function will be called.
       * @param callBack the callback funtion  
       */
      inline void regRecvCallback(const std::function<void(const PointcloudMsg<PointT> &)> callBack);
```

```c++
      /**
       * @description: Register the lidar scan message callback funtion. 
       * When lidar scan message is ready, this function will be called.
       * @param callBack the callback funtion  
       */
      inline void regRecvCallback(const std::function<void(const ScanMsg &)> callBack);
```

```c++
      /**
       * @description: Register the lidar difop packet message callback funtion. 
       * When lidar difop packet message is ready, this function will be called.
       * @param callBack the callback funtion  
       */
      inline void regRecvCallback(const std::function<void(const PacketMsg &)> callBack);
```

```c++
      /**
       * @description: Register the exception message callback funtion. 
       * When error occurs, this function will be called.
       * @param excallBack The callback funtion  
       */
      inline void regExceptionCallback(const std::function<void(const Error &)> excallBack);
```

```c++
      /**
       * @description: Decode the scan message. Can be called when processing offline lidar message.
       **NOTE** This function will only work after decodeDifopPkt is called unless wait_for_difop is set to false,
                because the driver need difop packet to help to decode scan message.
       * @param pkt_scan_msg The lidar scan message used to be decode
       * @param point_msg The output point cloud message
       * @return bool
       */
      inline bool decodeMsopScan(const ScanMsg &pkt_scan_msg, PointcloudMsg<PointT> &point_msg);
```

```c++
      /**
       * @description: Decode the lidar difop message. **Must** be called when processing offline lidar message.
       * @param pkt_msg The lidar difop packet 
       */
      inline void decodeDifopPkt(const PacketMsg &pkt_msg);
```

