# Api Introduction

### **1 namespace**

  ```C++
robosene::lidar
  ```

### **2 constructor**

  ```C++
LidarDriver() 
  ```

### **3 member functions**

```c++
/**
 * @brief The initialize function, used to set the related parameters and instance objects, 
 *        used when get packets from online lidar or pcap 
 * @param param The custom struct RSDriverParam 
 * @return If success, return ture; else return false
 */
inline bool init(const RSDriverParam &param);
```

```c++
/**
 * @brief The initialize function which only initialize decoder(not include input module). If lidar packets are from ROS or other ways excluding online lidar and pcap, call this function to initialize instead of calling init()
 * @param param The custom struct RSDriverParam 
 */
inline void initDecoderOnly(const RSDriverParam &param);
```



```c++
/**
 * @brief Start the threads to receive packets and decode packets
 * @return If success, return ture; else return false
 */
inline bool start();
```

```c++
/**
 * @brief Stop all threads
 */
inline void stop();
```

```c++
/**
 * @brief Register the lidar point cloud callback function to driver. When point cloud is ready, this function will be called
 * @param callback The callback funtion  
 */
inline void regRecvCallback(const std::function<void(const PointCloudMsg<PointT> &)> callback);
```

```c++
/**
 * @brief Register the lidar scan message callback funtion to driver.When lidar scan message is ready, this function will be called
 * @param callback The callback funtion  
 */
inline void regRecvCallback(const std::function<void(const ScanMsg &)> callback);
```

```c++
/**
 * @brief Register the lidar difop packet message callback funtion to driver. When lidar difop packet message is ready, this function will be called
 * @param callback The callback funtion  
 */
inline void regRecvCallback(const std::function<void(const PacketMsg &)> callback);
```

```c++
/**
 * @brief Register the exception message callback funtion to driver. When error occurs, this function will be called
 * @param callback The callback funtion  
 */
inline void regExceptionCallback(const std::function<void(const Error &)> callback);
```

```c++
/**
 * @brief Decode the lidar scan message to point cloud
 * @note This function will only work after decodeDifopPkt is called unless wait_for_difop is set to false
 * @param pkt_scan_msg The lidar scan message
 * @param point_cloud_msg The output point cloud message
 * @return if decode success, return true; else return false
 */
inline bool decodeMsopScan(const ScanMsg &pkt_scan_msg, PointCloudMsg<PointT> &point_cloud_msg);
```

```c++
/**
 * @brief Decode the lidar difop message
 * @param packet_msg The lidar difop packet 
 */
inline void decodeDifopPkt(const PacketMsg &packet_msg);
```

