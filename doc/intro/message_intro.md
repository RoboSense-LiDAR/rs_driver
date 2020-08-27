# Message Introcution

#### 1 PacketMsg
```c++
struct alignas(16) PacketMsg ///< LiDAR single packet message
{
  double timestamp = 0.0;	///< Time stamp
  std::string frame_id = ""; ///< Frame ID
  std::array<uint8_t, RSLIDAR_PKT_LEN> packet{}; ///< Data
};
```



#### 2 LidarScanMsg

```c++
struct alignas(16) ScanMsg
{
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string frame_id = "";
  std::vector<PacketMsg> packets; ///< A vector which store a scan of packets (the size of the vector is not fix)
};
```




#### 3 LidarPointCloudMsg
```c++
struct alignas(16) PointCloudMsg
{
  typedef std::vector<PointT> PointCloud;
  typedef std::shared_ptr<PointCloud> PointCloudPtr;
  typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string frame_id = "";        ///< the point cloud frame id
  uint32_t height = 0;              ///< the height of point cloud
  uint32_t width = 0;               ///< the width of point cloud
  bool is_dense = false;            ///< if is_dense=true, the point cloud does not contain NAN points
  PointCloudPtr point_cloud_ptr;     ///< the point cloud pointer
  PointCloudMsg() = default;
  PointCloudMsg(const PointCloudPtr &ptr) : point_cloud_ptr(ptr)
  {
  }
  typedef std::shared_ptr<PointCloudMsg> Ptr;
  typedef std::shared_ptr<const PointCloudMsg> ConstPtr;
};
```



