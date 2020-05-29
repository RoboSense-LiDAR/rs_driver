
### 1. 工程简介
  **rs_lidar_driver**为速腾聚创雷达驱动软件。该软件支持**RS-LiDAR-16**、**RS-LiDAR-32**、**RS-Bpearl**和**RS-Ruby**的点云数据解析，同时提供了demo方便第三方开发人员的使用；

### 2. 依赖介绍

### 3. 安装方式

```sh
    git clone http://github.com/
    cd rs_lidar_driver
    mkdir build
    cmake .. && make -j4
    sudo make install
```

### 4. 参数简介
+ LiDAR_TYPE为支持的雷达型号列表:
  + RS16代表速腾聚创RS-LiDAR-16;
  + RS32代表速腾聚创RS-LiDAR-32;
  + RSBP代表速腾聚创RS-Bpearl;
  + RS128代表速腾聚创RS-Ruby;

+ RS_ECHO_MODE为雷达支持的回波模式列表，需要使用RSView修改:
  + RS_ECHO_DUAL表示双回波模式；
  + RS_ECHO_MAX表示最大回波模式；
  + RS_ECHO_LAST表示最后回波模式；

+ RSDecoder_Param为雷达解析配置参数:
  + echo_mode用于设置雷达的回波模式；
  + max_distance用于设置雷达的最远有效距离；
  + min_distance用于设置雷达的最短有效距离；
  + start_angle用于设置雷达一帧数据的起始角度；
  + end_angle用于设置雷达一帧数据的结束角度；
  + mode_split_frame
  + num_pkts_split
  + cut_angle表示雷达的分割角度；

+ RSInput_Param为雷达解析输入参数:
  + device_ip用于设置雷达的IP；
  + msop_port用于设置雷达的msop端口；
  + difop_port用于设置雷达的difop端口；
  + read_pcap用于设置是否读取pcap；
  + pcap_repeat用于设置是否重复读取pcap；
  + pcap_file_dir用于设置pcap数据的路径；

+ RSLiDAR_Driver_Param为雷达驱动配置参数：
  + calib_path用于设置雷达的标定参数路径；
  + frame_id用于设置雷达的frame_id；
  + lidar_type用于设置雷达的类型，包括RS16，RS32,RSBP,RS128；
  + use_lidar_clock用于设置雷达消息时间戳类型，true:使用lidar时间戳,false:使用系统时间戳；
  + input_param用于设置雷达解析输入参数；
  + decoder_param用于设置雷达解析配置参数；

### 5. 参数简介
+ LidarPacketMsg单个packet消息:
    + timestamp用于存储消息的时间戳；
    + frame_id用于存储消息的frame_id；
    + packet用于存储一个packet的雷达数据；

+ LidarScanMsg单帧雷达消息:
    + timestamp用于存储消息的时间戳；
    + seq用于存储消息的序列号；
    + parent_frame_id用于存储消息的parent_frame_id；
    + frame_id用于存储消息的fram_id；
    + packets用于存储一帧雷达消息的所有packet数据；

+ LidarPointcloudMsg单帧雷达点云消息:
    +  timestamp用于存储消息的时间戳；
    +  seq由于存储消息的序列号；
    +  parent_frame_id用于存储消息的parent_frame_id;
    +  frame_id用于存储消息的frame_id;
    +  height用于存储点云的高度；
    +  width用于存储点云的宽度；
    +  is_dense用于存储点云是否为稠密点云；
    +  is_transform用于存储点云是否经过变换；
    +  is_motion_correct用于存储点云是否经过运动补偿；
    +  pointcloud_ptr用于存储点云消息；

### 6. Error code简介

+   ErrCode_Success: 正常
+   ErrCode_PcapFinished: pcap 文件解析完成；
+   ErrCode_PcapRepeat: pcap将会重复播放；
+   ErrCode_PcapExit: 将会退出pcap线程；
+   ErrCode_MsopPktTimeout: msop消息接收超时(1秒)；
+   ErrCode_DifopPktTimeout: difop消息接收超时(2秒)；
+   ErrCode_MsopPktIncomplete: msop消息接收未完成；
+   ErrCode_DifopPktIncomplete: difop消息接收未完成；
+   ErrCode_NoDifopRecv: 点云解析未开始直到收到difop消息；
+   ErrCode_ZeroPoints: 点云的大小为0，请检查雷达参数是否配置正确；
+   ErrCode_PcapWrongDirectory: pcap文件路径错误；
+   ErrCode_PcapContradiction: pcap功能被禁止，但是仍然尝试解析pcap文件；
+   ErrCode_MsopPortBuzy: msop端口被占用；
+   ErrCode_DifopPortBuzy: difop端口被占用；

### 7. 接口文档
  + 名称空间<br/>
  ```C++
      robosene::lidar
  ```

  + 构造函数<br/>
  ```C++
      LidarDriverInterface() #默认构造函数
  ```

  + 成员函数（接口）

      + inline void init(const RSLiDAR_Driver_Param &param)
      > **功能:**用于初始化雷达驱动<br/>
      > **参数:**RSLiDAR_Driver_Param，雷达驱动参数<br/>
      > **返回值:**void<br/>
      
      + inline void start()
      > ****功能:**用于启动雷达驱动<br/>
      > **参数:**无<br/>
      > **返回值:**void<br/>

      + inline void stop()<br/>
      > **功能:**用于停止雷达驱动<br/>
      > **参数:**无<br/>
      > **返回值:**void<br/>

      + inline void regPointRecvCallback(const std::function<void(const LidarPointcloudMsg<PointT> &)> callBack)
      > **功能:**用于注册单帧雷达点云回调函数，当雷达收到一帧LidarPointsMsg时，调用回调函数<br/>
      > **参数:*LidarPointsMsg回调函数*<br/>
      > **返回值:**void<br/>

      + inline void regRecvCallback(const std::function<void(const LidarScanMsg &)> callBack)<br/>
      > **功能:**用于注册msop消息回调函数，当雷达驱动收到一帧雷达LidarScanMsg消息时，调用回调函数
      > **参数:**LidarScanMsg回调函数<br/>
      > **返回值:**void<br/>

      + inline void regRecvCallback(const std::function<void(const LidarPacketMsg &)> callBack)
      > **功能:**用于注册difop消息回调函数，当雷达驱动收到LidarPacketMsg消息时，调用回调函数<br/>
      > **参数:**LidarPacketMsg回调函<br/>
      > **返回值:**void<br/>

      + inline void regExceptionCallback(const std::function<void(const Error &)> excallBack)
      > **功能:**用于注册ErrorCode回调函数<br/>
      > **参数:**ErrCode回调函数<br/>
      > **返回值:**void<br/>

      + inline void decodeMsopScan(const LidarScanMsg &pkt_scan_msg, LidarPointcloudMsg<PointT> &point_msg)
      > **功能:**用于解msop数据<br/>
      > **参数:**LidarScanMsg，msop消息类型；LidarPointsMsg<PointT>，点云消息类型<br/>
      > **返回值:**void<br/>

      + inline void decodeDifopPkt(const LidarPacketMsg &pkt_msg)
      > **功能:**用于解difop数据<br/>
      > **参数:**LidarPacketMsg, difop消息类型<br/>
      > **返回值:**void<br/>

### 8. 使用说明

ROS下的在线驱动使用示例:主要包括需要连接雷达，正确配置参数，然后初始化雷达驱动，设置正确的回调函数，最后启动雷达驱动。demo详情如下所示:
```C++
#include "rs_driver/interface/lidar_interface.h"
using namespace robosense::lidar;

struct PointXYZI ///< user defined point type
{
    double x;
    double y;
    double z;
    double intensity;
};

/**
 * @description: The point cloud callback function. This funciton will be registered to lidar driver.
 *              When the point cloud message is ready, driver can send out message through this function.
 * @param msg  The lidar point cloud message. 
 */
void pointCloudCallback(const PointcloudMsg<PointXYZI> &msg)
{
    /* Note: Please do not put time-consuming operations in the callback function!
     Make a copy of the message and process it in another thread is recommended*/
    std::cout << "msg: " << msg.seq << " pointcloud size: " << msg.pointcloud_ptr->size() << std::endl;
}

/**
 * @description: The exception callback function. This function will be registered to lidar driver.
 * @param code The error code struct.
 */
void exceptionCallback(const Error &code)
{
    /* Note: Please do not put time-consuming operations in the callback function!
     Make a copy of the error message and process it in another thread is recommended*/
    std::cout << "Error code : " << code.toString() << std::endl;
}

int main(int argc, char *argv[])
{
    LidarDriverInterface<PointXYZI> driver;          ///< Declare the driver object
    RSLiDAR_Driver_Param param;                      ///< Creat a parameter object
    param.input_param.msop_port = 6688;              ///< Set the lidar msop port number the default 6699
    param.input_param.difop_port = 7799;             ///< Set the lidar difop port number the default 7788
    param.lidar_type = LiDAR_TYPE::RS16;             ///< Set the lidar type. Make sure this type is correct!
    driver.init(param);                              ///< Call the init funtion and pass the parameter
    driver.regPointRecvCallback(pointCloudCallback); ///< Register the point cloud callback funtion into the driver
    driver.regExceptionCallback(exceptionCallback);  ///<Register the exception callback funtion into the driver
    driver.start();                                  ///< Call the start funtion. The driver thread will start.
    std::cout << "Robosense Lidar-Driver Linux pcap demo start......" << std::endl;
    while (true)
    {
        sleep(1);
    }
}
```