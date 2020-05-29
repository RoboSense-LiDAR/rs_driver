
### 1. 工程简介
  **rs_driver**为速腾聚创雷达驱动内核。支持**RS-LiDAR-16**、**RS-LiDAR-32**、**RS-Bpearl**和**RS-Ruby**的点云数据解析，方便用户二次开发使用。



### 2. 依赖介绍

- Boost
- pthread
- pcap

Boost 与 pthread 均为系统库，可直接链接使用。 Pcap则需使用以下指令安装:

```
sudo apt-get install -y  libpcap-dev
```



### 3. 驱动安装

```sh
	cd rs_driver
    mkdir build
    cmake .. && make -j4
    sudo make install
```



### 4. 参数简介



```
typedef struct RSDecoder_Param ///< The lidar decoder parameter
{
    RS_ECHO_MODE echo_mode = RS_ECHO_MAX; ///< Echo mode
    float max_distance = 200.0f;          ///< The max distance of lidar detect range
    float min_distance = 0.2f;            ///< The minimum distance of lidar detect range
    float start_angle = 0.0f;             ///< The start angle of point cloud
    float end_angle = 360.0f;             ///< The end angle of point cloud
    uint16_t mode_split_frame = 1;        ///< 1: Split frame depends on cut_angle; 2:Split frame depends on packet rate; 3:Split frame depends on num_pkts_split
    uint32_t num_pkts_split = 0;          ///< The number of packets in one frame, only be used when mode_split_frame=3
    float cut_angle = 0.0f;               ///< The cut angle used to split frame, only be used when mode_split_frame=1
} RSDecoder_Param;
```



RSDecoder_Param为雷达解析配置参数:
+ echo_mode用于设置雷达的回波模式；
+ max_distance用于设置雷达的最远有效距离；
+ min_distance用于设置雷达的最短有效距离；
+ start_angle用于设置雷达一帧数据的起始角度；
+ end_angle用于设置雷达一帧数据的结束角度；
+ mode_split_frame 1: 角度分帧模式 2：固定包数分帧 3： 自定义包数分帧
+ num_pkts_split 自定义的分帧包数
+ cut_angle表示雷达的分割角度；



```
typedef struct RSInput_Param ///< The lidar input parameter
{
    std::string device_ip = "192.168.1.200"; ///< The ip of lidar
    uint16_t msop_port = 6699;               ///< The msop packet port number
    uint16_t difop_port = 7788;              ///< The difop packet port number
    bool read_pcap = false;                  ///< True: The driver will process the pcap through pcap_file_dir. False: The driver will get data from online lidar
    bool pcap_repeat = false;                ///< True: The pcap bag will repeat play
    std::string pcap_file_dir = "";          ///< The absolute path of pcap file
} RSInput_Param;
```



RSInput_Param为雷达解析输入参数:
+ device_ip用于设置雷达的IP；
+ msop_port用于设置雷达的msop端口；
+ difop_port用于设置雷达的difop端口；
+ read_pcap用于设置是否读取pcap；
+ pcap_repeat用于设置是否重复读取pcap；
+ pcap_file_dir用于设置pcap数据的路径；



```
typedef struct RSLiDAR_Driver_Param ///< The lidar driver parameter
{
    std::string calib_path = "";              ///< The path of the folder which store lidar calibration files
    std::string frame_id = "rslidar";         ///< The frame id of lidar message
    LiDAR_TYPE lidar_type = LiDAR_TYPE::RS16; ///< Lidar type
    bool use_lidar_clock = false;             ///< True: lidar message timestamp is the lidar clock. False: timestamp is the computer system clock
    RSInput_Param input_param;
    RSDecoder_Param decoder_param;
} RSLiDAR_Driver_Param;
```



RSLiDAR_Driver_Param为雷达驱动配置参数：
+ calib_path用于设置雷达的标定参数路径；
+ frame_id用于设置雷达的frame_id；
+ lidar_type用于设置雷达的类型，包括RS16，RS32,RSBP,RS128；
+ use_lidar_clock用于设置雷达消息时间戳类型，true:使用lidar时间戳,false:使用系统时间戳；
+ input_param用于设置雷达解析输入参数；
+ decoder_param用于设置雷达解析配置参数；



### 5. 消息简介

```
    struct alignas(16) PacketMsg ///< LiDAR single packet message
    {
      double timestamp = 0.0;
      std::string frame_id = "";
      std::array<uint8_t, 1248> packet{};
    };
```



LidarPacketMsg单个packet消息:
+ timestamp用于存储消息的时间戳；
+ frame_id用于存储消息的frame_id；
+ packet用于存储一个packet的雷达数据；

```
    struct alignas(16) ScanMsg
    {
      double timestamp = 0.0;
      uint32_t seq = 0;
      std::string parent_frame_id = "";
      std::string frame_id = "";
      std::vector<PacketMsg> packets; ///< A vector which store a scan of packets (the size of the vector is not fix)
    };
```



LidarScanMsg单帧雷达消息:

+ timestamp用于存储消息的时间戳；
+ seq用于存储消息的序列号；
+ parent_frame_id用于存储消息的parent_frame_id；
+ frame_id用于存储消息的fram_id；
+ packets用于存储一帧雷达消息的所有packet数据；

```
    struct alignas(16) PointcloudMsg
    {
      typedef std::vector<PointT> PointCloud;
      typedef std::shared_ptr<PointCloud> PointCloudPtr;
      typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;
      double timestamp = 0.0;
      uint32_t seq = 0;
      std::string parent_frame_id = "";
      std::string frame_id = "";
      uint32_t height = 0;
      uint32_t width = 0;
      bool is_dense = false;
      bool is_transform = false;
      bool is_motion_correct = false;
      PointCloudPtr pointcloud_ptr;
      PointcloudMsg() = default;
      PointcloudMsg(const PointCloudPtr &_point_ptr) : pointcloud_ptr(_point_ptr)
      {
      }
      typedef std::shared_ptr<PointcloudMsg> Ptr;
      typedef std::shared_ptr<const PointcloudMsg> ConstPtr;
    };
```



LidarPointcloudMsg单帧雷达点云消息:
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

```
        /**
         * @description:Error Code for Robosense LiDAR Driver.
             * 0x01 ~ 0x40 for Infos, some infomation during the program running
             * 0x41 ~ 0x80 for Warning, the program may not work normally
             * 0x81 ~ 0xC0 for Critical Error, the program will exit
         */
        enum ErrCode
        {
            ErrCode_Success = 0x00,            ///< Normal
            ErrCode_PcapFinished = 0x01,       ///< The pcap file is finished.
            ErrCode_PcapRepeat = 0x02,         ///< The pcap file will repeat play.
            ErrCode_PcapExit = 0x03,           ///< The pcap thread will exit.
            ErrCode_MsopPktTimeout = 0x41,     ///< The msop packets receive timeout (1 sec).
            ErrCode_DifopPktTimeout = 0x42,    ///< The difop packets receive timeout (2 sec).oo
            ErrCode_MsopPktIncomplete = 0x43,  ///< The received msop packets incomplete.
            ErrCode_DifopPktIncomplete = 0x44, ///< The received difop packets incomplete.
            ErrCode_NoDifopRecv = 0x45,        ///< The point cloud decoding will not start until the difop packet receive
            ErrCode_ZeroPoints = 0x46,         ///< The size of point cloud is zero. Please check the lidar type parameter when this error occur.
            ErrCode_PcapWrongDirectory = 0x81, ///< The input directory of pcap file is wrong
            ErrCode_PcapContradiction = 0x82,  ///< The pcap function is disable but try to decode pcap file
            ErrCode_MsopPortBuzy = 0x83,       ///< The input msop port is already used
            ErrCode_DifopPortBuzy = 0x84,      ///< The input difop port is already used

        };
```

​	0x00~0x40为提示信息，程序正常运行

​	0x40~0x80为警告信息，程序可能无法正常工作

​	0x80~0xC0为严重错误，程序将立即退出

+ ErrCode_Success: 正常

+ ErrCode_PcapFinished: pcap 文件解析完成；

+ ErrCode_PcapRepeat: pcap将会重复播放；

+ ErrCode_PcapExit: 将会退出pcap线程；

+ ErrCode_MsopPktTimeout: msop消息接收超时(1秒)；

+ ErrCode_DifopPktTimeout: difop消息接收超时(2秒)；

+ ErrCode_MsopPktIncomplete: msop消息接收未完成；

+ ErrCode_DifopPktIncomplete: difop消息接收未完成；

+ ErrCode_NoDifopRecv: 点云解析未开始直到收到difop消息；

+ ErrCode_ZeroPoints: 点云的大小为0，请检查雷达参数是否配置正确；

+ ErrCode_PcapWrongDirectory: pcap文件路径错误；

+ ErrCode_PcapContradiction: pcap功能被禁止，但是仍然尝试解析pcap文件；

+ ErrCode_MsopPortBuzy: msop端口被占用；

+ ErrCode_DifopPortBuzy: difop端口被占用；

  

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
      > **功能:**用于启动雷达驱动<br/>
      > **参数:**无<br/>
      > **返回值:**void<br/>

      + inline void stop()<br/>
      > **功能:**用于停止雷达驱动<br/>
      > **参数:**无<br/>
      > **返回值:**void<br/>

      + inline void regPointRecvCallback(const std::function<void(const LidarPointcloudMsg<PointT> &)> callBack)
      > **功能:**用于注册单帧雷达点云回调函数，当雷达收到一帧LidarPointsMsg时，调用回调函数<br/>
      > **参数:**LidarPointsMsg回调函数*<br/>
      > **返回值:void<br/>

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

### 8. 使用示例


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