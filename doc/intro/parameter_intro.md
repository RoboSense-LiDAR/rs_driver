# Parameters Introduction

#### 1. RSDecoderParam
```c++
typedef struct RSDecoderParam ///< The lidar decoder parameter
{
    float max_distance = 200.0f;   ///< The max distance of lidar detect range
    float min_distance = 0.2f;     ///< The minimum distance of lidar detect range
    float start_angle = 0.0f;      ///< The start angle of point cloud
    float end_angle = 360.0f;      ///< The end angle of point cloud
    uint16_t mode_split_frame = 1; ///< 1: Split frame depends on cut_angle; 2:Split frame depends on packet rate; 3:Split frame depends on num_pkts_split
    uint32_t num_pkts_split = 0;   ///< The number of packets in one frame, only be used when mode_split_frame=3
    float cut_angle = 0.0f;        ///< The cut angle used to split frame, only be used when mode_split_frame=1
    void print() const             ///< This function is used to print all the parameters for debug
} RSDecoderParam;
```



#### 2. RSInputParam

```c++
typedef struct RSInputParam ///< The lidar input parameter
{
    std::string device_ip = "192.168.1.200"; ///< The ip of lidar
    uint16_t msop_port = 6699;               ///< The msop packet port number
    uint16_t difop_port = 7788;              ///< The difop packet port number
    bool read_pcap = false;                  ///< True: The driver will process the pcap through pcap_directory. False: The driver will get data from online lidar
    double pcap_rate = 1;                    ///< The rate to read the pcap file
    bool pcap_repeat = true;                 ///< True: The pcap bag will repeat play
    std::string pcap_directory = "null";      ///< The absolute path of pcap file
    void print() const                       ///< This function is used to print all the parameters for debug
} RSInputParam;
```




#### 3. RSDriverParam
```c++
typedef struct RSDriverParam ///< The lidar driver parameter
{
    RSInputParam input_param;                 ///< The input parameter
    RSDecoderParam decoder_param;             ///< The decoder parameter
    std::string angle_path = "null";          ///< The path of angle calibration files(angle.csv)(for latest version lidar, this file is not needed)
    std::string frame_id = "rslidar";         ///< The frame id of lidar message
    LidarType lidar_type = LidarType::RS16;   ///< Lidar type
    bool use_lidar_clock = false;             ///< True: lidar message timestamp is the lidar clock. False: timestamp is the computer system clock
    bool wait_for_difop = true;               ///< True: start sending pointcloud until receive difop packet
    void print() const                        ///< This function is used to print all the parameters for debug
} RSDriverParam;
```



