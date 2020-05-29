### Error code简介

```c++
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