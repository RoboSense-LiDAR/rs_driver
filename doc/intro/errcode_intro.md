# Error Code Introduction

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
            ErrCode_PcapRepeat = 0x01,         ///< The pcap file will repeat play.
            ErrCode_PcapExit = 0x02,           ///< The pcap thread will exit.
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

