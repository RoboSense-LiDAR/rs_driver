# Error Code Introduction

```c++
/**
  * @brief Error Code for RoboSense LiDAR Driver.
  * 0x00 For success info
  * 0x01 ~ 0x40 for Info, information during the program running
  * 0x41 ~ 0x80 for Warning, the program may not work normally
  * 0x81 ~ 0xC0 for Critical Error, the program will exit
  */
enum ErrCode
{
  ErrCode_Success = 0x00,            ///< Normal
  ErrCode_PcapRepeat = 0x01,         ///< The pcap file will play repeatedly
  ErrCode_PcapExit = 0x02,           ///< The pcap thread will exit
  ErrCode_MsopPktTimeout = 0x41,     ///< The msop packets receive overtime (1 sec)
  ErrCode_DifopPktTimeout = 0x42,    ///< The difop packets receive overtime (2 sec)
  ErrCode_MsopPktIncomplete = 0x43,  ///< The incomplete msop packets received
  ErrCode_DifopPktIncomplete = 0x44, ///< The incomplete difop packets received
  ErrCode_NoDifopRecv = 0x45,        ///< The point cloud decoding process will not start until the difop packet receive
  ErrCode_ZeroPoints = 0x46,         ///< The size of the point cloud is zero
  ErrCode_StartBeforeInit = 0x47,    ///< The start() function is called before initializing successfully
  ErrCode_PcapWrongDirectory = 0x48, ///< The input directory of pcap file is wrong
  ErrCode_MsopPortBuzy = 0x49,       ///< The input msop port is already used
  ErrCode_DifopPortBuzy = 0x50,      ///< The input difop port is already used
  ErrCode_WrongPktHeader = 0x51,     ///< The packet header is wrong
  ErrCode_PktNull = 0x52,            ///< The input packet is null
  ErrCode_PktBufOverFlow = 0x53      ///< The packet buffer is over flow


};
```

