/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#pragma once
#include <rs_driver/common/common_header.h>
namespace robosense
{
namespace lidar
{
/**
 * @brief Error Code for RoboSense LiDAR Driver.
 * 0x00 For success info
 * 0x01 ~ 0x40 for Infos, some infomation during the program running
 * 0x41 ~ 0x80 for Warning, the program may not work normally
 * 0x81 ~ 0xC0 for Critical Error, the program will exit
 */
enum class ErrCodeType
{
  INFO_CODE,     ///< Common information
  WARNING_CODE,  ///< The program may not work normally
  ERROR_CODE     ///< The program will exit immediately
};
enum ErrCode
{
  ErrCode_Success = 0x00,             ///< Normal
  ErrCode_PcapRepeat = 0x01,          ///< The pcap file will play repeatedly
  ErrCode_PcapExit = 0x02,            ///< The pcap thread will exit
  ErrCode_MsopPktTimeout = 0x41,      ///< The msop packets receive overtime (1 sec)
  ErrCode_DifopPktTimeout = 0x42,     ///< The difop packets receive overtime (2 sec)
  ErrCode_MsopPktIncomplete = 0x43,   ///< The incomplete msop packets received
  ErrCode_DifopPktIncomplete = 0x44,  ///< The incomplete difop packets received
  ErrCode_NoDifopRecv = 0x45,      ///< The point cloud decoding process will not start until the difop packet receive
  ErrCode_ZeroPoints = 0x46,       ///< The size of the point cloud is zero
  ErrCode_StartBeforeInit = 0x47,  ///< The start() function is called before initializing successfully
  ErrCode_PcapWrongDirectory = 0x48,  ///< The input directory of pcap file is wrong
  ErrCode_MsopPortBuzy = 0x49,        ///< The input msop port is already used
  ErrCode_DifopPortBuzy = 0x50,       ///< The input difop port is already used
  ErrCode_WrongPktHeader = 0x51,      ///< The packet header is wrong
  ErrCode_PktNull = 0x52,             ///< The input packet is null
  ErrCode_PktBufOverFlow = 0x53       ///< The packet buffer is over flow
};

struct Error
{
  ErrCode error_code;
  ErrCodeType error_code_type;
  explicit Error(const ErrCode& code) : error_code(code)
  {
    if (error_code <= 0x40)
    {
      error_code_type = ErrCodeType::INFO_CODE;
    }
    else if (error_code <= 0x80)
    {
      error_code_type = ErrCodeType::WARNING_CODE;
    }
    else
    {
      error_code_type = ErrCodeType::ERROR_CODE;
    }
  }
  std::string toString() const
  {
    switch (error_code)
    {
      case ErrCode_PcapWrongDirectory:
        return "ErrCode_PcapWrongDirectory";
      case ErrCode_MsopPortBuzy:
        return "ErrCode_MsopPortBuzy";
      case ErrCode_DifopPortBuzy:
        return "ErrCode_DifopPortBuzy";
      case ErrCode_PcapRepeat:
        return "ErrCode_PcapRepeat";
      case ErrCode_PcapExit:
        return "ErrCode_PcapExit";
      case ErrCode_MsopPktTimeout:
        return "ErrCode_MsopPktTimeout";
      case ErrCode_DifopPktTimeout:
        return "ErrCode_DifopPktTimeout";
      case ErrCode_MsopPktIncomplete:
        return "ErrCode_MsopPktIncomplete";
      case ErrCode_DifopPktIncomplete:
        return "ErrCode_DifopPktIncomplete";
      case ErrCode_NoDifopRecv:
        return "ErrCode_NoDifopRecv";
      case ErrCode_ZeroPoints:
        return "ErrCode_ZeroPoints";
      case ErrCode_StartBeforeInit:
        return "ErrCode_StartBeforeInit";
      case ErrCode_WrongPktHeader:
        return "ErrCode_WrongPktHeader";
      case ErrCode_PktNull:
        return "ErrCode_PktNull";
      case ErrCode_PktBufOverFlow:
        return "ErrCode_PktBufOverFlow";
      default:
        return "ErrCode_Success";
    }
  }
};

}  // namespace lidar
}  // namespace robosense
