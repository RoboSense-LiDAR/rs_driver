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
         * @description:Error Code for Robosense LiDAR Driver.
             * 0x01 ~ 0x40 for Infos, some infomation during the program running
             * 0x41 ~ 0x80 for Warning, the program may not work normally
             * 0x81 ~ 0xC0 for Critical Error, the program will exit
         */
        enum class ErrCodeType
        {
            INFO_CODE,    ///< Common information
            WARNING_CODE, ///< The program may not work normally
            ERROR_CODE    ///< The program will exit immediately
        };
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

        struct Error
        {
            ErrCode error_code;
            ErrCodeType error_code_type;
            Error(const ErrCode &_code) : error_code(_code)
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
                case ErrCode_PcapContradiction:
                    return "ErrCode_PcapContradiction";
                case ErrCode_MsopPortBuzy:
                    return "ErrCode_MsopPortBuzy";
                case ErrCode_DifopPortBuzy:
                    return "ErrCode_DifopPortBuzy";
                case ErrCode_PcapFinished:
                    return "ErrCode_PcapFinished";
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
                default:
                    return "ErrCode_Success";
                }
            }
        };

    } // namespace lidar
} // namespace robosense
