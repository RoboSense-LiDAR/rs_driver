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

namespace robosense
{
    namespace lidar
    {
        /**
         * @description:Error Code for Robosense LiDAR Driver.
             * 0x01 ~ 0x40 for normal
             * 0x41 ~ 0x80 for Warning
             * 0x81 ~ 0xC0 for Critical Error
         */
        enum ErrCode
        {
            ErrCode_PcapWrongDirectory = 0x81, ///< The input directory of pcap file is wrong
            ErrCode_PcapContradiction = 0x82,  ///< The pcap function is disable but try to decode pcap file
        };

        struct Error
        {
            ErrCode error_code;
            Error(const ErrCode &_code) : error_code(_code) {}
            std::string toString() const
            {
                switch (error_code)
                {
                case ErrCode_PcapWrongDirectory:
                    return "ErrCode_PcapWrongDirectory";
                case ErrCode_PcapContradiction:
                    return "ErrCode_PcapContradiction";
                }
            }
        };

    } // namespace lidar
} // namespace robosense
