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
#include <string>

namespace robosense
{

    namespace lidar
    {

        enum class LiDAR_TYPE ///< The lidar type
        {
            RS16,
            RS32,
            RSBP,
            RS128
        };
        enum RS_ECHO_MODE ///< The lidar echo mode. Need to be set correspond to the hardware actual working mode.
        {
            RS_ECHO_DUAL = 0,
            RS_ECHO_MAX,
            RS_ECHO_LAST
        };
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

        typedef struct RSInput_Param ///< The lidar input parameter
        {
            std::string device_ip = "192.168.1.200"; ///< The ip of lidar
            uint16_t msop_port = 6699;               ///< The msop packet port number
            uint16_t difop_port = 7788;              ///< The difop packet port number
            bool read_pcap = false;                  ///< True: The driver will process the pcap through pcap_file_dir. False: The driver will get data from online lidar
            bool pcap_repeat = false;                ///< True: The pcap bag will repeat play
            std::string pcap_file_dir = "";          ///< The absolute path of pcap file
        } RSInput_Param;

        typedef struct RSLiDAR_Driver_Param ///< The lidar driver parameter
        {
            std::string angle_path = "";              ///< The path of the folder which store lidar calibration files
            std::string frame_id = "rslidar";         ///< The frame id of lidar message
            LiDAR_TYPE lidar_type = LiDAR_TYPE::RS16; ///< Lidar type
            bool use_lidar_clock = false;             ///< True: lidar message timestamp is the lidar clock. False: timestamp is the computer system clock
            RSInput_Param input_param;
            RSDecoder_Param decoder_param;
        } RSLiDAR_Driver_Param;
    } // namespace lidar
} // namespace robosense