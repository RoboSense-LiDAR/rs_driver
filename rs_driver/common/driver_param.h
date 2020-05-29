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
enum class LiDAR_TYPE
{
    RS16,
    RS32,
    RSBP,
    RS128
};
enum RS_ECHO_MODE
{
    RS_ECHO_DUAL = 0,
    RS_ECHO_MAX,
    RS_ECHO_LAST
};
typedef struct RSDecoder_Param
{
    RS_ECHO_MODE echo = RS_ECHO_MAX;
    float max_distance = 200.0f;
    float min_distance = 0.2f;
    float start_angle = 0.0f;
    float end_angle = 360.0f;
    uint16_t mode_split_frame = 1;
    uint32_t num_pkts_split = 0;
    float cut_angle = 0.0f;
} RSDecoder_Param;

typedef struct RSInput_Param
{
    std::string device_ip = "192.168.1.200";
    uint16_t msop_port = 6699;
    uint16_t difop_port = 7788;
    bool read_pcap = false;
    bool pcap_repeat = false;
    std::string pcap_file_dir = "";
} RSInput_Param;

typedef struct RSLiDAR_Driver_Param
{
    std::string calib_path = "";
    std::string frame_id = "rslidar";
    LiDAR_TYPE lidar_type = LiDAR_TYPE::RS16;
    bool use_lidar_clock = false;
    RSInput_Param input_param;
    RSDecoder_Param decoder_param;
} RSLiDAR_Driver_Param;