/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
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
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <math.h>
#include "driver/decoder/rs_print.hpp"
namespace robosense
{
namespace sensor
{
#define RS_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define RS_SWAP_LONG(x) ((((x)&0xFF) << 24) | (((x)&0xFF00) << 8) | (((x)&0xFF0000) >> 8) | (((x)&0xFF000000) >> 24))
#define RS_TO_RADS(x) ((x) * (M_PI) / 180)
#define RS_RESOLUTION_5mm_DISTANCE_COEF (0.005)
#define RS_RESOLUTION_10mm_DISTANCE_COEF (0.01)

enum E_DECODER_RESULT
{
    E_DECODE_FAIL = -2,
    E_PARAM_INVALID = -1,
    E_DECODE_OK = 0,
    E_FRAME_SPLIT = 1
};

enum RS_ECHO_MODE
{
    RS_ECHO_DUAL = 0,
    RS_ECHO_MAX,
    RS_ECHO_LAST
};

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif

typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t ms;
    uint16_t us;
}
#ifdef __GNUC__
__attribute__((packed)) 
#endif
ST_Timestamp;

typedef struct
{
    uint64_t sync;
    uint8_t reserved1[12];
    ST_Timestamp timestamp;
    uint8_t lidar_type;
    uint8_t reserved2[7];
    uint16_t temp_raw;
    uint8_t reserved3[2];
} 
#ifdef __GNUC__
__attribute__((packed))
#endif
ST_MsopHeader;

typedef struct
{
    uint8_t lidar_ip[4];
    uint8_t host_ip[4];
    uint8_t mac_addr[6];
    uint16_t local_port;
    uint16_t dest_port;
    uint16_t port3;
    uint16_t port4;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
ST_EthNet;

typedef struct
{
    uint16_t start_angle;
    uint16_t end_angle;
} 
#ifdef __GNUC__
__attribute__((packed))
#endif
ST_FOV;

typedef struct
{
    uint8_t sign;
    uint8_t value[2];
} 
#ifdef __GNUC__
__attribute__((packed))
#endif
ST_CorAngle;

typedef struct
{
    uint16_t distance;
    uint8_t intensity;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
ST_Channel;

typedef struct
{
    uint8_t main_sn[5];
    uint8_t bottom_sn[5];
} 
#ifdef __GNUC__
__attribute__((packed)) 
#endif
ST_Version;

typedef struct
{
    uint8_t num[6];
} 
#ifdef __GNUC__
__attribute__((packed))
#endif
ST_SN;

typedef struct
{
    uint8_t device_current[3];
    uint8_t main_current[3];
    uint16_t vol_12v;
    uint16_t vol_12vm;
    uint16_t vol_5v;
    uint16_t vol_3v3;
    uint16_t vol_2v5;
    uint16_t vol_1v2;
} 
#ifdef __GNUC__
__attribute__((packed))
#endif
ST_Status;

typedef struct
{
    uint8_t reserved1[10];
    uint8_t checksum;
    uint16_t manc_err1;
    uint16_t manc_err2;
    uint8_t gps_status;
    uint16_t temperature1;
    uint16_t temperature2;
    uint16_t temperature3;
    uint16_t temperature4;
    uint16_t temperature5;
    uint8_t reserved2[5];
    uint16_t cur_rpm;
    uint8_t reserved3[7];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
ST_Diagno;

#ifdef _MSC_VER
#pragma pack(pop)
#endif


typedef struct eRSDecoder_Param
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

//----------------- Decoder ---------------------
template <typename vpoint>
class DecoderBase
{
public:
    DecoderBase(RSDecoder_Param &param);
    virtual ~DecoderBase();
    virtual E_DECODER_RESULT processMsopPkt(const uint8_t *pkt, std::vector<vpoint> &pointcloud_vec,int &height);
    virtual int32_t processDifopPkt(const uint8_t *pkt);
    virtual double getLidarTime(const uint8_t *pkt) = 0;
    virtual void loadCalibrationFile(std::string cali_path) = 0;

protected:
    int32_t rpm_;
    uint8_t echo_mode_;
    uint8_t channel_num_;
    float Rx_;
    float Ry_;
    float Rz_;
    float max_distance_;
    float min_distance_;
    int start_angle_;
    int end_angle_;
    bool angle_flag_;
    int temperature_min_;
    int temperature_max_;
    uint32_t pkts_per_frame_;
    uint32_t pkt_counter_;
    uint16_t mode_split_frame_; // 1 - angle,  2 - theoretical packets; 3 - setting packets
    uint32_t num_pkts_split_; // number of setting packets
    int32_t cut_angle_;
    int32_t last_azimuth_;
    //calibration data
    std::string cali_files_dir_;
    uint32_t cali_data_flag_;
    float vert_angle_list_[128];
    float hori_angle_list_[128];
    int32_t channel_cali_[128][51];
    float channel_dis_cali_[4][129];
    std::vector<double> cos_lookup_table_;
    std::vector<double> sin_lookup_table_;

protected:
    virtual float computeTemperatue(const uint16_t temp_raw);
    virtual float distanceCalibration(int32_t distance, int32_t channel, float temp);
    virtual int32_t azimuthCalibration(float azimuth, int32_t channel);
    virtual int32_t decodeMsopPkt(const uint8_t *pkt, std::vector<vpoint> &vec,int &height) = 0;
    virtual int32_t decodeDifopPkt(const uint8_t *pkt) = 0;
};

template <typename vpoint>
DecoderBase<vpoint>::DecoderBase(RSDecoder_Param &param) : rpm_(600),
                                                    pkts_per_frame_(84),
                                                    cali_files_dir_("."),
                                                    pkt_counter_(0),
                                                    last_azimuth_(-36001),
                                                    cali_data_flag_(0x00),
                                                    angle_flag_(true),
                                                    start_angle_(param.start_angle * 100),
                                                    end_angle_(param.end_angle * 100),
                                                    echo_mode_(param.echo),
                                                    max_distance_(param.max_distance),
                                                    min_distance_(param.min_distance),
                                                    mode_split_frame_(param.mode_split_frame),
                                                    num_pkts_split_(param.num_pkts_split),
                                                    cut_angle_(param.cut_angle * 100),
                                                    temperature_min_(31),
                                                    temperature_max_(81)
{
    memset(this->channel_cali_, 0, sizeof(int) * 128 * 51);
    if (cut_angle_ > 36000)
    {
        cut_angle_ = 0;
    }

    if (this->start_angle_ > 36000 || this->start_angle_ < 0 || this->end_angle_ > 36000 || this->end_angle_ < 0)
    {
        this->start_angle_ = 0;
        this->end_angle_ = 36000;
    }
    if (this->start_angle_ > this->end_angle_)
    {
        this->angle_flag_ = false;
    }
    cos_lookup_table_.resize(36000);
    sin_lookup_table_.resize(36000);
    for (unsigned int i = 0; i < 36000; i++)
    {
        double rad = RS_TO_RADS(i / 100.0f);
        cos_lookup_table_[i] = std::cos(rad);
        sin_lookup_table_[i] = std::sin(rad);
    }

//    rs_print(RS_INFO, "[RSBASE] Constructor.");
}

template <typename vpoint>
DecoderBase<vpoint>::~DecoderBase()
{
    this->cos_lookup_table_.clear();
    this->sin_lookup_table_.clear();
	
//	rs_print(RS_INFO, "[RSBASE] Destructor.");
}

template <typename vpoint>
int32_t DecoderBase<vpoint>::processDifopPkt(const uint8_t *pkt)
{
    if (pkt == NULL)
    {
//		rs_print(RS_ERROR, "[RSBASE] DIFOP pkt buffer NULL.");
        return -1;
    }
    return decodeDifopPkt(pkt);
}

template <typename vpoint>
E_DECODER_RESULT DecoderBase<vpoint>::processMsopPkt(const uint8_t *pkt, std::vector<vpoint> &pointcloud_vec,int &height)
{
    if (pkt == NULL)
    {
	//	rs_print(RS_ERROR, "[RSBASE] MSOP pkt buffer NULL.");
        return E_PARAM_INVALID;
    }

    int azimuth = decodeMsopPkt(pkt, pointcloud_vec,height);
    if (azimuth < 0)
    {
   //   rs_print(RS_ERROR, "[RSBASE] MSOP pkt decode fail.");
        return E_DECODE_FAIL;
    }

    this->pkt_counter_++;

    if (mode_split_frame_ == 1)
    {
        if (azimuth < this->last_azimuth_)
        {
            this->last_azimuth_ -= 36000;
        }
        if (this->last_azimuth_ != -36001 && this->last_azimuth_ < this->cut_angle_ && azimuth >= this->cut_angle_)
        {
            this->last_azimuth_ = azimuth;
            this->pkt_counter_ = 0;
            return E_FRAME_SPLIT;
        }
        this->last_azimuth_ = azimuth;
    }
    else if (mode_split_frame_ == 2)
    {
        if (this->pkt_counter_ >= this->pkts_per_frame_)
        {
            this->pkt_counter_ = 0;
            return E_FRAME_SPLIT;
        }
    }
    else if (mode_split_frame_ == 3)
    {
        if (this->pkt_counter_ >= this->num_pkts_split_)
        {
            this->pkt_counter_ = 0;
            return E_FRAME_SPLIT;
        }
    }

    return E_DECODE_OK;
}

template <typename vpoint>
float DecoderBase<vpoint>::computeTemperatue(const uint16_t temp_raw)
{
    uint8_t neg_flag = (temp_raw >> 8) & 0x80;
    float msb = (temp_raw >> 8) & 0x7F;
    float lsb = (temp_raw & 0xFF00) >> 3;
    float temp;
    if (neg_flag == 0x80)
    {
        temp = -1 * (msb * 32 + lsb) * 0.0625f;
    }
    else
    {
        temp = (msb * 32 + lsb) * 0.0625f;
    }

    return temp;
}

template <typename vpoint>
float DecoderBase<vpoint>::distanceCalibration(int distance, int channel, float temp)
{
    int temp_idx = (int)floor(temp + 0.5);
    if (temp_idx < temperature_min_)
    {
        temp_idx = 0;
    }
    else if (temp_idx > temperature_max_)
    {
        temp_idx = temperature_max_ - temperature_min_;
    }
    else
    {
        temp_idx = temp_idx - temperature_min_;
    }
    float estimate_distance = 0.0;
    float dis_ret = this->channel_cali_[channel][temp_idx];
    if (distance < dis_ret)
    {
        estimate_distance = 0.0;
    }
    else
    {
        estimate_distance = (float)(distance - dis_ret);
    }

    return estimate_distance;
}

template <typename vpoint>
int DecoderBase<vpoint>::azimuthCalibration(float azimuth, int channel)
{
    int azi_ret;

    if (azimuth > 0.0 && azimuth < 3000.0)
    {
        azimuth = azimuth + this->hori_angle_list_[channel] * 100 + 36000.0f;
    }
    else
    {
        azimuth = azimuth + this->hori_angle_list_[channel] * 100;
    }
    azi_ret = (int)azimuth;
    azi_ret %= 36000;

    return azi_ret;
}
} // namespace sensor
} // namespace robosense