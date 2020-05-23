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
#include "driver/decoder/decoder_base.hpp"
namespace robosense
{
namespace sensor
{
#define RS128_MSOP_SYNC (0x5A05AA55) // big endian
#define RS128_BLOCK_ID (0xFE)
#define RS128_DIFOP_SYNC (0x555511115A00FFA5) // big endian
#define RS128_CHANNELS_PER_BLOCK (128)
#define RS128_BLOCKS_PER_PKT (3)
#define RS128_TEMPERATURE_MIN (31)
#define RS128_TEMPERATURE_RANGE (50)
#define RS128_DSR_TOFFSET (3.23)
#define RS128_BLOCK_TDURATION (55.55)

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif
typedef struct
{
    uint8_t id;
    uint8_t ret_id;
    uint16_t azimuth;
    ST_Channel channels[RS128_CHANNELS_PER_BLOCK];
} 
#ifdef __GNUC__
__attribute__((packed))
#endif
ST128_MsopBlock;

typedef struct
{
    uint32_t sync;
    uint8_t reserved1[3];
    uint8_t wave_mode;
    uint8_t temp_low;
    uint8_t temp_high;
    ST_Timestamp timestamp;
    uint8_t reserved2[60];
} 
#ifdef __GNUC__
__attribute__((packed))
#endif
ST128_MsopHeader;

typedef struct
{
    ST128_MsopHeader header;
    ST128_MsopBlock blocks[RS128_BLOCKS_PER_PKT];
    uint32_t index;
}
#ifdef __GNUC__
__attribute__((packed))
#endif 
ST128_MsopPkt;

typedef struct
{
    uint8_t reserved[229];
} 
#ifdef __GNUC__
__attribute__((packed))
#endif 
ST128_Reserved;

typedef struct
{
    uint8_t sync_mode;
    uint8_t sync_sts;
    ST_Timestamp timestamp;
}
#ifdef __GNUC__
__attribute__((packed))
#endif 
ST128_TimeInfo;

typedef struct
{
    uint8_t lidar_ip[4];
    uint8_t dest_ip[4];
    uint8_t mac_addr[6];
    uint16_t msop_port;
    uint16_t reserve_1;
    uint16_t difop_port;
    uint16_t reserve_2;
} 
#ifdef __GNUC__
__attribute__((packed))
#endif
ST128_EthNet;

typedef struct
{
    uint8_t top_firmware_ver[5];
    uint8_t bot_firmware_ver[5];
    uint8_t bot_soft_ver[5];
    uint8_t motor_firmware_ver[5];
    uint8_t hw_ver[3];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
ST128_Version;


typedef struct
{
    uint64_t sync;
    uint16_t rpm;
    ST128_EthNet eth;
    ST_FOV fov;
    uint16_t reserved_0;
    uint16_t lock_phase_angle;
    ST128_Version version;
    uint8_t reserved_1[229];
    ST_SN sn;
    uint16_t zero_cali;
    uint8_t return_mode;
    ST128_TimeInfo time_info;
    ST_Status status;
    uint8_t reserved_2[11];
    ST_Diagno diagno;
    uint8_t gprmc[86];
    ST_CorAngle ver_angle_cali[128];
    ST_CorAngle hori_angle_cali[128];
    uint8_t reserved_3[10];
    uint16_t tail;
}
#ifdef __GNUC__
__attribute__((packed))
#endif 
ST128_DifopPkt;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

template <typename vpoint>
class Decoder128 : public DecoderBase<vpoint>
{
public:
    Decoder128(RSDecoder_Param &param);
    int32_t decodeDifopPkt(const uint8_t *pkt);
    int32_t decodeMsopPkt(const uint8_t *pkt, std::vector<vpoint> &vec,int &height);
    double getLidarTime(const uint8_t *pkt);
    void   loadCalibrationFile(std::string cali_path);
    float computeTemperatue(const uint8_t temp_low, const uint8_t temp_high);
};

template <typename vpoint>
Decoder128<vpoint>::Decoder128(RSDecoder_Param &param) : DecoderBase<vpoint>(param)
{
    this->Rx_ = 0.03615;
    this->Ry_ = -0.017;
    this->Rz_ = 0;
    this->channel_num_ = 128;

    if (param.max_distance > 230.0f || param.max_distance < 2.0f)
    {
        this->max_distance_ = 230.0f;
    }
    else
    {
        this->max_distance_ = param.max_distance;
    }

    if (param.min_distance > 230.0f || param.min_distance > param.max_distance)
    {
        this->min_distance_ = 2.0f;
    }
    else
    {
        this->min_distance_ = param.min_distance;
    }

    int pkt_rate = 6000;
    this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);

//    rs_print(RS_INFO, "[RS128] Constructor.");
}

template <typename vpoint>
double Decoder128<vpoint>::getLidarTime(const uint8_t *pkt)
{
    ST128_MsopPkt *mpkt_ptr = (ST128_MsopPkt *)pkt;
    std::tm stm;
    memset(&stm, 0, sizeof(stm));
    stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
    stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
    stm.tm_mday = mpkt_ptr->header.timestamp.day;
    stm.tm_hour = mpkt_ptr->header.timestamp.hour;
    stm.tm_min = mpkt_ptr->header.timestamp.minute;
    stm.tm_sec = mpkt_ptr->header.timestamp.second;
    return std::mktime(&stm) + (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms) / 1000.0 + (double)RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us) / 1000000.0;
}

template <typename vpoint>
float Decoder128<vpoint>::computeTemperatue(const uint8_t temp_low, const uint8_t temp_high)
{
    float neg_flag = temp_low & 0x80;
    float msb = temp_low & 0x7F;
    float lsb = temp_high >> 4;
    float temp;
    if (neg_flag == 0x80)
    {
        temp = -1 * (msb * 16 + lsb) * 0.0625f;
    }
    else
    {
        temp = (msb * 16 + lsb) * 0.0625f;
    }

    return temp;
}

template <typename vpoint>
int Decoder128<vpoint>::decodeMsopPkt(const uint8_t *pkt, std::vector<vpoint> &vec,int &height)
{ 
    height=128;
    ST128_MsopPkt *mpkt_ptr = (ST128_MsopPkt *)pkt;
    if (mpkt_ptr->header.sync != RS128_MSOP_SYNC)
    {
//      rs_print(RS_ERROR, "[RS128] MSOP pkt sync no match.");
      return -2;
    }

    float azimuth_corrected_float;
    int azimuth_corrected;
    float temperature = computeTemperatue(mpkt_ptr->header.temp_low, mpkt_ptr->header.temp_high);
    int first_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);

    for (int blk_idx = 0; blk_idx < RS128_BLOCKS_PER_PKT; blk_idx++)
    {
        if (mpkt_ptr->blocks[blk_idx].id != RS128_BLOCK_ID)
        {
            break;
        }

        int azimuth_blk = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
        int azi_prev = 0;
        int azi_cur = 0;
        if (this->echo_mode_ == RS_ECHO_DUAL)
        {
            if (blk_idx < (RS128_BLOCKS_PER_PKT - 2)) // 3
            {
                azi_prev = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 2].azimuth);
                azi_cur = azimuth_blk;
            }
            else
            {
                azi_prev = azimuth_blk;
                azi_cur = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 2].azimuth);
            }
        }
        else
        {
            if (blk_idx < (RS128_BLOCKS_PER_PKT - 1)) // 3
            {
                azi_prev = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth);
                azi_cur = azimuth_blk;
            }
            else
            {
                azi_prev = azimuth_blk;
                azi_cur = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth);
            }
        }

        float azimuth_diff = (float)((36000 + azi_prev - azi_cur) % 36000);

        for (int channel_idx = 0; channel_idx < RS128_CHANNELS_PER_BLOCK; channel_idx++)
        {
            int dsr_temp = channel_idx/4;
            dsr_temp = dsr_temp % 16;

            azimuth_corrected_float = azimuth_blk + (azimuth_diff * (dsr_temp * RS128_DSR_TOFFSET) / RS128_BLOCK_TDURATION);
            azimuth_corrected = this->azimuthCalibration(azimuth_corrected_float, channel_idx);
            
            float intensity = mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
            int distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance);
            float distance_cali = distance * RS_RESOLUTION_5mm_DISTANCE_COEF;

            int angle_horiz_ori = (int)(azimuth_corrected_float + 36000) % 36000;
            int angle_horiz = (azimuth_corrected + 36000) % 36000;
            int angle_vert = (((int)(this->vert_angle_list_[channel_idx] * 100) % 36000) + 36000) % 36000;

            vpoint point;
            if ((distance_cali > this->max_distance_ || distance_cali < this->min_distance_)
              ||(this->angle_flag_ && (angle_horiz < this->start_angle_ || angle_horiz > this->end_angle_))
              ||(!this->angle_flag_ && (angle_horiz > this->start_angle_ && angle_horiz < this->end_angle_)))
            {
                point.x = NAN;
                point.y = NAN;
                point.z = NAN;
                point.intensity = 0;
            }
            else
            {
                const double vert_cos_value = this->cos_lookup_table_[angle_vert];
                const double horiz_cos_value = this->cos_lookup_table_[angle_horiz];
                const double horiz_ori_cos_value =this->cos_lookup_table_[angle_horiz_ori];
                point.x = distance_cali * vert_cos_value * horiz_cos_value + this->Rx_ * horiz_ori_cos_value;

                const double horiz_sin_value = this->sin_lookup_table_[angle_horiz];
                const double horiz_ori_sin_value = this->sin_lookup_table_[angle_horiz_ori];
                point.y = -distance_cali * vert_cos_value * horiz_sin_value - this->Rx_ * horiz_ori_sin_value;

                const double vert_sin_value = this->sin_lookup_table_[angle_vert];
                point.z = distance_cali * vert_sin_value + this->Rz_;

                point.intensity = intensity;
                if (std::isnan(point.intensity))
                {
                  point.intensity = 0;
                }
            }

            
            vec.push_back(std::move(point));
        }
    }

    return first_azimuth;
}

template <typename vpoint>
int Decoder128<vpoint>::decodeDifopPkt(const uint8_t *pkt)
{
    ST128_DifopPkt *rs128_ptr = (ST128_DifopPkt *)pkt;
    if (rs128_ptr->sync != RS128_DIFOP_SYNC)
    {
//		rs_print(RS_ERROR, "[RS128] DIFOP pkt sync no match.");
        return -2;
    }

    int pkt_rate = 6000;
    this->rpm_ = rs128_ptr->rpm;

    if (rs128_ptr->return_mode == 0x01 || rs128_ptr->return_mode == 0x02)
    {
        this->echo_mode_ = rs128_ptr->return_mode;
    }
    else
    {
        this->echo_mode_ = RS_ECHO_DUAL;
    }
    
    if (this->echo_mode_ == RS_ECHO_DUAL)
    {
        pkt_rate = pkt_rate * 2;
    }
    this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);

    if (!(this->cali_data_flag_ & 0x2))
    {
        bool angle_flag = true;
        const uint8_t *p_ver_cali;
        p_ver_cali = (uint8_t *)(rs128_ptr->ver_angle_cali);
        if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) && (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
            (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) && (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF))
        {
            angle_flag = false;
        }

        if (angle_flag)
        {
            int lsb, mid, msb, neg = 1;
            for (int i = 0; i < 128; i++)
            {
                // calculation of vertical angle 
                lsb = rs128_ptr->ver_angle_cali[i].sign;
                mid = rs128_ptr->ver_angle_cali[i].value[0];
                msb = rs128_ptr->ver_angle_cali[i].value[1];
                if (lsb == 0)
                {
                    neg = 1;
                }
                else
                {
                    neg = -1;
                }
                this->vert_angle_list_[i] = (mid * 256 + msb) * neg * 0.01f;

                // horizontal offset angle
                lsb = rs128_ptr->hori_angle_cali[i].sign;
                mid = rs128_ptr->hori_angle_cali[i].value[0];
                msb = rs128_ptr->hori_angle_cali[i].value[1];
                if (lsb == 0)
                {
                    neg = 1;
                }
                else
                {
                    neg = -1;
                }
                this->hori_angle_list_[i] = (mid * 256 + msb) * neg * 0.01f;
            }
            this->cali_data_flag_ = this->cali_data_flag_ | 0x2;
        }
    }
    return 0;
}


template <typename vpoint>
void Decoder128<vpoint>::loadCalibrationFile(std::string cali_path)
{
    int row_index = 0;
    int laser_num = 128;
    std::string line_str;
    this->cali_files_dir_ = cali_path;
    std::string angle_file_path = this->cali_files_dir_ + "/angle.csv";

    // read angle.csv
    std::ifstream fd_angle(angle_file_path.c_str(), std::ios::in);
    if (!fd_angle.is_open())
    {
//        rs_print(RS_WARNING, "[RS128] Calibration file: %s does not exist!", angle_file_path.c_str());
        // std::cout << angle_file_path << " does not exist"<< std::endl;
    }
    else
    {
        row_index = 0;
        while (std::getline(fd_angle, line_str))
        {
            std::stringstream ss(line_str);
            std::string str;
            std::vector<std::string> vect_str;
            while (std::getline(ss, str, ','))
            {
                vect_str.push_back(str);
            }
            this->vert_angle_list_[row_index] = std::stof(vect_str[0]); // degree
            this->hori_angle_list_[row_index] = std::stof(vect_str[1]); // degree
            row_index++;
            if (row_index >= laser_num)
            {
                break;
            }
        }
        fd_angle.close();
    }
}


} // namespace sensor
} // namespace robosense