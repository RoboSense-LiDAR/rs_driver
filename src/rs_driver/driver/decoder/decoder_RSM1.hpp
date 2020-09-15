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
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

#include <rs_driver/driver/decoder/decoder_base.hpp>

namespace robosense
{
namespace lidar
{
using namespace boost::numeric::ublas;

#pragma pack(push, 1)
#define RSM1_ANGLE_RESOLUTION (0.01)
#define RSM1_DISTANCE_RESOLUTION (0.005)
#define RSM1_BLOCKS_PER_PKT (50)
#define RSM1_SCANS_PER_FIRING (5)
#define RSM1_POINTNUM_PER_VIEW (15750 * 2)  // TODO, double wave mode now

typedef struct
{
  uint16_t distance;
  uint16_t intensity;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1Channel;

typedef struct
{
  uint16_t pitch;
  uint16_t yaw;
  RSM1Channel channel[RSM1_SCANS_PER_FIRING];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1Block;

typedef struct
{
  uint8_t sync[4];
  uint8_t cmd[4];
  uint8_t reserved_front[22];
  uint8_t lidar_type;
  uint8_t wave_mode;
  uint8_t mems_temp;
  uint8_t pps_lock;
  RSTimestampUTC timestamp_utc;
} RSM1MsopHeader;

typedef struct raw_mems_packet
{
  RSM1MsopHeader header;
  RSM1Block blocks[RSM1_BLOCKS_PER_PKT];
  uint8_t reserved_back[4];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1MsopPkt;

typedef struct
{
  unsigned char ip_local[4];
  unsigned char ip_remote[4];
  unsigned char mac[6];
  unsigned char ip_msop_port[2];
  unsigned char net_reserve1[2];
  unsigned char ip_difop_port[2];
  unsigned char net_reserve2[2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopEther;

typedef struct
{
  unsigned char fov_start[2];
  unsigned char fov_end[2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopFov;

typedef struct
{
  unsigned char pl_ver[5];
  unsigned char ps_ver[5];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopVerInfo;

typedef struct
{
  unsigned char syn_methord;
  unsigned char syn_status;
  unsigned char current_time[10];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopTimeInfo;

typedef struct
{
  unsigned char current1[3];
  unsigned char current2[3];
  unsigned char voltage1[2];
  unsigned char voltage2[2];
  unsigned char reserve[8];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopRunSts;

typedef struct
{
  unsigned char param_sign;
  unsigned char data[2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopCalibration;

typedef struct
{
  uint64_t id;
  unsigned char reserve1;
  unsigned char frame_rate;
  RSM1DifopEther ether;
  RSM1DifopFov fov_setting;
  unsigned char reserve2[4];
  RSM1DifopVerInfo ver_info;
  unsigned char reserve3[242];
  unsigned char serial_number[6];
  unsigned char reserve4[2];
  unsigned char wave_mode;
  RSM1DifopTimeInfo time_info;
  RSM1DifopRunSts run_status;
  unsigned char reserve5[11];
  unsigned char diag_reserve[40];
  unsigned char reserve6[86];
  RSM1DifopCalibration cali_param[20];
  unsigned char reserve7[718];
  unsigned char frame_tail[2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
RSM1DifopPkt;
#pragma pack(pop)

template <typename vpoint>
class DecoderRSM1 : public DecoderBase<vpoint>
{
public:
  DecoderRSM1(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<vpoint>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);
  void loadCalibrationFile(const std::string& calibration_file_path);
  RSDecoderResult processMsopPkt(const uint8_t* pkt, std::vector<vpoint>& pointcloud_vec, int& height);
  float pitchConvertDeg(int deg);
  float yawConvertDeg(int deg);
  float pixelToDistance(int distance, int dsr);

private:
  int channel_num_[RSM1_SCANS_PER_FIRING + 1];
  float pitch_rate_;
  float yaw_rate_;
  float pitch_offset_[RSM1_SCANS_PER_FIRING + 1];
  float yaw_offset_[RSM1_SCANS_PER_FIRING + 1];
  float distance_max_thd_;
  float distance_min_thd_;

  uint32_t point_idx_;
  int last_pitch_index_;
  int skip_block_;
  int last_pkt_index_;
  int pitch_max_;
  int pitch_max_last_frame_;
  matrix<float> input_ref_;
  matrix<float> rotate_gal_;
  /* tan lookup table */
  std::vector<float> tan_lookup_table_;
};

template <typename vpoint>
DecoderRSM1<vpoint>::DecoderRSM1(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : DecoderBase<vpoint>(param, lidar_const_param), input_ref_(matrix<float>(3, 5)), rotate_gal_(matrix<float>(3, 3))
{
  point_idx_ = 0;
  last_pitch_index_ = 0;
  skip_block_ = 0;
  last_pkt_index_ = -1;
  distance_max_thd_ = param.max_distance;
  distance_min_thd_ = param.min_distance;
  pitch_max_ = 65535;
}

template <typename vpoint>
double DecoderRSM1<vpoint>::getLidarTime(const uint8_t* pkt)
{
  RSM1MsopPkt* mpkt_ptr = (RSM1MsopPkt*)pkt;

  union u_ts
  {
    uint8_t data[8];
    uint64_t ts;
  } timestamp;
  timestamp.data[7] = 0;
  timestamp.data[6] = 0;
  timestamp.data[5] = mpkt_ptr->header.timestamp_utc.sec[0];
  timestamp.data[4] = mpkt_ptr->header.timestamp_utc.sec[1];
  timestamp.data[3] = mpkt_ptr->header.timestamp_utc.sec[2];
  timestamp.data[2] = mpkt_ptr->header.timestamp_utc.sec[3];
  timestamp.data[1] = mpkt_ptr->header.timestamp_utc.sec[4];
  timestamp.data[0] = mpkt_ptr->header.timestamp_utc.sec[5];
  return (double)timestamp.ts + ((double)(RS_SWAP_LONG(mpkt_ptr->header.timestamp_utc.ns))) / 1000000000.0d;
}
template <typename vpoint>
RSDecoderResult DecoderRSM1<vpoint>::processMsopPkt(const uint8_t* pkt, std::vector<vpoint>& pointcloud_vec,
                                                    int& height)
{
  int azimuth = 0;
  RSDecoderResult ret = decodeMsopPkt(pkt, pointcloud_vec, height, azimuth);
  return ret;
}

template <typename vpoint>
float DecoderRSM1<vpoint>::pitchConvertDeg(int deg)  // convert deg into  -625 ~ +625
{
  float result_f;
  float deg_f = deg;

  result_f = deg_f * (1250.0f / (0 - pitch_max_)) + 625.0f;
  return result_f;
}

template <typename vpoint>
float DecoderRSM1<vpoint>::yawConvertDeg(int deg)  // convert deg into  -675 ~ 675
{
  float result_f;
  float deg_f = deg;

  result_f = (deg_f * (1350.0f / 65534.0f) - 675.0f);
  return result_f;
}
template <typename vpoint>
float DecoderRSM1<vpoint>::pixelToDistance(int distance, int dsr)
{
  float result;
  int cor = channel_num_[dsr];

  if (distance <= cor)
  {
    result = 0.0;
  }
  else
  {
    result = distance - cor;
  }
  return result;
}

template <typename vpoint>
RSDecoderResult DecoderRSM1<vpoint>::decodeMsopPkt(const uint8_t* pkt, std::vector<vpoint>& vec, int& height,
                                                   int& azimuth)
{
  azimuth = 0;  // TODO mems not use this
  height = this->lidar_const_param_.LASER_NUM;
  vpoint point;

  RSM1MsopPkt* raw = (RSM1MsopPkt*)pkt;
  int pkt_index = 256 * raw->header.cmd[0] + raw->header.cmd[1];

  // TODO
  if (pkt_index - last_pkt_index_ > 1 && pkt_index - last_pkt_index_ < RSM1_POINTNUM_PER_VIEW / RSM1_BLOCKS_PER_PKT &&
      last_pkt_index_ != -1)
  {
    int lose_pkt_count = pkt_index - last_pkt_index_ - 1;
    while (lose_pkt_count--)
    {
      for (int block = 0; block < RSM1_BLOCKS_PER_PKT; block++)
      {
        for (int dsr = 0; dsr < RSM1_SCANS_PER_FIRING; dsr++)
        {
          point.x = NAN;
          point.y = NAN;
          point.z = NAN;
          point.intensity = 0;
          vec.emplace_back(std::move(point));
        }
        point_idx_++;
      }
    }
  }

  last_pkt_index_ = pkt_index;

  // unpack block
  for (int block = 0; block < RSM1_BLOCKS_PER_PKT; block++)  // 1 packet:25 data blocks
  {
    RSM1Block* p_block_pkt = (RSM1Block*)(raw->blocks + block);
    int index_temp = RS_SWAP_SHORT(p_block_pkt->pitch);

    // find the pitch_max
    if (index_temp > pitch_max_last_frame_)
    {
      pitch_max_last_frame_ = index_temp;
    }
    float pitch = pitchConvertDeg(index_temp) * RSM1_ANGLE_RESOLUTION;                   // degree
    float yaw = yawConvertDeg(RS_SWAP_SHORT(p_block_pkt->yaw)) * RSM1_ANGLE_RESOLUTION;  // degree
    // unpack points
    vector<float> n_gal(3, 1);
    vector<float> i_out(3, 1);
    for (int dsr = 0; dsr < RSM1_SCANS_PER_FIRING; dsr++)  // 5 channels, channel index 1 ~ 5
    {
      float tanax = 0.0f, tanay = 0.0f;
      int ax = 0, ay = 0;

      ax = (int)(100 * pitch_rate_ * (pitch + pitch_offset_[dsr]));  // ax: -1000 ~ 1000, 0.01deg
      ay = (int)(100 * yaw_rate_ * (yaw + yaw_offset_[dsr]));        // ay: -1000 ~ 1000, 0.01deg
      tanax = this->tan_lookup_table_[ax + 1000];
      tanay = this->tan_lookup_table_[ay + 1000];
      tanax = -tanax;
      tanay = -tanay;

      // calc i_out
      n_gal(0) = tanay;
      n_gal(1) = -tanax;
      n_gal(2) = 1;

      float norm_n_gal = norm_2(n_gal);
      n_gal(0) = n_gal(0) / norm_n_gal;
      n_gal(1) = n_gal(1) / norm_n_gal;
      n_gal(2) = n_gal(2) / norm_n_gal;

      n_gal = prod(rotate_gal_, n_gal);
      i_out = column(input_ref_, dsr) - 2 * prod(outer_prod(n_gal, n_gal), column(input_ref_, dsr));

      float distance = pixelToDistance(RS_SWAP_SHORT(p_block_pkt->channel[dsr].distance), dsr);
      distance = distance * RSM1_DISTANCE_RESOLUTION;

      if (distance > distance_max_thd_ || distance < distance_min_thd_)  // invalid data
      {
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
        point.intensity = 0;
      }
      else
      {
        point.x = i_out(2) * distance;
        point.y = i_out(0) * distance;
        point.z = i_out(1) * distance;
        point.intensity = RS_SWAP_SHORT(p_block_pkt->channel[dsr].intensity);
      }
      vec.emplace_back(std::move(point));
    }
    point_idx_++;
  }

  if (pkt_index < last_pkt_index_ || pkt_index == 630)  // new frame
  {
    last_pkt_index_ = pkt_index;
    point_idx_ = 0;
    pitch_max_ = pitch_max_last_frame_;
    pitch_max_last_frame_ = 0;
    return RSDecoderResult::FRAME_SPLIT;
  }

  return RSDecoderResult::DECODE_OK;
}

template <typename vpoint>
RSDecoderResult DecoderRSM1<vpoint>::decodeDifopPkt(const uint8_t* pkt)
{
  RSM1DifopPkt* difop_ptr = (RSM1DifopPkt*)pkt;

  if (difop_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }

  if (!this->difop_flag_)
  {
    bool cali_param_flag = false;
    // check difop reigon has been flashed the right data
    for (int i = 9; i < 20; ++i)
    {
      if ((difop_ptr->cali_param[i].data[0] != 0x00 || difop_ptr->cali_param[i].data[1] != 0x00))
      {
        cali_param_flag = true;
      }
    }

    // angle
    if (cali_param_flag)
    {
      int bit1, bit2, bit3, symbolbit = 0;
      for (int i = 0; i < RSM1_SCANS_PER_FIRING + 1; ++i)
      {
        // distance cor
        bit1 = static_cast<int>(difop_ptr->cali_param[0 + i].param_sign);
        bit2 = static_cast<int>(difop_ptr->cali_param[0 + i].data[0]);
        bit3 = static_cast<int>(difop_ptr->cali_param[0 + i].data[1]);
        if (bit1 == 0)
          symbolbit = 1;
        else if (bit1 == 1)
          symbolbit = -1;
        channel_num_[i] = 0.01 * (bit2 * 256 + bit3) * symbolbit;
        // pitch offset angle
        bit1 = static_cast<int>(difop_ptr->cali_param[8 + i].param_sign);
        bit2 = static_cast<int>(difop_ptr->cali_param[8 + i].data[0]);
        bit3 = static_cast<int>(difop_ptr->cali_param[8 + i].data[1]);
        if (bit1 == 0)
        {
          symbolbit = 1;
        }
        else if (bit1 == 1)
        {
          symbolbit = -1;
        }
        pitch_offset_[i] = 0.01 * (bit2 * 256 + bit3) * symbolbit;
        // yaw offset angle
        bit1 = static_cast<int>(difop_ptr->cali_param[14 + i].param_sign);
        bit2 = static_cast<int>(difop_ptr->cali_param[14 + i].data[0]);
        bit3 = static_cast<int>(difop_ptr->cali_param[14 + i].data[1]);
        if (bit1 == 0)
        {
          symbolbit = 1;
        }
        else if (bit1 == 1)
        {
          symbolbit = -1;
        }
        yaw_offset_[i] = 0.01 * (bit2 * 256 + bit3) * symbolbit;
      }
      // pitch_rate
      bit1 = static_cast<int>(difop_ptr->cali_param[6].param_sign);
      bit2 = static_cast<int>(difop_ptr->cali_param[6].data[0]);
      bit3 = static_cast<int>(difop_ptr->cali_param[6].data[1]);
      if (bit1 == 0)
        symbolbit = 1;
      else if (bit1 == 1)
        symbolbit = -1;
      pitch_rate_ = 0.01 * (bit2 * 256 + bit3) * symbolbit;

      // yaw_rate
      bit1 = static_cast<int>(difop_ptr->cali_param[7].param_sign);
      bit2 = static_cast<int>(difop_ptr->cali_param[7].data[0]);
      bit3 = static_cast<int>(difop_ptr->cali_param[7].data[1]);
      if (bit1 == 0)
        symbolbit = 1;
      else if (bit1 == 1)
        symbolbit = -1;
      yaw_rate_ = 0.01 * (bit2 * 256 + bit3) * symbolbit;
    }
    this->difop_flag_ = true;
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename vpoint>
void DecoderRSM1<vpoint>::loadCalibrationFile(const std::string& calib_path)
{
  FILE* f_channel = fopen((calib_path + "/ChannelNum.csv").c_str(), "r");
  if (!f_channel)
  {
    // TODO
    RS_WARNING << "ChannelNum.csv does not exist! Please check the path: " << (calib_path + "/ChannelNum.csv")
               << RS_REND;
    for (int i = 0; i < RSM1_SCANS_PER_FIRING + 1; i++)
    {
      channel_num_[i] = 0;
      pitch_offset_[i] = 0;
      yaw_offset_[i] = 0;
    }
    pitch_rate_ = 1;
    yaw_rate_ = 1;
  }
  else
  {
    int loopm = 0;
    float tmp_buf[32];
    const float default_temp_buf[32] = { 0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,  // distance correction offset
                                         1.0f,    1.0f,                                    // pitch yaw rate
                                         0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,  // pitch offset
                                         0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,  // yaw offset
                                         -6.875f, 6.875f, -6.875f, 6.875f, -6.875f, 6.875f,
                                         -6.875f, 6.875f, -6.875f, 6.875f, -6.875f, 6.875f };  // yaw start angle
    while (!feof(f_channel))
    {
      if (1 != fscanf(f_channel, "%f%*[^\n]%*c\n", &tmp_buf[loopm]))
      {
        break;
      }
      loopm++;
      if (loopm >= 32)
      {
        break;
      }
    }
    if (loopm != 32)
    {
      std::cout << "channel num file not correct" << std::endl;
      for (int i = 0; i < 32; ++i)
      {
        tmp_buf[i] = default_temp_buf[i];
      }
    }

    for (int i = 0; i < RSM1_SCANS_PER_FIRING + 1; i++)
    {
      channel_num_[i] = (int)(tmp_buf[i]);
      pitch_offset_[i] = tmp_buf[8 + i];
      yaw_offset_[i] = tmp_buf[14 + i];
    }
    pitch_rate_ = tmp_buf[6];
    yaw_rate_ = tmp_buf[7];

    fclose(f_channel);
  }

  // lookup table init, -10 ~ 10 deg, 0.01 resolution
  this->tan_lookup_table_.resize(2000);

  for (int i = -1000; i < 1000; i++)
  {
    double rad = RS_TO_RADS(i / 100.0f);
    this->tan_lookup_table_[i + 1000] = std::tan(rad);
  }

  input_ref_(0, 0) = 0.748956;
  input_ref_(0, 1) = 0.410719;
  input_ref_(0, 2) = 0;
  input_ref_(0, 3) = -0.410719;
  input_ref_(0, 4) = -0.748956;

  input_ref_(1, 0) = 0.360889;
  input_ref_(1, 1) = 0.496581;
  input_ref_(1, 2) = 0.544639;
  input_ref_(1, 3) = 0.496581;
  input_ref_(1, 4) = 0.360889;

  input_ref_(2, 0) = -0.55572;
  input_ref_(2, 1) = -0.764668;
  input_ref_(2, 2) = -0.838671;
  input_ref_(2, 3) = -0.764668;
  input_ref_(2, 4) = -0.55572;

  rotate_gal_(0, 0) = 1;
  rotate_gal_(0, 1) = 0;
  rotate_gal_(0, 2) = 0;

  rotate_gal_(1, 0) = 0;
  rotate_gal_(1, 1) = 0.958820;
  rotate_gal_(1, 2) = -0.284015;

  rotate_gal_(2, 0) = 0;
  rotate_gal_(2, 1) = 0.284015;
  rotate_gal_(2, 2) = 0.958820;
}

}  // namespace lidar
}  // namespace robosense
