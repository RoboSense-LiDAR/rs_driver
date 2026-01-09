/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <rs_driver/driver/decoder/decoder.hpp>
#include <rs_driver/utility/simd.hpp>
#include <array>

namespace robosense
{
namespace lidar
{
#pragma pack(push, 1)

typedef struct
{
  uint16_t SecondsHigh;
  uint32_t SecondsLow;
  uint32_t NanoSeconds;
} AC2Timestamp;

typedef struct
{
  uint8_t id[4];
  uint16_t pkt_seq;
  uint8_t frame_cnt;
  uint8_t tof_id;
  uint8_t distance_resolution;
  uint8_t reserved[14];
  uint8_t frame_sync_status;
  uint8_t return_mode;
  uint8_t time_mode;
  AC2Timestamp timestamp;
} AC2MsopHeader;
typedef struct
{
  uint16_t radius;
  uint8_t intensity;
  uint8_t point_attribute;
  uint8_t raw_data[8];
} AC2Wave;

typedef struct
{
  AC2Wave wave[2];
} AC2Pixel;

typedef struct
{
  AC2MsopHeader header;
  AC2Pixel pixels[240];
  uint8_t reserved_1[640];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} AC2MsopPkt;

typedef struct
{
  uint8_t header[10];
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temperature;
  uint64_t timestamp_sec;
  uint64_t timestamp_nsec;
} AC2ImuPkt;

typedef struct
{
  uint8_t id[4];
  uint8_t pkt_seq;
  uint32_t total_size;
  uint8_t data[1400];
} AC2DifopPkt;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderRSAC2 : public Decoder<T_PointCloud>
{
public:
  void decodeDifopPkt(const uint8_t* packet, size_t size);
  bool decodeMsopPkt(const uint8_t* packet, size_t size);

  void decodeImuFrame(const uint8_t* src_data, size_t data_len);
  void decodeImageFrame(const uint8_t* src_data, size_t data_len);
  void decodePcFrame(const uint8_t* src_data, size_t data_len);

  virtual ~DecoderRSAC2() = default;

  explicit DecoderRSAC2(const RSDecoderParam& param);

private:
  std::vector<int16_t> tof_vector_x_array_;
  std::vector<int16_t> tof_vector_y_array_;
  std::vector<int16_t> tof_vector_z_array_;
  static constexpr size_t kBytesPerPixelRgb = 3;
  static constexpr size_t kSrcPixelSize = 4;
  static constexpr size_t kTofHeight = 96;

  static constexpr size_t kHeaderSize = 20;

  static constexpr size_t kPointCloudWidth = 240;
  static constexpr size_t kPointCloudHeight = 192;
  static constexpr size_t kTotalPixels = (kPointCloudWidth * kPointCloudHeight);
  static constexpr size_t kPointCloudWaveNum = 2;
  static constexpr size_t kPointNums = (kTotalPixels * kPointCloudWaveNum);

  static constexpr size_t kImageWidth = 1600;
  static constexpr size_t kImageHeight = 1200;

  static constexpr size_t kAngleSizePerPacket = 1400;
  static constexpr size_t kAnglePacketNum = 198;

  static constexpr uint8_t kTof1Id = 0xA5;
  static constexpr uint8_t kTof2Id = 0xA6;
  static constexpr uint8_t kCmos1Id = 0xAA;
  static constexpr uint8_t kCmos2Id = 0xAB;

  static constexpr size_t kSingleAngleDataBytes = (kTotalPixels * 2);
  static constexpr size_t kAngleDataBytes = (kSingleAngleDataBytes * 3);
  static constexpr size_t kCrcSize = 2;
  static constexpr size_t kAngleDataTotalLen = kAngleDataBytes + kCrcSize;

  static constexpr uint16_t kCrc16InitValue = 0xFFFFU;
  static constexpr uint16_t kSwCrcDataWidth8Bit = 1U;
  float distance_resolution_;
  uint32_t point_index_;
  uint32_t dense_point_cnt_;
  uint8_t pre_frame_index_;

  static RSDecoderParam& getParam();
  static RSDecoderConstParam& getConstParam();

  std::vector<int16_t> loadCSV(const std::string& filename, size_t expected_rows = 96, size_t expected_cols = 240);
  std::shared_ptr<uint8_t> allocateImageBuffer(const size_t size);
  void copy_3of4_bytes(const uint8_t* src, uint8_t* dst, size_t total_src_bytes);
  double timestampToDouble(const AC2Timestamp& timestamp);
  float parseDistanceResolution(uint8_t resolution);
  bool processMipiData(const uint8_t* packet, size_t packet_size, uint32_t width, uint32_t height,
                       const std::shared_ptr<StereoImageMsg>& stereo_data);
  bool processImageData(const AC2MsopPkt& pkt, double& image_ts, const std::shared_ptr<uint8_t>& image_data);
  bool processPointCloudData(const AC2MsopPkt& pkt, uint32_t& point_index, uint32_t& dense_point_cnt, double frame_recv_ts = 0.0);
  bool init(const RSDecoderParam& param);

  bool loadAngleFiles(const std::string& base_path);
  bool parseAngleData(const uint8_t* input_bytes, size_t input_size, std::vector<int16_t>& output_vector);
  bool checkAngleData(const size_t& expected_size);
  bool checkCrc16X25(const uint8_t* src, const std::string& data_desc, size_t core_data_len);
  bool processAngleData(const uint8_t* src, size_t src_size);
  bool parseDeviceInfo(const uint8_t* src, size_t src_size);
  double getArmRealtimeOffset();
  uint32_t getTofRingId(uint16_t pkt_seq, uint8_t tof_id, uint32_t tof_height);
  double getTofTimeOffset(uint16_t pkt_seq, uint8_t tof_id);
  std::array<uint8_t, kAngleSizePerPacket * kAnglePacketNum> raw_angle_array_;
  std::array<bool, kAnglePacketNum> difop_packet_received_;
  size_t received_count_;
};

template <typename T_PointCloud>
inline RSDecoderConstParam& DecoderRSAC2<T_PointCloud>::getConstParam()
{
  static RSDecoderConstParam param = {
    6448  // msop len
    ,
    1409  // difop len
    ,
    3  // msop id len
    ,
    4  // difop id len
    ,
    { 0x55, 0xAA, 0x5A }  // msop id
    ,
    { 0xA5, 0xFF, 0x00, 0x5A }  // difop id
    ,
    {},
    1  // laser number
    ,
    240  // blocks per packet
    ,
    1  // channels per block
    ,
    0.1f  // distance min
    ,
    12.0f  // distance max
    ,
    0.001f  // distance resolution
    ,
    80.0f  // initial value of temperature
  };

  return param;
}

template <typename T_PointCloud>
inline DecoderRSAC2<T_PointCloud>::DecoderRSAC2(const RSDecoderParam& param)
  : Decoder<T_PointCloud>(getConstParam(), param)
  , distance_resolution_(0.001)
  , point_index_(0U)
  , dense_point_cnt_(0U)
  , pre_frame_index_(0)
  , raw_angle_array_({ 0 })
  , difop_packet_received_({ false })
  , received_count_(0)
{
  init(param);
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::init(const RSDecoderParam& param)
{
  constexpr double kFrameDuration = 0.10;
  constexpr size_t kSinglePktNum = 192;
  this->packet_duration_ = kFrameDuration / kSinglePktNum;

  if (!loadAngleFiles(param.angle_path))
  {
    RS_ERROR << "Failed to load angle files." << RS_REND;
  }
#if defined(ENABLE_AVX2)
  RS_DEBUG << "ENABLE_AVX2" << RS_REND;
#elif defined(ENABLE_SSSE3)
  RS_DEBUG << "ENABLE_SSSE3" << RS_REND;
#elif defined(ENABLE_ARM64)
  RS_DEBUG << "ENABLE_ARM64" << RS_REND;
#elif defined(ENABLE_ARM_NEON)
  RS_DEBUG << "ENABLE_ARM_NEON" << RS_REND;
#else
  RS_DEBUG << "No SIMD acceleration enabled" << RS_REND;
#endif

  return true;
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::loadAngleFiles(const std::string& base_path)
{
  if (!this->param_.config_from_file)
  {
    return true;
  }
  const std::vector<std::string> csv_filenames = { "vector_dec_x_up.csv", "vector_dec_x_down.csv",
                                                   "vector_dec_y_up.csv", "vector_dec_y_down.csv",
                                                   "vector_dec_z_up.csv", "vector_dec_z_down.csv" };
  std::vector<int16_t> vector_x_up_array;
  std::vector<int16_t> vector_y_up_array;
  std::vector<int16_t> vector_z_up_array;
  std::vector<int16_t> vector_x_down_array;
  std::vector<int16_t> vector_y_down_array;
  std::vector<int16_t> vector_z_down_array;
  std::vector<std::vector<int16_t>*> target_vectors = {
    &vector_x_up_array,   &vector_x_down_array, &vector_y_up_array,
    &vector_y_down_array, &vector_z_up_array,   &vector_z_down_array
  };

  for (size_t i = 0; i < csv_filenames.size(); ++i)
  {
    std::string full_path = base_path + csv_filenames[i];
    try
    {
      *target_vectors[i] = loadCSV(full_path);
    }
    catch (const std::exception& e)
    {
      RS_ERROR << "Error loading CSV: " << e.what() << RS_REND;
      return false;
    }
  }
  tof_vector_x_array_.insert(tof_vector_x_array_.end(), vector_x_up_array.begin(), vector_x_up_array.end());
  tof_vector_x_array_.insert(tof_vector_x_array_.end(), vector_x_down_array.begin(), vector_x_down_array.end());
  tof_vector_y_array_.insert(tof_vector_y_array_.end(), vector_y_up_array.begin(), vector_y_up_array.end());
  tof_vector_y_array_.insert(tof_vector_y_array_.end(), vector_y_down_array.begin(), vector_y_down_array.end());
  tof_vector_z_array_.insert(tof_vector_z_array_.end(), vector_z_up_array.begin(), vector_z_up_array.end());
  tof_vector_z_array_.insert(tof_vector_z_array_.end(), vector_z_down_array.begin(), vector_z_down_array.end());

  if (!checkAngleData(kTotalPixels))
  {
    return false;
  }
  RS_INFO << "Angle data loaded from file successfully." << RS_REND;
  this->angles_ready_ = true;
  return true;
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::checkAngleData(const size_t& expected_size)
{
  return tof_vector_x_array_.size() == expected_size && tof_vector_y_array_.size() == expected_size &&
         tof_vector_z_array_.size() == expected_size;
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::parseAngleData(const uint8_t* input_bytes, size_t input_size,
                                                       std::vector<int16_t>& output_vector)
{
  if (input_bytes == nullptr || input_size % 4 != 0)
  {
    RS_ERROR << "Invalid input parameters: nullptr or size not divisible by 4." << RS_REND;
    return false;
  }

  const size_t output_size = input_size / 2;
  output_vector.clear();
  output_vector.resize(output_size);

  for (size_t i = 0; i < input_size; i += 4)
  {
    size_t output_index = i / 2;
    uint16_t low_word = static_cast<uint16_t>(input_bytes[i + 1]) | (static_cast<uint16_t>(input_bytes[i]) << 8);

    output_vector[output_index] = *reinterpret_cast<int16_t*>(&low_word);

    uint16_t high_word = static_cast<uint16_t>(input_bytes[i + 3]) | (static_cast<uint16_t>(input_bytes[i + 2]) << 8);

    output_vector[output_index + 1] = *reinterpret_cast<int16_t*>(&high_word);
  }

  return true;
}

template <typename T_PointCloud>
inline std::vector<int16_t> DecoderRSAC2<T_PointCloud>::loadCSV(const std::string& filename, size_t expected_rows,
                                                                size_t expected_cols)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("failed to open file: " + filename);
  }

  std::vector<int16_t> data;
  std::string line;
  size_t row_count = 0;

  while (std::getline(file, line) && row_count < expected_rows)
  {
    std::stringstream ss(line);
    std::string cell;
    size_t col_count = 0;
    while (std::getline(ss, cell, ','))
    {
      try
      {
        int16_t value = static_cast<int16_t>(std::stoi(cell));
        data.push_back(value);
      }
      catch (const std::invalid_argument& e)
      {
        throw std::runtime_error("row " + std::to_string(row_count + 1) + " includes non-numeric value: " + cell);
      }
      catch (const std::out_of_range& e)
      {
        throw std::runtime_error("row " + std::to_string(row_count + 1) + " cell out of range: " + cell);
      }
      ++col_count;
    }
    if (col_count != expected_cols)
    {
      throw std::runtime_error("row " + std::to_string(row_count + 1) + " size does not match (expected: " +
                               std::to_string(expected_cols) + ", actual: " + std::to_string(col_count) + ")");
    }
    ++row_count;
  }

  if (row_count != expected_rows)
  {
    throw std::runtime_error("rows (expected: " + std::to_string(expected_rows) +
                             ", actual: " + std::to_string(row_count) + ")");
  }

  return data;
}

template <typename T_PointCloud>
inline double DecoderRSAC2<T_PointCloud>::timestampToDouble(const AC2Timestamp& timestamp)
{
  uint16_t sh = ntohs(timestamp.SecondsHigh);
  uint32_t sl = ntohl(timestamp.SecondsLow);
  uint32_t ns_raw = ntohl(timestamp.NanoSeconds);

  uint64_t sec = ((uint64_t)sh << 34) | ((uint64_t)sl << 2) | ((uint64_t)(ns_raw & 0xC0000000) >> 30);

  uint32_t ns = ns_raw & 0x3FFFFFFF;

  double ts = sec + ns * 1e-9;
  return ts;
}

template <typename T_PointCloud>
inline float DecoderRSAC2<T_PointCloud>::parseDistanceResolution(uint8_t resolution)
{
  switch (resolution)
  {
    case 0x00:
      return 0.001f;
    case 0x01:
      return 0.0005f;
    case 0x02:
      return 0.00025f;
    case 0x03:
      return 0.000125f;
    case 0x11:
      return 0.002f;
    case 0x12:
      return 0.004f;
    case 0x13:
      return 0.005f;
    default:
      return 0.001f;
  }
}

template <typename T_PointCloud>
inline void DecoderRSAC2<T_PointCloud>::copy_3of4_bytes(const uint8_t* src, uint8_t* dst, size_t total_src_bytes)
{
  copy_3of4_bytes_optimized(src, dst, total_src_bytes);
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::processImageData(const AC2MsopPkt& pkt, double& image_ts,
                                                         const std::shared_ptr<uint8_t>& image_data)
{
  const uint16_t pkt_seq = ntohs(pkt.header.pkt_seq) - 1;
  if (pkt_seq >= kImageHeight)
  {
    RS_WARNING << "image pkt_seq out of range: " << pkt_seq << RS_REND;
    return false;
  }

  if (nullptr == image_data)
  {
    RS_ERROR << "image_data is nullptr." << RS_REND;
    return false;
  }

  if (pkt_seq == 0)
  {
    if (this->param_.use_lidar_clock)
    {
      image_ts = timestampToDouble(pkt.header.timestamp);
    }
    else
    {
      image_ts = getTimeHost() * 1e-6;
    }
  }

  uint8_t* dst_row = image_data.get() + pkt_seq * (kImageWidth * kBytesPerPixelRgb);

  copy_3of4_bytes(reinterpret_cast<const uint8_t*>(&pkt.pixels), dst_row, kImageWidth * kSrcPixelSize);
  return true;
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::processPointCloudData(const AC2MsopPkt& pkt, uint32_t& point_index,
                                                       uint32_t& dense_point_cnt, double frame_recv_ts)
{
  constexpr float kScale = 1.0f / 32768;
  constexpr float kOffset = 0.0058f;
  
  const uint16_t pkt_seq = ntohs(pkt.header.pkt_seq) - 1;
  if (pkt_seq >= kTofHeight)
  {
    RS_WARNING << "Invalid point cloud packet sequence: " << pkt_seq << RS_REND;
    return false;
  }

  double pkt_ts = 0.0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = timestampToDouble(pkt.header.timestamp);
  }
  else
  {
    pkt_ts = frame_recv_ts + getTofTimeOffset(pkt_seq, pkt.header.tof_id);
  }

  const auto& vec_x = tof_vector_x_array_;
  const auto& vec_y = tof_vector_y_array_;
  const auto& vec_z = tof_vector_z_array_;

  const size_t max_kPointNums = this->point_cloud_->points.size();
  const uint32_t ring_id = getTofRingId(pkt_seq, pkt.header.tof_id, kTofHeight);
  if (ring_id == 0)
  {
    this->first_point_ts_ = pkt_ts;
  }
  if (ring_id == (kPointCloudHeight - 1))
  {
    this->prev_point_ts_ = pkt_ts;
  }
  
  const float z_offset = (pkt.header.tof_id == 0x0) ? kOffset : -kOffset;
  const float distance_resolution = parseDistanceResolution(pkt.header.distance_resolution);
  bool ret = true;
  size_t base_index = ring_id * kPointCloudWidth;
  for (size_t pixel_idx = 0; pixel_idx < kPointCloudWidth; ++pixel_idx)
  {
    const AC2Pixel& pixel = pkt.pixels[pixel_idx];
    for (size_t wave_idx = 0; wave_idx < kPointCloudWaveNum; ++wave_idx)
    {
      const AC2Wave& wave = pixel.wave[wave_idx];
      uint16_t radius = ntohs(wave.radius);
      float distance = radius * distance_resolution;

      if (point_index < max_kPointNums)
      {
        auto& point = this->point_cloud_->points[point_index];
        if (this->distance_section_.in(distance))
        {
          const float scaled_dist = distance * kScale;
          int16_t vector_x = vec_x[base_index + pixel_idx];
          int16_t vector_y = vec_y[base_index + pixel_idx];
          int16_t vector_z = vec_z[base_index + pixel_idx];

          auto x = vector_x * scaled_dist;
          auto y = vector_y * scaled_dist;
          auto z = vector_z * scaled_dist + z_offset;

          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, wave.intensity);
          setTimestamp(point, pkt_ts);
          setRing(point, ring_id);
          ++dense_point_cnt;
          ++point_index;
        }
        else if (!this->param_.dense_points)
        {
          setX(point, NAN);
          setY(point, NAN);
          setZ(point, NAN);
          setIntensity(point, 0);
          setTimestamp(point, pkt_ts);
          setRing(point, ring_id);
          ++point_index;
        }
      }
      else
      {
        RS_WARNING << "point_index out of range: " << point_index << RS_REND;
        ret = false;
        break;
      }
    }
  }
  return ret;
}

template <typename T_PointCloud>
inline uint32_t DecoderRSAC2<T_PointCloud>::getTofRingId(uint16_t pkt_seq, uint8_t tof_id, uint32_t tof_height) {
  const uint32_t sub_idx = pkt_seq & 0x7U;
  const uint32_t base = pkt_seq - sub_idx;  

  uint32_t row_idx = 0;
  if(sub_idx < 4)
  {
    row_idx = base + (sub_idx << 1);
  }
  else
  {
    row_idx =  base + ((sub_idx - 4U) << 1) + 1U;
  }
  return tof_id == 0 ? row_idx : (row_idx + tof_height);
}

template <typename T_PointCloud>
inline double DecoderRSAC2<T_PointCloud>::getTofTimeOffset(uint16_t pkt_seq, uint8_t tof_id) {
  constexpr double kTof0BaseOffset = -0.0065;
  constexpr double kTof1BaseOffset = -0.004487;
  constexpr double kTofPktTimeIntervalSec = 0.004027;

  const uint32_t group_idx = pkt_seq >> 2;
  const double base_offset = tof_id == 0 ? kTof0BaseOffset : kTof1BaseOffset;
  const double final_time_offset = base_offset + group_idx * kTofPktTimeIntervalSec;

  return final_time_offset;
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::checkCrc16X25(const uint8_t* src, const std::string& data_desc, size_t data_len)
{
  uint16_t actual_crc =
      calculateCrc16X25(kCrc16InitValue, kSwCrcDataWidth8Bit, src, static_cast<uint32_t>(data_len - 2));
  uint16_t expected_crc = static_cast<uint16_t>(src[data_len - 2] << 8) | src[data_len - 1];

  if (actual_crc != expected_crc)
  {
    RS_ERROR << data_desc << " crc check failed. actual_crc: 0x" << std::hex << actual_crc << ", expected_crc: 0x"
             << expected_crc << std::dec << RS_REND;
    return false;
  }
  else
  {
    RS_INFO << data_desc << " crc check passed, actual_crc: 0x" << std::hex << actual_crc << ", expected_crc: 0x"
            << expected_crc << std::dec << RS_REND;
    return true;
  }
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::processAngleData(const uint8_t* src, size_t src_size)
{
  if (this->angles_ready_)
  {
    return true;
  }
  if (!src || src_size < kAngleDataTotalLen)
  {
    RS_ERROR << "Invalid angle data size: " << src_size << RS_REND;
    return false;
  }
  if (!checkCrc16X25(src, "Angle data", kAngleDataTotalLen))
  {
    return false;
  }

  bool ret = parseAngleData(src, kSingleAngleDataBytes, tof_vector_x_array_);
  if (!ret)
  {
    RS_ERROR << "Failed to parse x vector angle data." << RS_REND;
    return false;
  }
  ret = parseAngleData(src + kSingleAngleDataBytes, kSingleAngleDataBytes, tof_vector_y_array_);
  if (!ret)
  {
    RS_ERROR << "Failed to parse y vector angle data." << RS_REND;
    return false;
  }
  ret = parseAngleData(src + kSingleAngleDataBytes * 2, kSingleAngleDataBytes, tof_vector_z_array_);
  if (!ret)
  {
    RS_ERROR << "Failed to parse z vector angle data." << RS_REND;
    return false;
  }
  RS_INFO << "Angle data processed successfully." << RS_REND;
  this->angles_ready_ = true;
  return ret;
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::parseDeviceInfo(const uint8_t* src, size_t src_size)
{
  // Constants
  constexpr uint8_t kEndMarker = 0xFF;
  constexpr size_t kMinBufferSize = 2;      // Minimum buffer for one parameter (name_len + end_marker)

  // Check CRC only on first parse
  if (this->device_info_.state)
  {
    return true;
  }
  // Early validation
  if (!src || src_size < kMinBufferSize)
  {
    RS_ERROR << "Invalid device info buffer: null or too small, size=" << src_size << RS_REND;
    return false;
  }

  if (!checkCrc16X25(src, "Device info", src_size))
  {
    RS_ERROR << "Device info CRC check failed" << RS_REND;
    return false;
  }

  // Lambda function to read variable-length string field
  auto readStringField = [&](const uint8_t* buf, size_t remaining_size, size_t& bytes_consumed,
                             const std::string& field_type) -> std::string {
    bytes_consumed = 0;
    
    // Check buffer validity
    if (!buf || remaining_size < 1)
    {
      RS_ERROR << field_type << " buffer is null or too short" << RS_REND;
      return "";
    }

    uint8_t str_len = buf[0];

    if (str_len == kEndMarker || str_len == 0)
    {
      bytes_consumed = 1;
      return "";
    }

    // Check buffer has enough space for string content
    if (remaining_size < static_cast<size_t>(1 + str_len))
    {
      RS_ERROR << field_type << " buffer underflow, need " << (1 + str_len) << " bytes, only " << remaining_size
               << " available" << RS_REND;
      return "";
    }

    bytes_consumed = 1 + str_len;

    // Create string from buffer
    return std::string(reinterpret_cast<const char*>(buf + 1), str_len);
  };

  // Lambda function to safely convert string to double
  auto safeStringToDouble = [&](const std::string& str, size_t param_index, const std::string& param_name,
                                double& out_val) -> bool {
    if (str.empty())
    {
      RS_ERROR << "Param " << param_index << " (" << param_name << ") value is empty" << RS_REND;
      return false;
    }

    try
    {
      out_val = std::stod(str);
      return true;
    }
    catch (const std::invalid_argument& e)
    {
      RS_ERROR << "Param " << param_index << " (" << param_name 
               << ") value conversion failed: \"" << str << "\", error: " 
               << e.what() << RS_REND;
    }
    catch (const std::out_of_range& e)
    {
      RS_ERROR << "Param " << param_index << " (" << param_name 
               << ") value out of range: \"" << str << "\", error: " 
               << e.what() << RS_REND;
    }
    return false;
  };


  // Clear existing parameters
  this->device_info_.calib_params.clear();
  size_t offset = 0;
  size_t param_index = 0;
  std::string temp_json_str;

  // Parse parameter pairs until end marker or end of buffer
  while (offset < src_size)
  {
    // Parse parameter name
    size_t name_bytes = 0;
    std::string param_name = readStringField(src + offset, src_size - offset, name_bytes, "param name");

    // bytes_consumed == 0 indicates parsing error
    if (name_bytes == 0)
    {
        RS_ERROR << "Failed to parse parameter name at offset " << offset << RS_REND;
        this->device_info_.calib_params.clear();
        return false;
    }

    offset += name_bytes;

    // Check for end marker (empty string from readStringField indicates end marker)
    if (name_bytes == 1)
    {
      break;
    }

    // Check if buffer has space for value
    if (offset >= src_size)
    {
      RS_ERROR << "Unexpected end of data after parameter name: " << param_name << RS_REND;
      this->device_info_.calib_params.clear();
      return false;
    }

    // Parse parameter value
    size_t value_bytes = 0;
    std::string value_str = readStringField(src + offset, src_size - offset, value_bytes, "param value");

    if (value_bytes == 0)
    {
      RS_ERROR << "Failed to parse parameter value for: " << param_name << RS_REND;
      this->device_info_.calib_params.clear();
      return false;
    }

    offset += value_bytes;
    if (value_bytes == 1 && value_str.empty())
    {
        RS_ERROR << "Unexpected END marker in parameter value for: " << param_name << RS_REND;
        this->device_info_.calib_params.clear();
        return false;
    }
    // Handle special parameter: DEVICE_ID
    if (param_name == "DEVICE_ID")
    {
      this->device_info_.device_id = value_str;
      continue;
    }

    // Convert string to double for regular parameters
    double param_value = 0.0;
    if (!safeStringToDouble(value_str, param_index, param_name, param_value))
    {
      this->device_info_.calib_params.clear();
      return false;
    }

    // Store parameter
    this->device_info_.calib_params[param_name] = param_value;

    param_index++;

    if (!temp_json_str.empty())
    {
      temp_json_str += ",";
    }
  
    temp_json_str += "\"";
    temp_json_str += param_name;
    temp_json_str += "\":";
    temp_json_str += "\"";
    temp_json_str += value_str;
    temp_json_str += "\"";
  }
  if (!temp_json_str.empty())
  {
    this->device_info_.calib_params_str = "{" + temp_json_str + "}";
  }
  this->device_info_.state = true;
  
  return true;
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::processMipiData(const uint8_t* packet, size_t packet_size, uint32_t width,
                                                 uint32_t height, const std::shared_ptr<StereoImageMsg>& stereo_data)
{
  constexpr size_t kMipiWidth = 6464;
  constexpr size_t kMipiHeight = (kImageHeight * 2 + kPointCloudHeight);
  constexpr size_t kMipiLen = (kMipiWidth * kMipiHeight);
  constexpr size_t kAngleDataHeight = 43;
  constexpr size_t kAngleDataOffset = kMipiWidth * kAngleDataHeight;
  constexpr size_t kAngleDataAndDeviceInfoHeight = 2636;
  constexpr size_t kRgbImageSize = kImageWidth * kImageHeight * kBytesPerPixelRgb;

  const uint8_t kHeaderIdLen = this->const_param_.MSOP_ID_LEN;
  const uint8_t* kHeaderId = this->const_param_.MSOP_ID;
  if (memcmp(packet, kHeaderId, kHeaderIdLen) != 0)
  {
    RS_WARNING << "Header mismatch at row " << 0 << ": expected 0x55AA5A, got 0x" << std::hex
               << static_cast<int>(packet[0]) << static_cast<int>(packet[1]) << static_cast<int>(packet[2]) << std::dec
               << RS_REND;
    return false;
  }

  stereo_data->data_bytes = kRgbImageSize;

  if (this->param_.image_mode == 0 || this->param_.image_mode == 1)
  {
    stereo_data->left_data = allocateImageBuffer(kRgbImageSize);
    if (!stereo_data->left_data)
    {
      RS_ERROR << "Failed to allocate left image buffer (size=" << kRgbImageSize << ")" << RS_REND;
      return false;
    }
  }

  if (this->param_.image_mode == 0 || this->param_.image_mode == 2)
  {
    stereo_data->right_data = allocateImageBuffer(kRgbImageSize);
    if (!stereo_data->right_data)
    {
      RS_ERROR << "Failed to allocate right image buffer (size=" << kRgbImageSize << ")" << RS_REND;
      return false;
    }
  }

  this->point_cloud_->points.resize(kPointNums);
  const size_t src_row_stride = width * kSrcPixelSize;

  uint32_t point_index = 0;
  uint32_t dense_point_cnt = 0;
  if (packet_size < height * src_row_stride)
  {
    RS_ERROR << "Invalid packet size: " << packet_size << RS_REND;
    return false;
  }
  if (src_row_stride < sizeof(AC2MsopPkt))
  {
    RS_ERROR << "Invalid src_row_stride: " << src_row_stride << RS_REND;
    return false;
  }

  if (height >= kAngleDataAndDeviceInfoHeight)
  {
    const uint8_t* angle_data = packet + kMipiLen;
    processAngleData(angle_data, kAngleDataTotalLen);

    const uint8_t* device_info = angle_data + kAngleDataOffset;
    parseDeviceInfo(device_info, kMipiWidth);
  }

  bool point_cloud_valid = true;
  bool image_valid = true;
  bool is_angle_data_ready = true;
  if (!checkAngleData(kTotalPixels))
  {
    RS_ERROR << "Angle data is not ready!." << RS_REND;
    is_angle_data_ready = false;
    return false;
  }
  double recv_time = getTimeHost() * 1e-6;
  const size_t MAX_ROW = (size_t)(height > kMipiHeight ? kMipiHeight : height);
  for (size_t row = 0; row < MAX_ROW; ++row)
  {
    const uint8_t* src_row = packet + row * src_row_stride;
    if (memcmp(src_row, kHeaderId, kHeaderIdLen) != 0)
    {
      RS_WARNING << "Header mismatch at row " << row << ": expected 0x55AA5A, got 0x" << std::hex
                 << static_cast<int>(src_row[0]) << static_cast<int>(src_row[1]) << static_cast<int>(src_row[2])
                 << std::dec << RS_REND;
      break;
    }
    const AC2MsopPkt& pkt = *reinterpret_cast<const AC2MsopPkt*>(src_row);
    const uint8_t data_type = pkt.header.id[3];

    switch (data_type)
    {
      case kCmos1Id:  // right image
        if (this->param_.image_mode == 0 || this->param_.image_mode == 2)
        {
          image_valid = processImageData(pkt, stereo_data->right_timestamp, stereo_data->right_data);
        }
        break;

      case kCmos2Id:  // left image
        if (this->param_.image_mode == 0 || this->param_.image_mode == 1)
        {
          image_valid = processImageData(pkt, stereo_data->left_timestamp, stereo_data->left_data);
        }
        break;

      case kTof1Id:
      case kTof2Id:
        if (this->param_.enable_point_cloud && is_angle_data_ready)
        {
          point_cloud_valid = processPointCloudData(pkt, point_index, dense_point_cnt, recv_time);
        }
        break;

      default:
        break;
    }
  }

  if (image_valid)
  {
    stereo_data->state = image_valid;
    stereo_data->timestamp = stereo_data->left_timestamp;
    stereo_data->width = kImageWidth;
    stereo_data->height = kImageHeight;
    this->cb_image_data_();
  }

  if (point_cloud_valid)
  {
    if (this->param_.dense_points)
    {
      this->point_cloud_->points.resize(dense_point_cnt);
    }
    this->cb_split_frame_(kPointCloudHeight, this->cloudTs());
  }
  return point_cloud_valid || image_valid;
}

template <typename T_PointCloud>
inline double DecoderRSAC2<T_PointCloud>::getArmRealtimeOffset()
{
#if defined _WIN32
  return 0.0;
#elif defined(__aarch64__)
  unsigned long cycles, frq;

  asm volatile("mrs %0, cntfrq_el0" : "=r"(frq));
  asm volatile("mrs %0, cntvct_el0" : "=r"(cycles));

  if (frq == 0)
    return 0.0;
  unsigned long long tsc_ns = (cycles * 100 / (frq / 10000)) * 1000;
  double arm_time = tsc_ns / 1e9;

  struct timespec real_now;
  clock_gettime(CLOCK_REALTIME, &real_now);
  double real_time = real_now.tv_sec + real_now.tv_nsec / 1e9;

  return real_time - arm_time;

#else
  struct timespec real_now, mono_now;
  clock_gettime(CLOCK_REALTIME, &real_now);
  clock_gettime(CLOCK_MONOTONIC, &mono_now);

  double real_time = real_now.tv_sec + real_now.tv_nsec / 1e9;
  double mono_time = mono_now.tv_sec + mono_now.tv_nsec / 1e9;

  return real_time - mono_time;
#endif
}

template <typename T_PointCloud>
inline void DecoderRSAC2<T_PointCloud>::decodeImageFrame(const uint8_t* src_data, size_t data_len)
{
  if (!this->imageDataPtr_ || !this->cb_image_data_ || !src_data || data_len < kHeaderSize)
  {
    return;
  }

  auto stereo_data = std::dynamic_pointer_cast<StereoImageMsg>(this->imageDataPtr_);
  if (!stereo_data)
  {
    RS_WARNING << "ImageDataPtr is not of type StereoImageMsg as expected for FRAME_FORMAT_XR24" << RS_REND;
    return;
  }
  stereo_data->camera_mode = CameraMode::STEREO;
  stereo_data->state = false;
  memcpy(&stereo_data->frame_format, src_data + 2, 2);

  double timestamp = 0.0;
  uint32_t width = 0;
  uint32_t height = 0;

  memcpy(&timestamp, src_data + 4, 8);
  memcpy(&width, src_data + 12, 4);
  memcpy(&height, src_data + 16, 4);

  if (stereo_data->frame_format == frame_format_t::FRAME_FORMAT_XR24)
  {
    const size_t expected_data_size = width * height * 4;  // Size of 1 XR24 frame (4 channels)
    size_t data_size = data_len - kHeaderSize;
    if (data_size != expected_data_size)
    {
      stereo_data->state = false;
      return;
    }

    stereo_data->sot_timestamp = timestamp;
    stereo_data->left_sot_timestamp = timestamp;
    stereo_data->right_sot_timestamp = timestamp;
    this->point_cloud_->sot_timestamp = timestamp;

#ifdef ENABLE_GMSL
    double sot_timestamp_rt = timestamp + getArmRealtimeOffset();
    stereo_data->sot_timestamp_rt = sot_timestamp_rt;
    stereo_data->left_sot_timestamp_rt = sot_timestamp_rt;
    stereo_data->right_sot_timestamp_rt = sot_timestamp_rt;
    this->point_cloud_->sot_timestamp_rt = sot_timestamp_rt;
#endif
    processMipiData(src_data + kHeaderSize, data_len - kHeaderSize, width, height, stereo_data);
  }
  return;
}

template <typename T_PointCloud>
inline std::shared_ptr<uint8_t> DecoderRSAC2<T_PointCloud>::allocateImageBuffer(const size_t size)
{
  if (size == 0)
  {
    RS_WARNING << "allocateImageBuffer: Invalid buffer size (0 bytes)" << RS_REND;
    return nullptr;
  }

  // Note: std::make_shared for arrays requires C++20; use new+default_delete for compatibility
  try
  {
    return std::shared_ptr<uint8_t>(new uint8_t[size], std::default_delete<uint8_t[]>());
  }
  catch (const std::bad_alloc& e)
  {
    RS_ERROR << "allocateImageBuffer: Memory allocation failed (size=" << size << ", error=" << e.what() << ")"
             << RS_REND;
    return nullptr;
  }
}
template <typename T_PointCloud>
inline void DecoderRSAC2<T_PointCloud>::decodeImuFrame(const uint8_t* src_data, size_t data_len)
{
  constexpr float kAccelScaleFactor = 32768.0f / 16.0f;
  constexpr float kGyroScaleFactor = 32768.0f / 2000.0f;
  constexpr float kGravity = 9.80665f;
  constexpr float kRadPerDeg = M_PI / 180.0f;
  constexpr float kAccelConversion = kGravity / kAccelScaleFactor;
  constexpr float kGyroConversion = kRadPerDeg / kGyroScaleFactor;

  if (this->imuDataPtr_ && this->cb_imu_data_ && this->param_.enable_imu)
  {
    const size_t IMU_PKT_SIZE = sizeof(AC2ImuPkt);

    if (data_len == IMU_PKT_SIZE)
    {
      const AC2ImuPkt& pkt = *reinterpret_cast<const AC2ImuPkt*>(src_data);

      this->imuDataPtr_->linear_acceleration_x = -pkt.accel_z * kAccelConversion;
      this->imuDataPtr_->linear_acceleration_y = -pkt.accel_x * kAccelConversion;
      this->imuDataPtr_->linear_acceleration_z = pkt.accel_y * kAccelConversion;

      // Parse gyroscope data
      this->imuDataPtr_->angular_velocity_x = -pkt.gyro_z * kGyroConversion;
      this->imuDataPtr_->angular_velocity_y = -pkt.gyro_x * kGyroConversion;
      this->imuDataPtr_->angular_velocity_z = pkt.gyro_y * kGyroConversion;
      if (this->param_.use_lidar_clock)
      {
        this->imuDataPtr_->timestamp = pkt.timestamp_sec + pkt.timestamp_nsec * 1e-9;
      }
      else
      {
        this->imuDataPtr_->timestamp = getTimeHost() * 1e-6;
      }
      this->imuDataPtr_->state = true;
      this->cb_imu_data_();
    }
  }
}

template <typename T_PointCloud>
inline void DecoderRSAC2<T_PointCloud>::decodePcFrame(const uint8_t* src_data, size_t data_len)
{
}

template <typename T_PointCloud>
inline bool DecoderRSAC2<T_PointCloud>::decodeMsopPkt(const uint8_t* packet, size_t size)
{
  bool ret = false;
  if (this->param_.enable_point_cloud)
  {
    if (!checkAngleData(kTotalPixels))
    {
      return false;
    }
    const size_t msop_pkt_size = sizeof(AC2MsopPkt);
    if (size == msop_pkt_size)
    {
      const AC2MsopPkt& pkt = *reinterpret_cast<const AC2MsopPkt*>(packet);

      const uint8_t data_type = pkt.header.id[3];
      if (data_type != kTof1Id && data_type != kTof2Id)
      {
        return false;
      }

      if (this->point_cloud_->points.empty())
      {
        this->point_cloud_->points.resize(kPointNums);
      }
      if (pre_frame_index_ != pkt.header.frame_cnt)
      {
        pre_frame_index_ = pkt.header.frame_cnt;
        if (this->param_.dense_points)
        {
          this->point_cloud_->points.resize(dense_point_cnt_);
        }
        this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
        this->point_cloud_->points.resize(kPointNums);
        dense_point_cnt_ = 0;
        point_index_ = 0;
        ret = true;
      }
      double recv_time = getTimeHost() * 1e-6;
      processPointCloudData(pkt, point_index_, dense_point_cnt_, recv_time);
    }
  }
  return ret;
}

template <typename T_PointCloud>
inline void DecoderRSAC2<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const size_t DIFOOP_PKT_SIZE = sizeof(AC2DifopPkt);
  if (size == DIFOOP_PKT_SIZE)
  {
    const AC2DifopPkt& pkt = *reinterpret_cast<const AC2DifopPkt*>(packet);
    uint8_t pkt_seq = pkt.pkt_seq - 1;
    if (pkt_seq < kAnglePacketNum)
    {
      if (!this->angles_ready_ && !this->param_.config_from_file)
      {
        size_t offset = pkt_seq * kAngleSizePerPacket;
        std::copy(pkt.data, pkt.data + kAngleSizePerPacket, raw_angle_array_.data() + offset);
        if (!difop_packet_received_[pkt_seq])
        {
          difop_packet_received_[pkt_seq] = true;
          received_count_++;
        }
        if (received_count_ == kAnglePacketNum)
        {
          this->angles_ready_ = processAngleData(raw_angle_array_.data(), raw_angle_array_.size());
          if (!this->angles_ready_)
          {
            // reset received_count_ and difop_packet_received_ if processing fails
            received_count_ = 0;
            std::fill(difop_packet_received_.begin(), difop_packet_received_.end(), false);
          }
        }
      }
    }
  }
}

}  // namespace lidar
}  // namespace robosense
