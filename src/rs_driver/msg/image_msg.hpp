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

#include <vector>
#include <string>
#include <memory>

namespace robosense
{
namespace lidar
{

typedef enum frame_format
{
  FRAME_FORMAT_ANY = 0,           /**< Any supported format */
  FRAME_FORMAT_NV12,              /**< YUV420: NV12 format */
  FRAME_FORMAT_BGR24,
  FRAME_FORMAT_RGB24,
  FRAME_FORMAT_YUV422,
  FRAME_FORMAT_XR24,
  FRAME_FORMAT_GREY,
} frame_format_t;

enum class CameraMode {
  MONO = 0,   
  STEREO,
};
class ImageMsg {
  public:
      bool state = false;                  // State of the image
      uint32_t data_bytes = 0;             // Bytes of the image data
      uint32_t width = 0;                  // Width of the image
      uint32_t height = 0;                 // Height of the image
      frame_format_t frame_format;         // Format of the image
      double timestamp = 0.0;              // Timestamp of the image
      double sot_timestamp = 0.0;          // Timestamp when data was received
      double sot_timestamp_rt = 0.0;      // Timestamp when data was received in real time

      CameraMode camera_mode = CameraMode::MONO; 
      virtual ~ImageMsg() = default;
  
  };
  
  class MonoImageMsg : public ImageMsg {
  public:
      std::shared_ptr<uint8_t> data;      
  };
  class StereoImageMsg : public ImageMsg {
  public:
      double left_timestamp = 0.0;  // Timestamp of the left image
      double right_timestamp = 0.0; // Timestamp of the right image
      double left_sot_timestamp = 0.0;  // Timestamp when data was received of the left image
      double left_sot_timestamp_rt = 0.0;  // Timestamp when data was received in real time of the left image
      double right_sot_timestamp = 0.0; // Timestamp when data was received of the right image
      double right_sot_timestamp_rt = 0.0; // Timestamp when data was received in real time of the right image
      std::shared_ptr<uint8_t> left_data;  // Data of the left image
      std::shared_ptr<uint8_t> right_data; // Data of the right image
  };
  
} // namespace lidar
} // namespace robosense


