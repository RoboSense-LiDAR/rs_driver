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
} frame_format_t;


class ImageData {
public:
  bool state = false;
  std::shared_ptr<uint8_t> data;
  uint32_t data_bytes;                                /**< Size of the image data buffer in bytes */
  uint32_t width = 0;                                 /**< Width of the image in pixels */
  uint32_t height = 0;                                /**< Height of the image in pixels */
  frame_format_t frame_format = FRAME_FORMAT_ANY;     /**< Format of the pixel data */
  // uint32_t sequence = 0;                           /**< Frame sequence number */
  double timestamp = 0.0;
};

} // namespace lidar
} // namespace robosense