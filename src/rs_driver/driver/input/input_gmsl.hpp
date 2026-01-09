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

#include <sstream>

#ifdef _WIN32
// not support on windows
#else  //__linux__
#include <rs_driver/driver/decoder/basic_attr.hpp>
#include <rs_driver/driver/input/input.hpp>
#include <rs_driver/driver/input/gmsl/gmsl_capture.hpp>
namespace robosense
{
namespace lidar
{
using namespace video;
class InputGMSL : public Input
{
public:
  InputGMSL(const RSInputParam& input_param) : Input(input_param)
  {
  }

  virtual bool init();
  virtual bool start();
  virtual void stop() override;
  virtual ~InputGMSL();

private:
  void recvPacket(void* data, size_t size, double timestamp);

private:
  std::shared_ptr<GmslCapture> gmsl_capture_ptr_;
};

inline bool InputGMSL::init()
{
  if (init_flag_)
    return true;

  V4L2Params v4l2_params;

  v4l2_params.device_path = input_param_.device_path;
  v4l2_params.width = input_param_.image_width;
  v4l2_params.height = input_param_.image_height;

  gmsl_capture_ptr_ = std::make_shared<GmslCapture>(v4l2_params);

  if (!gmsl_capture_ptr_->init())
  {
    return false;
  }

  gmsl_capture_ptr_->setFrameCallback(
      std::bind(&InputGMSL::recvPacket, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  init_flag_ = true;
  return true;
}

inline bool InputGMSL::start()
{
  if (start_flag_)
    return true;

  if (!init_flag_)
  {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }
  if (!gmsl_capture_ptr_->start())
  {
    return false;
  }
  start_flag_ = true;
  return true;
}

inline InputGMSL::~InputGMSL()
{
  stop();
}

inline void InputGMSL::stop()
{
  if (!start_flag_)
    return;
  if (gmsl_capture_ptr_)
  {
    gmsl_capture_ptr_->stop();
  }
  RS_INFO << "InputGMSL::stop() called" << RS_REND;
  start_flag_ = false;
  return;
}

inline void InputGMSL::recvPacket(void* data, size_t size, double timestamp)
{
  std::shared_ptr<Buffer> pkt = cb_get_pkt_2_(size + 20);
  pkt->buf()[0] = 0xAA;
  pkt->buf()[1] = 0x66;
  pkt->buf()[2] = (uint8_t)FRAME_FORMAT_XR24;

#if 0
  static double last_timestamp = 0.0;
  double diff = timestamp - last_timestamp;
  if(last_timestamp != 0.0)
  {
    printf("intput timestamp:%f, diff:%f, fps:%f \n", timestamp, diff, 1.0/diff);
  }
  last_timestamp = timestamp;
#endif
  memcpy(&pkt->buf()[4], &timestamp, 8);
  int width = input_param_.image_width / 4;
  memcpy(&pkt->buf()[12], &width, 4);
  memcpy(&pkt->buf()[16], &input_param_.image_height, 4);
  memcpy(pkt->data() + 20, data, size);
  pkt->setData(0, size + 20);
  pushPacket2(pkt);
}

}  // namespace lidar
}  // namespace robosense

#endif