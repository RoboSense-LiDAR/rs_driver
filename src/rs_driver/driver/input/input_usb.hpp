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
#include <rs_driver/driver/input/input.hpp>

#include <sstream>

#ifdef _WIN32
#define WIN32
#else //__linux__
#endif

#ifdef ENABLE_USB
struct libusb_context;
struct libusb_device;
struct libusb_device_handle;

struct uvc_context;
struct uvc_device;
struct uvc_device_handle;
struct uvc_stream_ctrl;

namespace robosense
{
namespace lidar
{

typedef enum hid_req
{
    HID_REQ_IMU_UPLOAD_START = 0,         /**< Request to start the operation. */
    HID_REQ_IMU_UPLOAD_STOP,              /**< Request to stop the operation. */
    HID_REQ_SYNC,                         /**< Request to synchronize. */
    HID_REQ_DELAY_RESP,                   /**< Request for a delayed response. */
} hid_req_t;

typedef enum hid_resp
{
    HID_RESP_ERROR = 0,                   /**< Response indicating an error. */
    HID_RESP_HEAD_ERROR,
    HID_RESP_CRC_ERROR,
    HID_RESP_START_STOP,                  /**< Response for start/stop operation. */
    HID_RESP_SYNC,                        /**< Response for synchronization. */
    HID_RESP_DELAY,                       /**< Response for a delay request. */
    HID_RESP_IMU,                         /**< Response containing IMU data. */
    HID_RESP_OTHER
} hid_resp_t;

class InputUsb : public Input
{
public:
  InputUsb(const RSInputParam& input_param)
    : Input(input_param) 
  {
    _kill_handler_thread = 0;
    _usb_ctx = nullptr;
    _dev = nullptr;
    _devh = nullptr;

    _uvc_ctx_image = nullptr;
    _uvc_ctrl_image = nullptr;
    _uvc_dev_image = nullptr;
    _uvc_devh_image = nullptr;

    _uvc_ctx_pc = nullptr;
    _uvc_ctrl_pc = nullptr;
    _uvc_dev_pc = nullptr;
    _uvc_devh_pc = nullptr;
  }

  virtual bool init();
  virtual bool start();
  virtual void stop();
  virtual ~InputUsb();

private:
  void recvPacket();
  void usbEventHandler();
  bool findDevices(std::string uuid);
  bool imageStreamInit();
  bool imageStreamStart();
  void imageStreamClose();
  bool pcStreamInit();
  bool pcStreamStart();
  void pcStreamClose();
  bool imuStreamInit();
  bool imuStreamStart();
  void imuStreamClose();
  bool send_cmd(hid_req req);
  hid_resp parse_cmd(const uint8_t *data, int len);
  void creat_frame_header(uint8_t* data, int len);

private:
  std::shared_ptr<Buffer> _pkt_image;
  std::shared_ptr<Buffer> _pkt_pc;

  uint64_t _msec_to_delay_ = 1000000;
  
  int _kill_handler_thread = 1;

  std::thread _usb_thread;

  libusb_context *_usb_ctx;
  libusb_device *_dev;
  libusb_device_handle *_devh;

  uvc_context *_uvc_ctx_image;
  uvc_device *_uvc_dev_image;
  uvc_device_handle *_uvc_devh_image;
  struct uvc_stream_ctrl *_uvc_ctrl_image;

  uvc_context *_uvc_ctx_pc;
  uvc_device *_uvc_dev_pc;
  uvc_device_handle *_uvc_devh_pc;
  struct uvc_stream_ctrl *_uvc_ctrl_pc;

  int _hid_intface_num;
  int _hid_endpoint_in_num;
  int _hid_endpoint_out_num;

  int _width = 640;
  int _height = 480;
  int _fps = 30;
  int _frame_format = 0;

  bool _is_usb_300 = false;
};

}  // namespace lidar
}  // namespace robosense

#endif // ENABLE_USB