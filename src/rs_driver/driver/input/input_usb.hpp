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
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <atomic>

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
enum class HidReqType
{
    HID_REQ_IMU_UPLOAD_START_100HZ = 0,         /**< Request to start the operation. */
    HID_REQ_IMU_UPLOAD_START_200HZ,         /**< Request to start the operation. */
    HID_REQ_IMU_UPLOAD_STOP,              /**< Request to stop the operation. */
    HID_REQ_SYNC,                         /**< Request to synchronize. */
    HID_REQ_DELAY_RESP,                   /**< Request for a delayed response. */
};

enum class HidRespType
{
    HID_RESP_ERROR = 0,                   /**< Response indicating an error. */
    HID_RESP_HEAD_ERROR,
    HID_RESP_CRC_ERROR,
    HID_RESP_START_STOP,                  /**< Response for start/stop operation. */
    HID_RESP_SYNC,                        /**< Response for synchronization. */
    HID_RESP_DELAY,                       /**< Response for a delay request. */
    HID_RESP_IMU,                         /**< Response containing IMU data. */
    HID_RESP_OTHER
};

class InputUsb : public Input
{
public:
  InputUsb(const RSInputParam& input_param)
    : Input(input_param) 
  {
    kill_handler_thread_ = 0;
    usb_ctx_ = nullptr;
    dev_ = nullptr;
    devh_ = nullptr;

    uvc_ctx_image_ = nullptr;
    uvc_ctrl_image_ = nullptr;
    uvc_dev_image_ = nullptr;
    uvc_devh_image_ = nullptr;

    uvc_ctx_pc_ = nullptr;
    uvc_ctrl_pc_ = nullptr;
    uvc_dev_pc_ = nullptr;
    uvc_devh_pc_ = nullptr;
  }

  virtual bool init();
  virtual bool start();
  virtual void stop();
  virtual ~InputUsb();

  bool customCmd(const std::vector<uint8_t> &send, std::vector<uint8_t> &receive) override final;

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
  void hidStop();
  bool sendHidCmd(HidReqType type);
  HidRespType parseHidResp(const uint8_t *data, int len);
  void buildHidFrameHeader(uint8_t* data, int len);
  bool usbTransferTx(libusb_device_handle *devh, int endpoint_out, uint8_t *data, int len);
  void clearHidRecvCache();
  HidRespType usbTransferRx(libusb_device_handle *devh, int endpoint_in, uint8_t* data, int len, int &transfer_length, int time_out = 100);
private:

  libusb_context *usb_ctx_;
  libusb_device *dev_;
  libusb_device_handle *devh_;

  uvc_context *uvc_ctx_image_;
  uvc_device *uvc_dev_image_;
  uvc_device_handle *uvc_devh_image_;
  struct uvc_stream_ctrl *uvc_ctrl_image_;

  uvc_context *uvc_ctx_pc_;
  uvc_device *uvc_dev_pc_;
  uvc_device_handle *uvc_devh_pc_;
  struct uvc_stream_ctrl *uvc_ctrl_pc_;

  std::shared_ptr<Buffer> image_buf_ptr_{nullptr};
  std::shared_ptr<Buffer> pc_buf_ptr_{nullptr};
  
  std::thread usbEventProcThead_;
  int hid_intface_num_;
  int hid_endpoint_in_num_;
  int hid_endpoint_out_num_;

  int image_width_{640};
  int image_height_{480};
  int image_fps_{30};
  int image_frame_format_{0};

  bool is_usb_300_{false};
  int kill_handler_thread_{1};
  std::atomic<bool> hid_start_flag_{false};       
  std::atomic<bool> is_ac2_{false};            
  std::atomic<bool> transfer_custom_cmd_{false};
  std::condition_variable cond_;
  std::mutex mtx_;
};

}  // namespace lidar
}  // namespace robosense

#endif // ENABLE_USB