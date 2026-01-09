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
#include <rs_driver/driver/input/input_usb.hpp>

#ifdef ENABLE_USB
#include <rs_driver/utility/crc16.hpp>
#include <sstream>
#ifdef PLATFORM_WINDOWS_MSVC
#include <time.h>
#else
#include <sys/time.h>
#endif
#include <mutex>
#include <algorithm>
#include <chrono>
#include "libusb.h"
#include "libuvc/libuvc.h"

#define ROBOSENSE_VENDOR_ID 0x3840
#define RS_AC1_PRODUCT_ID 0x1010
#define RS_AC2_PRODUCT_ID 0x1020

#define ALIGN_UP(V_, ALIGN_) (((V_) + ((ALIGN_) - 1U)) & (~((ALIGN_) - 1U)))
#define RS_AC1_POINT_CLOUD_WIDTH 96U
#define RS_AC1_POINT_CLOUD_HEIGHT 288U
#define POINT_NUMS RS_AC1_POINT_CLOUD_WIDTH* RS_AC1_POINT_CLOUD_HEIGHT
#define RS_AC1_DEPTH_DATA_BYTES_BEFORE (10U + RS_AC1_POINT_CLOUD_WIDTH * 12U) * RS_AC1_POINT_CLOUD_HEIGHT
/**
 * Extra 10 bytes: TimeBase
 * Every pixel contains 12 bytes:
 *  TimeOffset, X, Y, Z, Distance, Reflectivity, Attribute
 */
#define RS_AC1_DEPTH_DATA_BYTES ALIGN_UP(RS_AC1_DEPTH_DATA_BYTES_BEFORE, 2048U)

#define USE_LIBUSB_RESET
constexpr int kMaxHidBufferSize = 1024;
constexpr uint8_t kHidFrameHead1 = 0xFE;
constexpr uint8_t kHidFrameHead2 = 0xAA;
constexpr int kFrameHeaderSize = 8;
constexpr int kCustomCmdTimeout = 1000;
constexpr int kClearRecvCacheTimes = 3;
constexpr int kMaxCustomCmdSize = kMaxHidBufferSize - kFrameHeaderSize;

namespace robosense
{
namespace lidar
{

bool InputUsb::init()
{
  int res = 0;
  if (init_flag_)
    return true;

  if (!usb_ctx_)
  {
    res = libusb_init(&usb_ctx_);
    if (res < 0)
    {
      RS_ERROR << "Failed to initialize libusb:" << libusb_strerror(res) << RS_REND;
      usb_ctx_ = nullptr;
      return false;
    }
  }

  if (findDevices(input_param_.device_uuid) == false)
  {
    RS_ERROR << "Device is not found, please check!" << RS_REND;
    return false;
  }

  if (!is_usb_300_ && !input_param_.enable_usb200)
  {
    RS_ERROR << "USB mode is not USB_3.0!" << RS_REND;
    return false;
  }

  kill_handler_thread_ = 0;
  usbEventProcThead_ = std::thread(std::bind(&InputUsb::usbEventHandler, this));

  res = libusb_open(dev_, &devh_);

  if (res != 0)
  {
    RS_ERROR << "Failed to open device: " << libusb_strerror(res) << RS_REND;
    return false;
  }

  if (imuStreamInit() == false)
  {
    RS_ERROR << "Depth stream init failed!" << RS_REND;
    return false;
  }

  if (input_param_.enable_image)
  {
    if (imageStreamInit() == false)
    {
      RS_ERROR << "Image stream init failed!" << RS_REND;
      return false;
    }
  }

  if (pcStreamInit() == false)
  {
    RS_ERROR << "Depth stream init failed!" << RS_REND;
    return false;
  }

#ifdef USE_LIBUSB_RESET
  RS_DEBUG << "[USE_LIBUSB_RESET=ON] Driver will call libusb_reset_device() on stop." << RS_REND;
#else
  RS_DEBUG << "[USE_LIBUSB_RESET=OFF] Driver will NOT call libusb_reset_device() on stop." << RS_REND;
#endif

  init_flag_ = true;
  return true;
}

bool InputUsb::start()
{
  if (start_flag_)
    return true;

  if (!init_flag_)
  {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  if (imuStreamStart() == false)
  {
    RS_ERROR << "Imu stream start failed!" << RS_REND;
    return false;
  }

  if (input_param_.enable_image)
  {
    if (imageStreamStart() == false)
    {
      RS_ERROR << "Image stream start failed!" << RS_REND;
      return false;
    }
  }

  if (pcStreamStart() == false)
  {
    RS_ERROR << "Depth stream start failed!" << RS_REND;
    return false;
  }

  start_flag_ = true;
  return true;
}

void InputUsb::stop()
{
  imageStreamClose();
  pcStreamClose();

  if (!to_exit_recv_)
  {
    to_exit_recv_ = true;
    if (recv_thread_.joinable())
    {
      recv_thread_.join();
    }
  }

  imuStreamClose();

  if (!kill_handler_thread_)
  {
    kill_handler_thread_ = 1;
    if (usbEventProcThead_.joinable())
    {
      usbEventProcThead_.join();
    }
  }

  if (devh_)
  {
#ifdef USE_LIBUSB_RESET
    int res = libusb_reset_device(devh_);
    if (res == 0)
    {
      RS_DEBUG << "Device reset successful" << RS_REND;
    }
    else
    {
      RS_ERROR << "Failed to reset device: " << libusb_error_name(res) << RS_REND;
    }
#endif

    libusb_close(devh_);
    devh_ = nullptr;
  }

  if (dev_)
  {
    libusb_unref_device(dev_);
    dev_ = nullptr;
  }

  if (usb_ctx_)
  {
    libusb_exit(usb_ctx_);
    usb_ctx_ = nullptr;
  }
}

InputUsb::~InputUsb()
{
  stop();
}

void InputUsb::usbEventHandler()
{
  struct timeval tv = { 0, 100000 };
  while (!kill_handler_thread_)
  {
    libusb_handle_events_timeout_completed(usb_ctx_, &tv, &kill_handler_thread_);
  }
}

bool InputUsb::findDevices(std::string device_uuid)
{
  bool find_flag = false;
  libusb_device** usb_dev_list;
  ssize_t num_usb_devices = libusb_get_device_list(usb_ctx_, &usb_dev_list);
  if (num_usb_devices < 0)
  {
    RS_ERROR << "Failed to get device list: " << libusb_strerror((int)num_usb_devices) << RS_REND;
  }

  for (int i = 0; i < num_usb_devices; i++)
  {
    libusb_device* device = usb_dev_list[i];
    libusb_device_descriptor desc;

    if (libusb_get_device_descriptor(device, &desc) != LIBUSB_SUCCESS)
    {
      continue;
    }
    if (desc.idVendor == ROBOSENSE_VENDOR_ID &&
        (desc.idProduct == RS_AC1_PRODUCT_ID || desc.idProduct == RS_AC2_PRODUCT_ID))
    {
      libusb_device_handle* handle;
      std::string uuid_;
      if (libusb_open(device, &handle) == 0)
      {
        unsigned char serial[256];
        int ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, serial, sizeof(serial));
        if (ret > 0)
        {
          uuid_ = std::string(reinterpret_cast<char*>(serial));
        }
        libusb_close(handle);
      }

      if (device_uuid == "")
      {
        dev_ = device;
        libusb_ref_device(device);
        find_flag = true;
        RS_INFO << "Find device, uuid is : " << uuid_ << RS_REND;
      }
      else if (device_uuid == uuid_)
      {
        dev_ = device;
        libusb_ref_device(device);
        find_flag = true;
        RS_INFO << "Find device : " << uuid_ << RS_REND;
      }
      // else
      // {
      //   return false;
      // }

      if (find_flag)
      {
        if (desc.bcdUSB >= 0x0300)
        {
          is_usb_300_ = true;
        }
        else
        {
          is_usb_300_ = false;
        }
        is_ac2_ = (desc.idProduct == RS_AC2_PRODUCT_ID);
        break;
      }
    }
  }

  libusb_free_device_list(usb_dev_list, 1);
  return find_flag;
}

bool InputUsb::imageStreamInit()
{
  uvc_ctrl_image_ = new uvc_stream_ctrl_t();

  uvc_error_t res;
  if (!uvc_ctx_image_)
  {
    res = uvc_init(&uvc_ctx_image_, usb_ctx_);
    if (res < 0)
    {
      RS_ERROR << "Failed to initialize UVC: " << uvc_strerror(res) << RS_REND;
      uvc_ctx_image_ = nullptr;
      return false;
    }
  }

  uvc_dev_image_ = usb_device_creat_uvc_device(uvc_ctx_image_, dev_);

#ifdef WIN32
  res = uvc_open2(uvc_dev_image_, 0, devh_, &uvc_devh_image_);
#else
  if (is_ac2_)
  {
    res = uvc_open(uvc_dev_image_, 0, &uvc_devh_image_);
  }
  else
  {
    res = uvc_open2(uvc_dev_image_, 0, devh_, &uvc_devh_image_);
  }
#endif

  if (res < 0)
  {
    RS_ERROR << "Failed to open device: " << uvc_strerror(res) << RS_REND;
    return false;
  }

  return true;
}

bool InputUsb::imageStreamStart()
{
  if (!uvc_devh_image_)
  {
    return false;
  }

  uvc_frame_format frame_format = UVC_FRAME_FORMAT_ANY;
  switch (input_param_.image_format)
  {
    case int(FRAME_FORMAT_NV12):
      frame_format = UVC_FRAME_FORMAT_NV12;
      break;

    case int(FRAME_FORMAT_BGR24):
      frame_format = UVC_FRAME_FORMAT_BGR;
      break;

    case int(FRAME_FORMAT_RGB24):
      frame_format = UVC_FRAME_FORMAT_RGB;
      break;

    case int(FRAME_FORMAT_YUV422):
      frame_format = UVC_FRAME_FORMAT_YUYV;
      break;
    case int(FRAME_FORMAT_XR24):
      frame_format = UVC_FRAME_FORMAT_XR24;
      break;
    default:
      break;
  }

  uvc_error_t res;
  res = uvc_get_stream_ctrl_format_size(uvc_devh_image_, uvc_ctrl_image_, frame_format, input_param_.image_width,
                                        input_param_.image_height, input_param_.image_fps);
  RS_INFO << "frame_format:" << frame_format << ",image_width: " << input_param_.image_width
          << ", image_height: " << input_param_.image_height << ", image_fps: " << input_param_.image_fps << RS_REND;
  if (res < 0)
  {
    RS_ERROR << "Error in setting parameters for the image stream!" << RS_REND;
    uvc_ctx_image_ = nullptr;
    return false;
  }
  res = uvc_start_streaming(
      uvc_devh_image_, uvc_ctrl_image_,
      [](size_t size, void* ptr) -> uint8_t* {
        auto* self = static_cast<InputUsb*>(ptr);
        self->image_buf_ptr_ = self->cb_get_pkt_2_(size);
        return self->image_buf_ptr_->data();
      },
      [](void* ptr) {
        auto* self = static_cast<InputUsb*>(ptr);
        self->image_buf_ptr_->buf()[0] = 0xAA;
        self->image_buf_ptr_->buf()[1] = 0x66;

        switch ((uint8_t)(self->image_buf_ptr_->buf()[2]))
        {
          case UVC_COLOR_FORMAT_NV12:
            self->image_buf_ptr_->buf()[2] = (uint8_t)FRAME_FORMAT_NV12;
            break;

          case UVC_FRAME_FORMAT_BGR:
            self->image_buf_ptr_->buf()[2] = (uint8_t)FRAME_FORMAT_BGR24;
            break;

          case UVC_FRAME_FORMAT_RGB:
            self->image_buf_ptr_->buf()[2] = (uint8_t)FRAME_FORMAT_RGB24;
            break;

          case UVC_FRAME_FORMAT_YUYV:
            self->image_buf_ptr_->buf()[2] = (uint8_t)FRAME_FORMAT_YUV422;
            break;
          case UVC_FRAME_FORMAT_XR24:
            self->image_buf_ptr_->buf()[2] = (uint8_t)FRAME_FORMAT_XR24;
            break;

          default:
            break;
        }
        self->image_buf_ptr_->setData(0, self->image_buf_ptr_->bufSize());
        self->pushPacket2(self->image_buf_ptr_);
      },
      this, 0);
  if (res < 0)
  {
    RS_ERROR << "Failed to start streaming image: " << uvc_strerror(res) << RS_REND;
    return false;
  }

  return true;
}

void InputUsb::imageStreamClose()
{
#ifdef WIN32
  if (uvc_devh_image_)
  {
    uvc_close2(uvc_devh_image_);
    uvc_devh_image_ = nullptr;
  }

  if (uvc_ctx_image_)
  {
    uvc_exit2(uvc_ctx_image_);
    uvc_ctx_image_ = nullptr;
  }
#else
  if (uvc_devh_image_)
  {
    if (is_ac2_)
    {
      uvc_stop_streaming(uvc_devh_image_);
      uvc_close(uvc_devh_image_);
    }
    else
    {
      uvc_close2(uvc_devh_image_);
    }
    uvc_devh_image_ = nullptr;
  }

  if (uvc_ctx_image_)
  {
    if (is_ac2_)
    {
      uvc_exit(uvc_ctx_image_);
    }
    else
    {
      uvc_exit2(uvc_ctx_image_);
    }
    uvc_ctx_image_ = nullptr;
  }
#endif

  if (uvc_dev_image_)
  {
    free_uvc_device(uvc_dev_image_);
    uvc_dev_image_ = nullptr;
  }

  if (uvc_ctrl_image_)
  {
    delete uvc_ctrl_image_;
    uvc_ctrl_image_ = nullptr;
  }
}

bool InputUsb::pcStreamInit()
{
  if (is_ac2_)
  {
    return true;
  }
  uvc_ctrl_pc_ = new uvc_stream_ctrl_t();

  uvc_error_t res;
  if (!uvc_ctx_pc_)
  {
    res = uvc_init(&uvc_ctx_pc_, usb_ctx_);
    if (res < 0)
    {
      RS_ERROR << "Failed to initialize UVC: " << uvc_strerror(res) << RS_REND;
      uvc_ctx_pc_ = nullptr;
      return false;
    }
  }

  uvc_dev_pc_ = usb_device_creat_uvc_device(uvc_ctx_pc_, dev_);

  res = uvc_open2(uvc_dev_pc_, 1, devh_, &uvc_devh_pc_);
  if (res < 0)
  {
    RS_ERROR << "Failed to open device: " << uvc_strerror(res) << RS_REND;
    return false;
  }

  return true;
}

bool InputUsb::pcStreamStart()
{
  if (is_ac2_)
  {
    return true;
  }
  if (!uvc_devh_pc_)
  {
    return false;
  }

  uvc_error_t res;
  res = uvc_get_stream_ctrl_format_size(uvc_devh_pc_, uvc_ctrl_pc_, UVC_FRAME_FORMAT_ANY,
                                        RS_AC1_DEPTH_DATA_BYTES >> 11U, 1024, 10);

  if (res < 0)
  {
    RS_ERROR << "Error in setting parameters for the depth stream!" << RS_REND;
    uvc_ctx_pc_ = nullptr;
    return false;
  }

  res = uvc_start_streaming(
      uvc_devh_pc_, uvc_ctrl_pc_,
      [](size_t size, void* ptr) -> uint8_t* {
        auto* self = static_cast<InputUsb*>(ptr);
        self->pc_buf_ptr_ = self->cb_get_pkt_3_(size);
        return self->pc_buf_ptr_->data();
      },
      [](void* ptr) {
        auto* self = static_cast<InputUsb*>(ptr);
        self->pc_buf_ptr_->buf()[0] = 0xAA;
        self->pc_buf_ptr_->buf()[1] = 0x77;
        self->pc_buf_ptr_->setData(0, self->pc_buf_ptr_->bufSize());
        self->pushPacket3(self->pc_buf_ptr_);
      },
      this, 0);

  if (res < 0)
  {
    RS_ERROR << "Failed to start streaming depth: " << uvc_strerror(res) << RS_REND;
    return false;
  }

  return true;
}

void InputUsb::pcStreamClose()
{
  if (is_ac2_)
  {
    return;
  }
  if (uvc_devh_pc_)
  {
    uvc_close2(uvc_devh_pc_);
    uvc_devh_pc_ = nullptr;
  }

  if (uvc_ctx_pc_)
  {
    uvc_exit2(uvc_ctx_pc_);
    uvc_ctx_pc_ = nullptr;
  }

  if (uvc_dev_pc_)
  {
    free_uvc_device(uvc_dev_pc_);
    uvc_dev_pc_ = nullptr;
  }

  if (uvc_ctrl_pc_)
  {
    delete uvc_ctrl_pc_;
    uvc_ctrl_pc_ = nullptr;
  }
}

bool InputUsb::imuStreamInit()
{
  int res;

  libusb_device_descriptor desc;
  res = libusb_get_device_descriptor(dev_, &desc);
  if (res < 0)
  {
    RS_ERROR << "Failed to get device descriptor." << RS_REND;
    return false;
  }

  libusb_config_descriptor* config;
  res = libusb_get_config_descriptor(dev_, 0, &config);
  if (res < 0)
  {
    libusb_free_config_descriptor(config);
    RS_ERROR << "Failed to get config descriptor." << RS_REND;
    return false;
  }

  const libusb_interface_descriptor* if_desc;
  for (int interface_idx = 0; interface_idx < config->bNumInterfaces; ++interface_idx)
  {
    if_desc = &config->interface[interface_idx].altsetting[0];

    if (if_desc->bInterfaceClass == 3 && if_desc->bInterfaceSubClass == 1)
    {
      RS_DEBUG << "Potential HID device found, interface_num :" << interface_idx << RS_REND;
      hid_intface_num_ = interface_idx;
      hid_endpoint_in_num_ = if_desc->endpoint[0].bEndpointAddress;
      hid_endpoint_out_num_ = if_desc->endpoint[1].bEndpointAddress;
      break;
    }
  }
  libusb_free_config_descriptor(config);

  if (hid_intface_num_ < 0)
  {
    return false;
  }

  if (libusb_kernel_driver_active(devh_, hid_intface_num_))
  {
    res = libusb_detach_kernel_driver(devh_, hid_intface_num_);
    if (res != 0)
    {
      RS_ERROR << "Failed to detach kernel driver: " << libusb_error_name(res) << RS_REND;
    }
  }

  res = libusb_claim_interface(devh_, hid_intface_num_);
  if (res != 0)
  {
    RS_ERROR << "Failed to claim interface: " << libusb_error_name(res) << RS_REND;
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputUsb::recvPacket, this));

  std::this_thread::sleep_for(std::chrono::microseconds(50));

  return true;
}

void InputUsb::recvPacket()
{
  uint8_t recv_hid_data_buf[kMaxHidBufferSize];
  const auto interval = std::chrono::milliseconds(1000);
  auto last_time = std::chrono::steady_clock::now() - interval;
  while (1)
  {
    if (to_exit_recv_)
    {
      RS_INFO << "input usb recv thread exit!" << RS_REND;
      break;
    }

    if (!transfer_custom_cmd_)
    {
      /* ptp */
      if (input_param_.sync_timestamps)
      {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - last_time >= interval)
        {
          if (sendHidCmd(HidReqType::HID_REQ_SYNC))
          {
            // nothing to do
          }
          else
          {
            RS_ERROR << "send SS_HID_REQ_SYNC error!" << RS_REND;
          }
          last_time = current_time;
        }
      }

      // get hid recv_hid_data_buf
      int transfer_length = 0;
      const auto& resp_ret =
          usbTransferRx(devh_, hid_endpoint_in_num_, recv_hid_data_buf, kMaxHidBufferSize, transfer_length, 10);
      switch (resp_ret)
      {
        case HidRespType::HID_RESP_START_STOP:
          break;
        case HidRespType::HID_RESP_SYNC: {
          if (!sendHidCmd(HidReqType::HID_REQ_DELAY_RESP))
          {
            RS_ERROR << "send SS_HID_REQ_DELAY_RESP error!" << RS_REND;
          }
        }
        break;

        case HidRespType::HID_RESP_DELAY: {
        }
        break;

        case HidRespType::HID_RESP_IMU: {
          if (!cb_get_pkt_)
          {
            RS_ERROR << "cb_get_pkt_ callback not set!" << RS_REND;
            break;
          }
          std::shared_ptr<Buffer> pkt = cb_get_pkt_(transfer_length);
          if (pkt)
          {
            memcpy(pkt->buf(), recv_hid_data_buf, transfer_length);
            pkt->buf()[0] = 0xAA;
            pkt->buf()[1] = 0x55;
            pkt->setData(0, transfer_length);
            pushPacket(pkt);
          }
        }
        break;

        default:
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    else
    {
      std::unique_lock<std::mutex> lock(mtx_);
      cond_.wait_for(lock, std::chrono::milliseconds(10), [this]() { return !transfer_custom_cmd_; });
    }
  }
  return;
}

bool InputUsb::imuStreamStart()
{
  if (hid_start_flag_)
  {
    return true;
  }
  HidReqType hid_req_type = HidReqType::HID_REQ_IMU_UPLOAD_START_100HZ;
  switch (input_param_.imu_fps)
  {
  case 100:
      hid_req_type = HidReqType::HID_REQ_IMU_UPLOAD_START_100HZ;
      break;
  case 200:
      hid_req_type = HidReqType::HID_REQ_IMU_UPLOAD_START_200HZ;
      break;
  default:
      RS_ERROR << "imu fps set error, only support 100Hz and 200Hz!" << RS_REND;
      return false;
  }
  if (!sendHidCmd(hid_req_type))
  {
    RS_ERROR << "imu start error!!" << RS_REND;
    return false;
  }
  hid_start_flag_ = true;
  return true;
}

void InputUsb::hidStop()
{
  if (hid_start_flag_)
  {
    sendHidCmd(HidReqType::HID_REQ_IMU_UPLOAD_STOP);
    hid_start_flag_ = false;
  }
}

void InputUsb::imuStreamClose()
{
  if (devh_)
  {
    hidStop();
  }

  hid_intface_num_ = -1;
  hid_endpoint_in_num_ = -1;
  hid_endpoint_out_num_ = -1;
}

bool InputUsb::sendHidCmd(HidReqType type)
{
  int res = 0;
  int transfer_length;
  uint8_t req_data[kMaxHidBufferSize];
  int len = 0;
  switch (type)
  {
    case HidReqType::HID_REQ_IMU_UPLOAD_START_100HZ:
    {
      req_data[8] = 0x2E;
      req_data[9] = 0x00;
      req_data[10] = 0x02;
      req_data[11] = 0x00;
      len = 12;
      buildHidFrameHeader(req_data, len);
    }
    break;
    case HidReqType::HID_REQ_IMU_UPLOAD_START_200HZ: 
    {
      req_data[8] = 0x2E;
      req_data[9] = 0x00;
      req_data[10] = 0x02;
      req_data[11] = 0x01;
      len = 12;
      buildHidFrameHeader(req_data, len);
    }
    break;

    case HidReqType::HID_REQ_IMU_UPLOAD_STOP: 
    {
      req_data[8] = 0x2E;
      req_data[9] = 0x00;
      req_data[10] = 0x00;
      req_data[11] = 0x00;
      len = 12;
      buildHidFrameHeader(req_data, len);
    }
    break;

    case HidReqType::HID_REQ_SYNC:
    case HidReqType::HID_REQ_DELAY_RESP: 
    {
      req_data[8] = 0x31;
      if (type == HidReqType::HID_REQ_SYNC)
      {
        req_data[9] = 0x00;
      }
      else if (type == HidReqType::HID_REQ_DELAY_RESP)
      {
        req_data[9] = 0x01;
      }
      struct timespec ts;

#ifdef PLATFORM_WINDOWS_MSVC
      FILETIME ft;
      unsigned __int64 tmpres = 0;
      GetSystemTimeAsFileTime(&ft);
      tmpres |= ft.dwHighDateTime;
      tmpres <<= 32;
      tmpres |= ft.dwLowDateTime;
      tmpres -= 116444736000000000Ui64;
      ts.tv_sec = (long)(tmpres / 10000000UL);
      ts.tv_nsec = (long)(tmpres % 10000000UL) * 100;
#else
      clock_gettime(CLOCK_REALTIME, &ts);
#endif

      memcpy(req_data + 10, &ts, sizeof(struct timespec));
      len = 26;
      buildHidFrameHeader(req_data, len);
    }
    break;

    default:
      break;
  }
  res = libusb_interrupt_transfer(devh_, hid_endpoint_out_num_, req_data, len, &transfer_length, 100);
  if (res == LIBUSB_ERROR_NO_DEVICE || res == LIBUSB_ERROR_IO)
  {
    cb_excep_(Error(ERRCODE_DEVICE_DISCONNECTED));
    return true;
  }

  return res < 0 ? false : true;
}

HidRespType InputUsb::parseHidResp(const uint8_t* data, int len)
{
  constexpr int kMinFrameSize = 10;
  if (!data || len < kMinFrameSize)
  {
    RS_ERROR << "Invalid HID data (null or too short)!" << RS_REND;
    return HidRespType::HID_RESP_ERROR;
  }
  if (!(data[0] == kHidFrameHead1 && data[1] == kHidFrameHead2))
  {
    RS_ERROR << "Hid frame head error!" << RS_REND;
    return HidRespType::HID_RESP_HEAD_ERROR;
  }

  if (len > kFrameHeaderSize)
  {
    uint16_t crc16_recv = (uint16_t)data[7] << 8 | data[6];
    uint16_t crc16 = crc_16(data + kFrameHeaderSize, len - kFrameHeaderSize);
    if (crc16_recv != crc16)
    {
      return HidRespType::HID_RESP_CRC_ERROR;
    }
  }

  switch (data[8])
  {
    case 0x6E:
      return HidRespType::HID_RESP_START_STOP;

    case 0x71:
      if (data[9] == 0x00)
        return HidRespType::HID_RESP_SYNC;
      else if (data[9] == 0x01)
        return HidRespType::HID_RESP_DELAY;
      else if (data[9] == 0x04)
        return HidRespType::HID_RESP_OTHER;
      break;

    case 0x75:
      if (data[9] == 0x01)
        return HidRespType::HID_RESP_IMU;
      break;

    case 0x7f:
      return HidRespType::HID_RESP_ERROR;

    default:
      break;
  }

  return HidRespType::HID_RESP_OTHER;
}

void InputUsb::buildHidFrameHeader(uint8_t* data, int len)
{
  if (!data || len < kFrameHeaderSize)
  {
    return;
  }
  data[0] = kHidFrameHead1;
  data[1] = kHidFrameHead2;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = (len - kFrameHeaderSize) & 0xFF;
  data[5] = ((len - kFrameHeaderSize) >> 8) & 0xFF;
  data[6] = 0x00;
  data[7] = 0x00;

  uint16_t crc16 = crc_16(data + kFrameHeaderSize, len - kFrameHeaderSize);
  data[6] = crc16 & 0xFF;
  data[7] = (crc16 >> 8) & 0xFF;
}

#define DEBUG_HID
bool InputUsb::usbTransferTx(libusb_device_handle* devh, int endpoint_out, uint8_t* data, int len)
{
  int transfer_length;
  int res = libusb_interrupt_transfer(devh, endpoint_out, data, len, &transfer_length, 100);

  return res < 0 ? false : true;
}

HidRespType InputUsb::usbTransferRx(libusb_device_handle* devh, int endpoint_in, uint8_t* data, int len,
                                     int& transfer_length, int time_out)
{
  int res = libusb_interrupt_transfer(devh, endpoint_in, data, len, &transfer_length, time_out);
  if (res == 0 && transfer_length > 0)
  {
    return parseHidResp(data, transfer_length);
  }

  return HidRespType::HID_RESP_ERROR;
}

void InputUsb::clearHidRecvCache()
{
  std::vector<uint8_t> dummy_data(kMaxHidBufferSize);
  int transfer_length = 0;
  for (int i = 0; i < kClearRecvCacheTimes; ++i)
  {
    usbTransferRx(devh_, hid_endpoint_in_num_, dummy_data.data(), kMaxHidBufferSize, transfer_length);
  }
}
bool InputUsb::customCmd(const std::vector<uint8_t>& send, std::vector<uint8_t>& receive)
{
  if (is_ac2_)
  {
    RS_ERROR << "AC2 device does not support custom cmd!" << RS_REND;
    return false;
  }
  if (send.size() > kMaxCustomCmdSize)
  {
    RS_ERROR << "Custom cmd size exceed max limit! Max: " << kMaxCustomCmdSize << ", Actual: " << send.size()
             << RS_REND;
    return false;
  }
  bool need_restart_imu = hid_start_flag_;
  hidStop();

  {
    std::lock_guard<std::mutex> lock(mtx_);
    transfer_custom_cmd_.store(true); 
  }

  bool ret = true;
  try
  {
    clearHidRecvCache();

    std::vector<uint8_t> req_data(kMaxHidBufferSize, 0);
    memcpy(req_data.data() + kFrameHeaderSize, send.data(), send.size());
    int len = kFrameHeaderSize + send.size();
    buildHidFrameHeader(req_data.data(), len);

    if (!usbTransferTx(devh_, hid_endpoint_out_num_, req_data.data(), len))
    {
      throw std::runtime_error("Custom cmd send failed");
    }

    std::vector<uint8_t> receive_data(kMaxHidBufferSize);
    int transfer_length = 0;
    const auto& resp = usbTransferRx(devh_, hid_endpoint_in_num_, receive_data.data(), kMaxHidBufferSize,
                                       transfer_length, kCustomCmdTimeout);
    if (resp == HidRespType::HID_RESP_HEAD_ERROR || resp == HidRespType::HID_RESP_CRC_ERROR || resp == HidRespType::HID_RESP_ERROR)
    {
      throw std::runtime_error("Custom cmd resp error: " + std::to_string(static_cast<uint8_t>(resp)));
    }

    if (receive_data.size() > kFrameHeaderSize)
    {
      receive.assign(receive_data.begin() + kFrameHeaderSize, receive_data.begin() + transfer_length);
    }
  }
  catch (const std::exception& e)
  {
    RS_ERROR << "Custom cmd exception: " << e.what() << RS_REND;
    ret = false;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    transfer_custom_cmd_.store(false);
    cond_.notify_one(); 
  }

  if (need_restart_imu)
  {
    imuStreamStart();
  }

  return ret;
}

}  // namespace lidar
}  // namespace robosense

#endif  // ENABLE_USB