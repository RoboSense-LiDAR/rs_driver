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

#define VENDOR_ID 0x3840
#define PRODUCT_ID 0x1010

#define ALIGN_UP(V_, ALIGN_)        (((V_) + ((ALIGN_) - 1U)) & (~((ALIGN_) - 1U)))
#define POINT_WIDTH_NUMS            96U
#define POINT_HEIGHT_NUMS           288U
#define POINT_NUMS                  POINT_WIDTH_NUMS * POINT_HEIGHT_NUMS
#define DEPTH_DATA_BYTES_BEFORE            (10U + POINT_WIDTH_NUMS * 12U) * POINT_HEIGHT_NUMS
/**
 * Extra 10 bytes: TimeBase
 * Every pixel contains 12 bytes:
 *  TimeOffset, X, Y, Z, Distance, Reflectivity, Attribute
 */
#define DEPTH_DATA_BYTES            ALIGN_UP(DEPTH_DATA_BYTES_BEFORE, 2048U)

namespace robosense
{
namespace lidar
{

bool InputUsb::init()
{
  int res = 0;
  if (init_flag_)
    return true;

  if (!_usb_ctx)
  {
      res = libusb_init(&_usb_ctx);
      if (res < 0)
      {
          RS_ERROR << "Failed to initialize libusb:" << libusb_strerror(res) << RS_REND;
          _usb_ctx = nullptr;
          return false;
      }
  }
  
  if(findDevices(input_param_.device_uuid) == false)
  {
    RS_ERROR << "Device is not found, please check!" << RS_REND;
    return false;
  }

  if(!_is_usb_300 && !input_param_.enable_usb200)
  {
    RS_ERROR << "USB mode is not USB_3.0!" << RS_REND;
    return false;
  }

  _kill_handler_thread = 0;
  _usb_thread = std::thread(std::bind(&InputUsb::usbEventHandler, this));

  res = libusb_open(_dev, &_devh);

  if(res != 0)
  {
    RS_ERROR << "Failed to open device: " << libusb_strerror(res) << RS_REND;
    return false;
  }

  if(imuStreamInit() == false)
  {
    RS_ERROR << "Depth stream init failed!" << RS_REND;
    return false;
  }

  if(input_param_.enable_image)
  {
    if(imageStreamInit() == false)
    {
      RS_ERROR << "Image stream init failed!" << RS_REND;
      return false;
    }
  }

  if(pcStreamInit() == false)
  {
    RS_ERROR << "Depth stream init failed!" << RS_REND;
    return false;
  }

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

  if(imuStreamStart() == false)
  {
    RS_ERROR << "Imu stream start failed!" << RS_REND;
    return false;
  }

  if(input_param_.enable_image)
  {
    if(imageStreamStart() == false)
    {
      RS_ERROR << "Image stream start failed!" << RS_REND;
      return false;
    }
  }

  if(pcStreamStart() == false)
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

  if (!_kill_handler_thread)
  {
    _kill_handler_thread = 1;
    if (_usb_thread.joinable())
    {
        _usb_thread.join();
    }
  }

  if(_devh)
  {
    libusb_close(_devh);
    _devh = nullptr;
  }

  if (_usb_ctx)
  {
      libusb_exit(_usb_ctx);
      _usb_ctx = nullptr;
  }
}

InputUsb::~InputUsb()
{
  stop();
  // RS_INFO << "--- ~InputUsb() ---" << RS_REND;
}

void InputUsb::usbEventHandler()
{

  struct timeval tv = {0, 100000};
  while (!_kill_handler_thread)
  {
    libusb_handle_events_timeout_completed(_usb_ctx, &tv, &_kill_handler_thread);
  }
}

bool InputUsb::findDevices(std::string device_uuid)
{
  bool find_flag = false;
  libusb_device **usb_dev_list;
  ssize_t num_usb_devices = libusb_get_device_list(_usb_ctx, &usb_dev_list);
  if (num_usb_devices < 0)
  {
    RS_ERROR << "Failed to get device list: " << libusb_strerror((int)num_usb_devices) << RS_REND;
  }

  for (int i = 0; i < num_usb_devices; i++)
  {
      libusb_device *device = usb_dev_list[i];
      libusb_device_descriptor desc;

      if (libusb_get_device_descriptor(device, &desc) != LIBUSB_SUCCESS)
      {
          continue;
      }
      if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID)
      {

        libusb_device_handle *handle;
        std::string uuid_;
        if (libusb_open(device, &handle) == 0)
        {
          unsigned char serial[256];
          int ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, serial, sizeof(serial));
          if (ret > 0)
          {
              uuid_ = std::string(reinterpret_cast<char *>(serial));
          }
          libusb_close(handle);
        }

        if (device_uuid == "")
        {
          _dev = device;
          libusb_ref_device(device);
          find_flag = true;
          RS_INFO << "Find device, uuid is : " << uuid_ << RS_REND;

        }
        else if (device_uuid == uuid_)
        {
          _dev = device;
          libusb_ref_device(device);
          find_flag = true;
          RS_INFO << "Find device : " << uuid_ << RS_REND;
        }
        else
        {
          return false;
        }

        if (desc.bcdUSB >= 0x0300)
        {
          // RS_INFO << "USB mode is USB3.0" << RS_REND;
          _is_usb_300 = true;
        }
        else
        {
          // RS_INFO << "USB mode is USB2.0" << RS_REND;
          _is_usb_300 = false;
        }
          
      }
  }

  libusb_free_device_list(usb_dev_list, 1);
  return find_flag;
}

bool InputUsb::imageStreamInit()
{
  _uvc_ctrl_image = new uvc_stream_ctrl_t();

  uvc_error_t res;
  if (!_uvc_ctx_image)
  {
    res = uvc_init(&_uvc_ctx_image, _usb_ctx);
    if (res < 0)
    {
      RS_ERROR << "Failed to initialize UVC: " << uvc_strerror(res) << RS_REND;
      _uvc_ctx_image = nullptr;
      return false;
    }
  }

  _uvc_dev_image = usb_device_creat_uvc_device(_uvc_ctx_image, _dev);
  res = uvc_open2(_uvc_dev_image, 0, _devh, &_uvc_devh_image);
  if (res < 0)
  {
    RS_ERROR << "Failed to open device: " << uvc_strerror(res) << RS_REND;
    return false;
  }
  
  return true;
}

bool InputUsb::imageStreamStart()
{
  if (!_uvc_devh_image)
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

  default:
      break;
  }

  uvc_error_t res;
  res = uvc_get_stream_ctrl_format_size(
      _uvc_devh_image, _uvc_ctrl_image, frame_format,
      input_param_.image_width, input_param_.image_height, input_param_.image_fps);

  if (res < 0)
  {
    RS_ERROR << "Error in setting parameters for the image stream!" << RS_REND;
    _uvc_ctx_image = nullptr;
    return false;
  }

  res = uvc_start_streaming(_uvc_devh_image, _uvc_ctrl_image, 
          [](size_t size, void *ptr)-> uint8_t* 
          {
            auto* self = static_cast<InputUsb*>(ptr);
            self->_pkt_image = self->cb_get_pkt_2_(size);
            return self->_pkt_image->data();
          }, 
          [](void *ptr) 
          {
            auto* self = static_cast<InputUsb*>(ptr);
            self->_pkt_image->buf()[0] = 0xAA;
            self->_pkt_image->buf()[1] = 0x66;

            switch ((uint8_t)(self->_pkt_image->buf()[2]))
            {
            case UVC_COLOR_FORMAT_NV12:
              self->_pkt_image->buf()[2] = (uint8_t)FRAME_FORMAT_NV12;
              break;

            case UVC_FRAME_FORMAT_BGR:
              self->_pkt_image->buf()[2] = (uint8_t)FRAME_FORMAT_BGR24;
              break;

            case UVC_FRAME_FORMAT_RGB:
              self->_pkt_image->buf()[2] = (uint8_t)FRAME_FORMAT_RGB24;
              break;

            case UVC_FRAME_FORMAT_YUYV:
              self->_pkt_image->buf()[2] = (uint8_t)FRAME_FORMAT_YUV422;
              break;

            default:
              break;
            }

            self->_pkt_image->setData(0, self->_pkt_image->bufSize());
            self->pushPacket2(self->_pkt_image);
          }, this, 0);

  if (res < 0)
  {
    RS_ERROR << "Failed to start streaming image: " << uvc_strerror(res) << RS_REND;
    return false;
  }

  return true;
}

void InputUsb::imageStreamClose()
{
  if (_uvc_devh_image)
  {
      uvc_close2(_uvc_devh_image);
      _uvc_devh_image = nullptr;
  }

  if (_uvc_ctx_image)
  {
      uvc_exit2(_uvc_ctx_image);
      _uvc_ctx_image = nullptr;
  }

  if (_uvc_dev_image)
  {
      free_uvc_device(_uvc_dev_image);
      _uvc_dev_image = nullptr;
  }

  if (_uvc_ctrl_image)
  {
      delete _uvc_ctrl_image;
      _uvc_ctrl_image = nullptr;
  }
}


bool InputUsb::pcStreamInit()
{
  _uvc_ctrl_pc = new uvc_stream_ctrl_t();

  uvc_error_t res;
  if (!_uvc_ctx_pc)
  {
    res = uvc_init(&_uvc_ctx_pc, _usb_ctx);
    if (res < 0)
    {
      RS_ERROR << "Failed to initialize UVC: " << uvc_strerror(res) << RS_REND;
      _uvc_ctx_pc = nullptr;
      return false;
    }
  }

  _uvc_dev_pc = usb_device_creat_uvc_device(_uvc_ctx_pc, _dev);
  res = uvc_open2(_uvc_dev_pc, 1, _devh, &_uvc_devh_pc);
  if (res < 0)
  {
    RS_ERROR << "Failed to open device: " << uvc_strerror(res) << RS_REND;
    return false;
  }

  return true;
}

bool InputUsb::pcStreamStart()
{
  if (!_uvc_devh_pc)
  {
      return false;
  }

  uvc_error_t res;
  res = uvc_get_stream_ctrl_format_size(
        _uvc_devh_pc, _uvc_ctrl_pc, UVC_FRAME_FORMAT_ANY,
        DEPTH_DATA_BYTES >> 11U, 1024, 10);

  if (res < 0)
  {
    RS_ERROR << "Error in setting parameters for the depth stream!" << RS_REND;
    _uvc_ctx_pc = nullptr;
    return false;
  }

  res = uvc_start_streaming(_uvc_devh_pc, _uvc_ctrl_pc, 
        [](size_t size, void *ptr)-> uint8_t* 
        {
          auto* self = static_cast<InputUsb*>(ptr);
          self->_pkt_pc = self->cb_get_pkt_3_(size);
          return self->_pkt_pc->data();
        }, 
        [](void *ptr) 
        {
          auto* self = static_cast<InputUsb*>(ptr);
          self->_pkt_pc->buf()[0] = 0xAA;
          self->_pkt_pc->buf()[1] = 0x77;
          self->_pkt_pc->setData(0, self->_pkt_pc->bufSize());
          self->pushPacket3(self->_pkt_pc);
        }, this, 0);


  if (res < 0)
  {
    RS_ERROR << "Failed to start streaming depth: " << uvc_strerror(res) << RS_REND;
    return false;
  }

  return true;
}

void InputUsb::pcStreamClose()
{
  if (_uvc_devh_pc)
  {
      uvc_close2(_uvc_devh_pc);
      _uvc_devh_pc = nullptr;
  }

  if (_uvc_ctx_pc)
  {
      uvc_exit2(_uvc_ctx_pc);
      _uvc_ctx_pc = nullptr;
  }

  if (_uvc_dev_pc)
  {
      free_uvc_device(_uvc_dev_pc);
      _uvc_dev_pc = nullptr;
  }

  if (_uvc_ctrl_pc)
  {
      delete _uvc_ctrl_pc;
      _uvc_ctrl_pc = nullptr;
  }
}


bool InputUsb::imuStreamInit()
{
  int res;

  libusb_device_descriptor desc;
  res = libusb_get_device_descriptor(_dev, &desc);
  if (res < 0)
  {
      RS_ERROR << "Failed to get device descriptor." << RS_REND;
      return false;
  }

  libusb_config_descriptor *config;
  res = libusb_get_config_descriptor(_dev, 0, &config);
  if (res < 0)
  {
      libusb_free_config_descriptor(config);
      RS_ERROR << "Failed to get config descriptor." << RS_REND;
      return false;
  }

  const libusb_interface_descriptor *if_desc;
  for (int interface_idx = 0; interface_idx < config->bNumInterfaces; ++interface_idx)
  {
      if_desc = &config->interface[interface_idx].altsetting[0];

      if (if_desc->bInterfaceClass == 3 && if_desc->bInterfaceSubClass == 1)
      {
          RS_DEBUG << "Potential HID device found, interface_num :" << interface_idx << RS_REND;
          _hid_intface_num = interface_idx;
          _hid_endpoint_in_num = if_desc->endpoint[0].bEndpointAddress;
          _hid_endpoint_out_num = if_desc->endpoint[1].bEndpointAddress;
          break;
      }
  }
  libusb_free_config_descriptor(config);

  if (_hid_intface_num < 0)
  {
      return false;
  }

  if (libusb_kernel_driver_active(_devh, _hid_intface_num))
  {
      res = libusb_detach_kernel_driver(_devh, _hid_intface_num);
      if (res != 0)
      {
        RS_ERROR << "Failed to detach kernel driver: " << libusb_error_name(res) << RS_REND;
      }
  }

  res = libusb_claim_interface(_devh, _hid_intface_num);
  if (res != 0)
  {
    RS_ERROR << "Failed to detach kernel driver: " << libusb_error_name(res) << RS_REND;
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputUsb::recvPacket, this));

  std::this_thread::sleep_for(std::chrono::microseconds(50));

  return true;
}

void InputUsb::recvPacket()
{
  uint8_t data[100];
  int transfer_length;
  const auto interval = std::chrono::milliseconds(1000);
  auto last_time = std::chrono::steady_clock::now() - interval;
  while (!to_exit_recv_)
  {
    /* ptp */
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - last_time >= interval)
    {
        // RS_INFO << "start SS_HID_REQ_SYNC!" << RS_REND;
        if (!send_cmd(HID_REQ_SYNC))
        {
          RS_ERROR << "send SS_HID_REQ_SYNC error!" << RS_REND;
        }
        last_time = current_time;
    }

    // get hid data
    int res = libusb_interrupt_transfer(_devh, _hid_endpoint_in_num, data, sizeof(data), &transfer_length, 10);

    if (res == 0 && transfer_length > 0)
    {
        switch (parse_cmd(data, transfer_length))
        {
        case HID_RESP_START_STOP:
            // RS_DEBUG << "start_stop receive!" << RS_REND;
            break;
        case HID_RESP_SYNC:
        {
            if (!send_cmd(HID_REQ_DELAY_RESP))
            {
              RS_ERROR << "send SS_HID_REQ_DELAY_RESP error!" << RS_REND;
            }
        }
        break;

        case HID_RESP_DELAY:
        {
          // RS_DEBUG << "ptp success!" << RS_REND;
        }
        break;

        case HID_RESP_IMU:
        {
          // RS_INFO << "HID_RESP_IMU!" << RS_REND;
          std::shared_ptr<Buffer> pkt = cb_get_pkt_(transfer_length);

          memcpy(pkt->buf(), data, transfer_length);
          pkt->buf()[0] = 0xAA;
          pkt->buf()[1] = 0x55;
          pkt->setData(0, transfer_length);
          pushPacket(pkt);
        }
        break;

        default:
            break;
        }
    }
  }
  return;
}


bool InputUsb::imuStreamStart()
{
  if (!send_cmd(HID_REQ_IMU_UPLOAD_START))
  {
      RS_ERROR << "imu start error!!" << RS_REND;
      return false;
  }

  return true;
}

void InputUsb::imuStreamClose()
{
  if(_devh)
    send_cmd(HID_REQ_IMU_UPLOAD_STOP);

  _hid_intface_num = -1;
  _hid_endpoint_in_num = -1;
  _hid_endpoint_out_num = -1;
}

bool InputUsb::send_cmd(hid_req req)
{
  int res = 0;
  int transfer_length;
  uint8_t req_data[1024];
  int len = 0;
  switch (req)
  {
  case HID_REQ_IMU_UPLOAD_START:
  {
    req_data[8] = 0x2E;
    req_data[9] = 0x00;
    req_data[10] = 0x02;
    len = 11;
    creat_frame_header(req_data, len);
  }
  break;

  case HID_REQ_IMU_UPLOAD_STOP:
  {
    req_data[8] = 0x2E;
    req_data[9] = 0x00;
    req_data[10] = 0x00;
    len = 11;
    creat_frame_header(req_data, len);
  }
  break;

  case HID_REQ_SYNC:
  case HID_REQ_DELAY_RESP:
  {
    req_data[8] = 0x31;
    if (req == HID_REQ_SYNC)
    {
        req_data[9] = 0x00;
    }
    else if (req == HID_REQ_DELAY_RESP)
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
    creat_frame_header(req_data, len);
  }
  break;

  default:
    break;
  }

  res = libusb_interrupt_transfer(_devh, _hid_endpoint_out_num, req_data, len, &transfer_length, 100);

  if(res == LIBUSB_ERROR_NO_DEVICE || res == LIBUSB_ERROR_IO)
  {
    cb_excep_(Error(ERRCODE_DEVICE_DISCONNECTED));
    return true;
  }

  return res < 0 ? false : true;
}

hid_resp InputUsb::parse_cmd(const uint8_t *data, int len)
{
  if (!(data[0] == 0xFE && data[1] == 0xAA))
  {
      RS_DEBUG << "Hid frame head error!" << RS_REND;
      return HID_RESP_HEAD_ERROR;
  }

    if (len > 8)
  {
      uint16_t crc16_recv = (uint16_t)data[7] << 8 | data[6];
      uint16_t crc16 = crc_16(data + 8, len - 8);
      if (crc16_recv != crc16)
      {
          return HID_RESP_CRC_ERROR;
      }
  }

  switch (data[8])
  {
  case 0x6E:
      return HID_RESP_START_STOP;

  case 0x71:
      if (data[9] == 0x00)
          return HID_RESP_SYNC;
      else if (data[9] == 0x01)
          return HID_RESP_DELAY;
      break;

  case 0x75:
      if (data[9] == 0x01)
          return HID_RESP_IMU;
      break;

  case 0x7f:
      return HID_RESP_ERROR;

  default:
      break;
  }

  return HID_RESP_ERROR;
}

void InputUsb::creat_frame_header(uint8_t* data, int len)
{
  data[0] = 0xFE;
  data[1] = 0xAA;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = (len - 8) & 0xFF;
  data[5] = ((len - 8) >> 8) & 0xFF;
  data[6] = 0x00;
  data[7] = 0x00;

  uint16_t crc16 = crc_16(data + 8, len - 8);
  data[6] = crc16 & 0xFF;
  data[7] = (crc16 >> 8) & 0xFF;
}

}  // namespace lidar
}  // namespace robosense

#endif // ENABLE_USB