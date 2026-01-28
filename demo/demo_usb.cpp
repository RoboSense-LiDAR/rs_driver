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

#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

#include <signal.h>
#include <atomic>

#define ORDERLY_EXIT

// Define the macro: 1 to enable IMU parsing, 0 to disable IMU parsing
#define ENABLE_IMU_PARSE 1
#define ENABLE_IMAGE_PARSE 1

#define PRINT_SENSOR_INFO 1

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace robosense::lidar;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

SyncQueue<std::shared_ptr<ImuData>> free_imu_data_queue;
SyncQueue<std::shared_ptr<ImuData>> stuffed_imu_data_queue;

SyncQueue<std::shared_ptr<ImageMsg>> free_image_data_queue;
SyncQueue<std::shared_ptr<ImageMsg>> stuffed_image_data_queue;

std::atomic<bool> g_should_exit(false);
std::atomic<bool> to_exit_process(false);
void signalHandler(int signum)
{
  if (signum == SIGINT)
  {
    g_should_exit = true;
    to_exit_process = true;
    RS_INFO << "Received Ctrl+C, exiting..." << RS_REND;
  }
}

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this fucntion, the driver gets an free/unused point cloud message from the caller.
// @param msg  The free/unused point cloud message.
//
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed point cloud message to the caller.
// @param msg  The stuffed point cloud message.
//
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud()
  //       below)
  stuffed_cloud_queue.push(msg);
}

//
// @brief IMU data callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed IMU data message to the caller.
// @param msg  The stuffed IMU data message.
//
std::shared_ptr<ImuData> driverGetImuDataFromCallerCallback(void)
{
  // Note: This callback function runs in the packet-parsing/imu-data-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<ImuData> msg = free_imu_data_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }
  return std::make_shared<ImuData>();
}

//
// @brief IMU data callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed IMU data message to the caller.
// @param msg  The stuffed IMU data message.
//
void driverReturnImuDataToCallerCallback(const std::shared_ptr<ImuData>& msg)
{
  // Note: This callback function runs in the packet-parsing/imu-data-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see
  //       processImuData() below)
  stuffed_imu_data_queue.push(msg);
}

// @brief Image data callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed Image data message to the caller.
// @param msg  The stuffed Image data message.
//
std::function<std::shared_ptr<ImageMsg>()> createImageDataCallback(LidarType lidar_type)
{
  return [lidar_type]() -> std::shared_ptr<ImageMsg> {
    std::shared_ptr<ImageMsg> msg = free_image_data_queue.pop();
    if (msg)
    {
      return msg;
    }

    switch (lidar_type)
    {
      case LidarType::RS_AC1:
        return std::make_shared<MonoImageMsg>();
      default:
        return std::make_shared<StereoImageMsg>();
    }
  };
}


//
// @brief Image data callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed Image data message to the caller.
// @param msg  The stuffed Image data message.
//
void driverReturnImageDataToCallerCallback(const std::shared_ptr<ImageMsg>& msg)
{
  // Note: This callback function runs in the packet-parsing/image-data-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see
  //       processImageData() below)
  stuffed_image_data_queue.push(msg);
}

//
// @brief exception callback function. The caller should register it to the lidar driver.
//        Via this function, the driver inform the caller that something happens.
// @param code The error code to represent the error/warning/information
//
void exceptionCallback(const Error& code)
{
  // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the
  // driver,
  //       so please DO NOT do time-consuming task here.
  RS_WARNING << code.toString() << RS_REND;
}

void processImuData(void)
{
  uint32_t imu_cnt = 0;
  while (!to_exit_process)
  {
    std::shared_ptr<ImuData> msg = stuffed_imu_data_queue.popWait();

    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the IMU data msg, even it is time-consuming.

    imu_cnt++;
    if(imu_cnt % 200 == 0)
    {
      RS_MSG << "imu cnt: " << imu_cnt << " , timestamp: " << msg->timestamp << RS_REND;
    }
#if 0
    RS_DEBUG  <<std::fixed << std::setprecision(8)  <<"imu data:, timestamp: " << msg->timestamp << " , linear_a_x " << msg->linear_acceleration_x 
      << " , linear_a_y " << msg->linear_acceleration_y << "  , linear_a_z " << msg->linear_acceleration_z   
      << " , angular_v_x " << msg->angular_velocity_x << " , angular_v_y " << msg->angular_velocity_y 
      << " , angular_v_z " <<msg->angular_velocity_z << RS_REND;
#endif

    free_imu_data_queue.push(msg);
  }
}

#define EXPECT_FPS 10
void processImageData(void)
{
  uint32_t image_cnt = 0;
  while (!to_exit_process)
  {
    std::shared_ptr<ImageMsg> msg = stuffed_image_data_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    if (msg->camera_mode == CameraMode::STEREO)
    {
      auto stereo = std::dynamic_pointer_cast<StereoImageMsg>(msg);
      if (stereo->left_data != NULL)
      {
      }
      if (stereo->right_data != NULL)
      {
      }
    }
    else
    {
      auto mono = std::dynamic_pointer_cast<MonoImageMsg>(msg);
    }
    // Well, it is time to process the Image data msg, even it is time-consuming.

    image_cnt++;

    RS_MSG << "image seq: " << image_cnt << " image data ts: " << msg->timestamp << RS_REND;

    free_image_data_queue.push(msg);
  }
}

void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    RS_MSG << "cloud seq: " << msg->seq << " cloud data ts: " << msg->timestamp << RS_REND;
    // Well, it is time to process the point cloud msg, even it is time-consuming.
    free_cloud_queue.push(msg);
  }
}

#define USE_AC1 0
int main(int argc, char* argv[])
{
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;

  signal(SIGINT, signalHandler);
  RSDriverParam param;  ///< Create a parameter object
  param.input_type = InputType::USB;
  
#if USE_AC1
  param.lidar_type = LidarType::RS_AC1;  ///< Set the lidar type. Make sure this type is correct
  // param.input_param.device_uuid = "00000000";
#else
  param.lidar_type = LidarType::RS_AC2;  ///< Set the lidar type. Make sure this type is correct
  param.input_param.image_width = 1616;  ///< Set the image width. Make sure this value is correct
  param.input_param.image_height = 2636;  ///< Set the image height. Make sure this value is correct
  param.input_param.image_format = FRAME_FORMAT_XR24;  ///< Set the image format. Make sure this value is correct
  param.input_param.image_fps = 15;  ///< Set the image fps. Make sure this value is correct
#endif
  param.print();

  RS_DEBUG << "Compiled on " << __DATE__ << " at " << __TIME__ << RS_REND;

  LidarDriver<PointCloudMsg> driver;  ///< Declare the driver object
  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback,
                               driverReturnPointCloudToCallerCallback);  ///< Register the point cloud callback
                                                                         ///< functions
  driver.regExceptionCallback(exceptionCallback);                        ///< Register the exception callback function
#if ENABLE_IMU_PARSE
  driver.regImuDataCallback(driverGetImuDataFromCallerCallback, driverReturnImuDataToCallerCallback);
#endif

#if ENABLE_IMAGE_PARSE

  driver.regImageDataCallback(createImageDataCallback(param.lidar_type), driverReturnImageDataToCallerCallback);
#endif

  if (!driver.init(param))  ///< Call the init function
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }

  std::thread cloud_handle_thread = std::thread(processCloud);
#if ENABLE_IMU_PARSE
  std::thread imuData_handle_thread = std::thread(processImuData);
#endif

#if ENABLE_IMAGE_PARSE
  std::thread imageData_handle_thread = std::thread(processImageData);
#endif
  if (!driver.start())  //< The driver thread will start
  {
    RS_ERROR << "Driver Start Error..." << RS_REND;
    driver.stop();

    to_exit_process = true;
    cloud_handle_thread.join();
#if ENABLE_IMU_PARSE
    imuData_handle_thread.join();
#endif
#if ENABLE_IMAGE_PARSE
    imageData_handle_thread.join();
#endif
    RS_INFO << "Driver Stop" << RS_REND;
    return -1;
  }
  RS_DEBUG << "RoboSense Lidar-Driver Linux usb demo start......" << RS_REND;

  std::this_thread::sleep_for(std::chrono::seconds(2));
  DeviceInfo device_info;
  RS_DEBUG << "Get device info..." << RS_REND;
  if (driver.getDeviceInfo(device_info))
  {
    RS_INFO << "DEVICE_ID: " << device_info.device_id << RS_REND;
    for (auto it = device_info.calib_params.begin(); it != device_info.calib_params.end(); it++)
    {
      RS_INFO << std::fixed << std::setprecision(6) << it->first << " : " << it->second << RS_REND;
    }
    RS_INFO << "CALIB_PARAMS_STR: " << device_info.calib_params_str << RS_REND;
  }
  else
  {
    RS_ERROR << "Get device info failed" << RS_REND;
  }
  
#ifdef ORDERLY_EXIT
  const int MAX_WAIT_SECONDS = 15;
#else
  const int MAX_WAIT_SECONDS = 2147483647;  // INT_MAX
#endif

  int wait_seconds = 0;
  while (!g_should_exit && wait_seconds < MAX_WAIT_SECONDS)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    wait_seconds++;
  }
  RS_DEBUG << "RoboSense Lidar-Driver Linux usb demo exit......" << RS_REND;
  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();
#if ENABLE_IMU_PARSE
  imuData_handle_thread.join();
#endif
#if ENABLE_IMAGE_PARSE
  imageData_handle_thread.join();
#endif
  return 0;
}
