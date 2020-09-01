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

#include "rs_driver/api/lidar_driver.h"
using namespace robosense::lidar;

struct PointXYZI  ///< user defined point type
{
  float x;
  float y;
  float z;
  float intensity;
};

/**
 * @brief The point cloud callback function. This function will be registered to lidar driver.
 *              When the point cloud message is ready, driver can send out messages through this function.
 * @param msg  The lidar point cloud message.
 */
void pointCloudCallback(const PointCloudMsg<PointXYZI>& msg)
{
  /* Note: Please do not put time-consuming operations in the callback function! */
  /* Make a copy of the message and process it in another thread is recommended*/
  RS_MSG << "msg: " << msg.seq << " point cloud size: " << msg.point_cloud_ptr->size() << RS_REND;
}

/**
 * @brief The exception callback function. This function will be registered to lidar driver.
 * @param code The error code struct.
 */
void exceptionCallback(const Error& code)
{
  /* Note: Please do not put time-consuming operations in the callback function! */
  /* Make a copy of the error message and process it in another thread is recommended*/
  RS_WARNING << "Error code : " << code.toString() << RS_REND;
}

int main(int argc, char* argv[])
{
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Core Version: V " << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR << "."
        << RSLIDAR_VERSION_PATCH << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;

  LidarDriver<PointXYZI> driver;  ///< Declare the driver object

  RSDriverParam param;                                             ///< Create a parameter object
  param.input_param.read_pcap = true;                              ///< Set read_pcap to true
  param.input_param.pcap_path = "/home/robosense/rs16.pcap";  ///< Set the pcap file directory
  param.input_param.device_ip = "192.168.1.200";  ///< Set the lidar ip address, the default is 192.168.1.200
  param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
  param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
  param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct
  param.print();

  driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function into the driver
  driver.regRecvCallback(pointCloudCallback);      ///< Register the point cloud callback function into the driver
  if (!driver.init(param))                         ///< Call the init function and pass the parameter
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }
  driver.start();  ///< The driver thread will start
  RS_DEBUG << "RoboSense Lidar-Driver Linux pcap demo start......" << RS_REND;

  while (true)
  {
    sleep(1);
  }

  return 0;
}
