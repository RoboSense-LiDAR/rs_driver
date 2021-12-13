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
#include <rs_driver/driver/lidar_driver_impl.hpp>

namespace robosense
{
namespace lidar
{

std::string getDriverVersion();

/**
 * @brief This is the RoboSense LiDAR driver interface class
 */
template <typename T_PointCloud>
class LidarDriver
{
public:

  /**
   * @brief Constructor, instanciate the driver pointer
   */
  LidarDriver() 
    : driver_ptr_(std::make_shared<LidarDriverImpl<T_PointCloud>>())
  {
  }

  /**
   * @brief The initialization function, used to set up parameters and instance objects,
   *        used when get packets from online lidar or pcap
   * @param param The custom struct RSDriverParam
   * @return If successful, return true; else return false
   */
  inline bool init(const RSDriverParam& param)
  {
    return driver_ptr_->init(param);
  }

  /**
   * @brief Start the thread to receive and decode packets
   * @return If successful, return true; else return false
   */
  inline bool start()
  {
    return driver_ptr_->start();
  }

  /**
   * @brief Stop all threads
   */
  inline void stop()
  {
    driver_ptr_->stop();
  }

  /**
   * @brief Register the lidar point cloud callback function to driver. When point cloud is ready, this function will be
   * called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put)
  {
    driver_ptr_->regRecvCallback(cb_get, cb_put);
  }

  /**
   * @brief Register the exception message callback function to driver. When error occurs, this function will be called
   * @param callback The callback function
   */
  inline void regExceptionCallback(const std::function<void(const Error&)>& callback)
  {
    driver_ptr_->regExceptionCallback(callback);
  }

  /**
   * @brief Register the lidar difop packet message callback function to driver. When lidar difop packet message is
   * ready, this function will be called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const uint8_t*, size_t)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Decode lidar msop/difop messages
   * @param pkt_msg The lidar msop/difop packet
   */
  inline void decodePacket(const uint8_t* pkt, size_t size)
  {
    driver_ptr_->decodePacket(pkt, size);
  }

  /**
   * @brief Get the current lidar temperature
   * @param temp The variable to store lidar temperature
   * @return if get temperature successfully, return true; else return false
   */
  inline bool getTemperature(float& temp)
  {
    return driver_ptr_->getTemperature(temp);
  }

private:
  std::shared_ptr<LidarDriverImpl<T_PointCloud>> driver_ptr_;  ///< The driver pointer
};

}  // namespace lidar
}  // namespace robosense
