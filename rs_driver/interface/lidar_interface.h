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
#pragma once
#include <rs_driver/driver/lidar_driver.hpp>

namespace robosense
{
  namespace lidar
  {
    /**
     * @description:  This is the Robosense LiDAR driver interface class.
     *                 Now support RS16,RS32, RSBP & RS128.
     */
    template <typename PointT>
    class LidarDriverInterface
    {
    public:
      /**
       * @description: Constructor, instanciate the driver pointer.
       * @param Null 
       * @return: Null
       */
      LidarDriverInterface() { driver_ptr_ = std::make_shared<LidarDriver<PointT>>(); };
      /**
       * @description: Deconstructor, stop all threads
       * @param Null
       * @return: Null
       */
      ~LidarDriverInterface() { stop(); }
      /**
       * @description: The initialize function, used to set the realated parameters and instance objects
       * @param The struct->RSLiDAR_Driver_Param 
       * @return: Null
       */
      inline void init(const RSLiDAR_Driver_Param &param)
      {
        driver_ptr_->init(param);
      }
      /**
       * @description: Used to start the thread to receive packets, and decode packets
       * @param Null
       * @return: Null
       */
      inline void start()
      {
        driver_ptr_->start();
      }
      /**
       * @description: Used to stop all threads
       * @param Null 
       * @return: Null
       */
      inline void stop()
      {
        driver_ptr_->stop();
      }
      /**
       * @description: Used to register the lidar point cloud callback function.
       *  When pointcloud is prepared, this function will be called.
       * @param callBack the callback funtion  
       * @return: Null
       */
      inline void regPointRecvCallback(const std::function<void(const PointcloudMsg<PointT> &)> callBack)
      {
        driver_ptr_->regPointRecvCallback(callBack);
      }
      /**
       * @description: Used to register the lidar scan message callback funtion. 
       * When lidar scan message is ready, this function will be called.
       * @param callBack the callback funtion  
       * @return: Null
       */
      inline void regRecvCallback(const std::function<void(const ScanMsg &)> callBack)
      {
        driver_ptr_->regRecvCallback(callBack);
      }
      /**
       * @description: Used to register the lidar difop packet message callback funtion. 
       * When lidar difop packet message is ready, this function will be called.
       * @param callBack the callback funtion  
       * @return: Null
       */
      inline void regRecvCallback(const std::function<void(const PacketMsg &)> callBack)
      {
        driver_ptr_->regRecvCallback(callBack);
      }
      /**
       * @description: Used to register the exception message callback funtion. 
       * When error occurs, this function will be called.
       * @param excallBack The callback funtion  
       * @return: Null
       */
      inline void regExceptionCallback(const std::function<void(const Error &)> excallBack)
      {
        driver_ptr_->regExceptionCallback(excallBack);
      }
      /**
       * @description: Used to decode the scan message. Can be called when processing offline lidar message.
       **NOTE** This function will only work after decodeDifopPkt is called,
                because the driver need difop packet to help to decode scan message.
       * @param pkt_scan_msg The lidar scan message used to be decode
       * @param point_msg The output point cloud message
       * @return: Null
       */
      inline void decodeMsopScan(const ScanMsg &pkt_scan_msg, PointcloudMsg<PointT> &point_msg)
      {
        driver_ptr_->decodeMsopScan(pkt_scan_msg, point_msg);
      }
      /**
       * @description: Used to decode the lidar difop message. **Must** be called when processing offline lidar message.
       * @param pkt_msg The lidar difop packet 
       * @return: Null
       */
      inline void decodeDifopPkt(const PacketMsg &pkt_msg)
      {
        driver_ptr_->decodeDifopPkt(pkt_msg);
      }

    private:
      std::shared_ptr<LidarDriver<PointT>> driver_ptr_; ///< The driver pointer
    };

  } // namespace lidar
} // namespace robosense
