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
#include "driver/lidar_driver.hpp"

namespace robosense
{
  namespace lidar
  {
    template <typename PointT>
    class LidarDriverInterface
    {
    public:
      LidarDriverInterface() { driver_ptr_ = std::make_shared<LidarDriver<PointT>>(); };
      ~LidarDriverInterface() { driver_ptr_->stop(); }
      inline void init(const RSLiDAR_Driver_Param &param)
      {
        driver_ptr_->init(param);
      }
      inline void start()
      {
        driver_ptr_->start();
      }
      inline void stop()
      {
        driver_ptr_->stop();
      }
      inline void regPointRecvCallback(const std::function<void(const LidarPointsMsg<PointT> &)> callBack)
      {
        driver_ptr_->regPointRecvCallback(callBack);
      }
      inline void regRecvCallback(const std::function<void(const LidarScanMsg &)> callBack)
      {
        driver_ptr_->regRecvCallback(callBack);
      }
      inline void regRecvCallback(const std::function<void(const LidarPacketMsg &)> callBack)
      {
        driver_ptr_->regRecvCallback(callBack);
      }
      inline void regExceptionCallback(const std::function<void(const Error &)> excallBack)
      {
        driver_ptr_->regExceptionCallback(excallBack);
      }
      inline void decodeMsopScan(const LidarScanMsg &pkt_scan_msg, LidarPointsMsg<PointT> &point_msg)
      {
        driver_ptr_->decodeMsopScan(pkt_scan_msg, point_msg);
      }
      inline void decodeDifopPkt(const LidarPacketMsg &pkt_msg)
      {
        driver_ptr_->decodeDifopPkt(pkt_msg);
      }

    private:
      std::shared_ptr<LidarDriver<PointT>> driver_ptr_;
    };

  } // namespace lidar
} // namespace robosense
