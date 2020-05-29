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
#include <rs_driver/msg/lidar_pointcloud_msg.h>
#include <rs_driver/msg/lidar_packet_msg.h>
#include <rs_driver/msg/lidar_scan_msg.h>
#include <rs_driver/utility/lock_queue.h>
#include <rs_driver/utility/thread_pool.hpp>
#include <rs_driver/utility/time.h>
#include <rs_driver/utility/error_code.h>
#include <rs_driver/driver/input.hpp>
#include <rs_driver/driver/decoder/decoder_factory.hpp>

namespace robosense
{

  namespace lidar
  {

    template <typename PointT>
    class LidarDriver
    {

    public:
      LidarDriver() = default;

      ~LidarDriver()
      {
        stop();
      }

      inline void init(const RSLiDAR_Driver_Param &param)
      {
        driver_param_ = param;
        lidar_decoder_ptr_ = DecoderFactory<PointT>::createDecoder(driver_param_.lidar_type, driver_param_.decoder_param);
        lidar_decoder_ptr_->loadCalibrationFile(driver_param_.calib_path);
        lidar_input_ptr_ = std::make_shared<Input>(driver_param_.lidar_type, driver_param_.input_param, std::bind(&LidarDriver::reportError, this, std::placeholders::_1));
        lidar_input_ptr_->regRecvMsopCallback(std::bind(&LidarDriver::msopCallback, this, std::placeholders::_1));
        lidar_input_ptr_->regRecvDifopCallback(std::bind(&LidarDriver::difopCallback, this, std::placeholders::_1));
        thread_pool_ptr_ = std::make_shared<ThreadPool>();
        pointcloud_ptr_ = typename LidarPointcloudMsg<PointT>::PointCloudPtr(new typename LidarPointcloudMsg<PointT>::PointCloud);
        scan_ptr_ = std::make_shared<LidarScanMsg>();
        thread_flag_ = false;
        difop_flag_ = false;
        points_seq_ = 0;
        scan_seq_ = 0;
      }

      inline void start()
      {
        lidar_input_ptr_->start();
      }

      inline void stop()
      {
        msop_pkt_queue_.clear();
        difop_pkt_queue_.clear();
      }

      inline void regPointRecvCallback(const std::function<void(const LidarPointcloudMsg<PointT> &)> _cb)
      {
        pointscb_.emplace_back(_cb);
      }

      inline void regRecvCallback(const std::function<void(const LidarScanMsg &)> _cb)
      {
        pkts_msop_cb_.emplace_back(_cb);
      }

      inline void regRecvCallback(const std::function<void(const LidarPacketMsg &)> _cb)
      {
        pkts_difop_cb_.emplace_back(_cb);
      }

      inline void regExceptionCallback(const std::function<void(const Error &)> _excb)
      {
        excb_.emplace_back(_excb);
      }

      inline void decodeMsopScan(const LidarScanMsg &pkt_scan_msg, LidarPointcloudMsg<PointT> &point_msg)
      {
        if (!difop_flag_)
        {
          reportError(ErrCode_NoDifopRecv);
          usleep(100000);
          return;
        }
        std::vector<std::vector<PointT>> point_vvec;
        int height = 1;
        typename LidarPointcloudMsg<PointT>::PointCloudPtr output_pointcloud_ptr = typename LidarPointcloudMsg<PointT>::PointCloudPtr(new typename LidarPointcloudMsg<PointT>::PointCloud);
        point_vvec.resize(pkt_scan_msg.packets.size());
#pragma omp parallel for
        for (uint32_t i = 0; i < pkt_scan_msg.packets.size(); i++)
        {
          std::vector<PointT> point_vec;
          int ret = lidar_decoder_ptr_->processMsopPkt(pkt_scan_msg.packets[i].packet.data(), point_vec, height);
          if (ret == E_DECODE_OK || ret == E_FRAME_SPLIT)
          {
            point_vvec[i] = std::move(point_vec);
          }
        }
        for (auto iiter : point_vvec)
        {
          for (auto iter = iiter.cbegin(); iter != iiter.cend(); iter++)
          {
            output_pointcloud_ptr->push_back(*iter);
          }
        }
        point_msg.pointcloud_ptr = output_pointcloud_ptr;
        point_msg.height = height;
        point_msg.width = point_msg.pointcloud_ptr->size() / point_msg.height;
        preparePointsMsg(point_msg);
        point_msg.timestamp = pkt_scan_msg.timestamp;
        if (point_msg.pointcloud_ptr->size() == 0)
        {
          reportError(ErrCode_ZeroPoints);
        }
      }

      void decodeDifopPkt(const LidarPacketMsg &pkt_msg)
      {
        lidar_decoder_ptr_->processDifopPkt(pkt_msg.packet.data());
        difop_flag_ = true;
      }

    private:
      inline void runCallBack(const LidarScanMsg &scan_msg)
      {
        if (scan_msg.seq != 0)
        {
          for (auto &it : pkts_msop_cb_)
          {
            it(scan_msg);
          }
        }
      }

      inline void runCallBack(const LidarPacketMsg &pkts_msg)
      {
        for (auto &it : pkts_difop_cb_)
        {
          it(pkts_msg);
        }
      }

      inline void runCallBack(const LidarPointcloudMsg<PointT> &points_msg)
      {
        if (points_msg.seq != 0)
        {
          for (auto &it : pointscb_)
          {
            it(points_msg);
          }
        }
      }

      inline void reportError(const Error &error)
      {
        for (auto &it : excb_)
        {
          it(error);
        }
      }

      void msopCallback(const LidarPacketMsg &msg)
      {
        LidarPacketMsg pkt_msg = msg;
        msop_pkt_queue_.push(pkt_msg);
        if (msop_pkt_queue_.is_task_finished.load())
        {
          msop_pkt_queue_.is_task_finished.store(false);
          thread_pool_ptr_->commit([this]() { processMsop(); });
        }
      }

      void difopCallback(const LidarPacketMsg &msg)
      {
        LidarPacketMsg pkt_msg = msg;
        difop_pkt_queue_.push(pkt_msg);
        if (difop_pkt_queue_.is_task_finished.load())
        {
          difop_pkt_queue_.is_task_finished.store(false);
          thread_pool_ptr_->commit([this]() { processDifop(); });
        }
      }

      void processMsop()
      {
        if (!difop_flag_)
        {
          reportError(ErrCode_NoDifopRecv);
          msop_pkt_queue_.clear();
          msop_pkt_queue_.is_task_finished.store(true);
          usleep(100000);
          return;
        }
        while (msop_pkt_queue_.m_quque.size() > 0)
        {
          LidarPacketMsg pkt = msop_pkt_queue_.m_quque.front();
          scan_ptr_->packets.emplace_back(pkt);
          msop_pkt_queue_.pop();
          std::vector<PointT> point_vec;
          int height = 1;
          int ret = lidar_decoder_ptr_->processMsopPkt(pkt.packet.data(), point_vec, height);
          if (ret == E_DECODE_OK || ret == E_FRAME_SPLIT)
          {
            for (auto iter = point_vec.cbegin(); iter != point_vec.cend(); iter++)
            {
              pointcloud_ptr_->push_back(*iter);
            }
            if (ret == E_FRAME_SPLIT)
            {
              LidarPointcloudMsg<PointT> msg(pointcloud_ptr_);
              msg.height = height;
              msg.width = pointcloud_ptr_->size() / msg.height;
              preparePointsMsg(msg);
              if (driver_param_.use_lidar_clock == true)
              {
                msg.timestamp = lidar_decoder_ptr_->getLidarTime(pkt.packet.data());
              }
              if (msg.pointcloud_ptr->size() == 0)
              {
                reportError(ErrCode_ZeroPoints);
              }
              else
              {
                runCallBack(msg);
              }
              pointcloud_ptr_.reset(new typename LidarPointcloudMsg<PointT>::PointCloud);
              prepareLidarScanMsg(*scan_ptr_);
              runCallBack(*scan_ptr_);
              scan_ptr_.reset(new LidarScanMsg);
            }
          }
        }
        msop_pkt_queue_.is_task_finished.store(true);
      }

      void processDifop()
      {
        while (difop_pkt_queue_.m_quque.size() > 0)
        {
          LidarPacketMsg pkt = difop_pkt_queue_.m_quque.front();
          difop_pkt_queue_.pop();
          decodeDifopPkt(pkt);
          runCallBack(pkt);
        }
        difop_pkt_queue_.is_task_finished.store(true);
      }

      void prepareLidarScanMsg(LidarScanMsg &msg)
      {
        msg.timestamp = getTime();
        if (driver_param_.use_lidar_clock == true)
        {
          msg.timestamp = lidar_decoder_ptr_->getLidarTime(msg.packets.back().packet.data());
        }
        msg.seq = scan_seq_++;
        msg.parent_frame_id = driver_param_.frame_id;
        msg.frame_id = driver_param_.frame_id;
      }

      void preparePointsMsg(LidarPointcloudMsg<PointT> &msg)
      {
        msg.timestamp = getTime();
        msg.seq = points_seq_++;
        msg.parent_frame_id = driver_param_.frame_id;
        msg.frame_id = driver_param_.frame_id;
        msg.is_dense = false;
        msg.is_transform = false;
        msg.is_motion_correct = false;
      }

    private:
      Queue<LidarPacketMsg> msop_pkt_queue_;
      Queue<LidarPacketMsg> difop_pkt_queue_;
      std::vector<std::function<void(const LidarScanMsg &)>> pkts_msop_cb_;
      std::vector<std::function<void(const LidarPacketMsg &)>> pkts_difop_cb_;
      std::vector<std::function<void(const LidarPointcloudMsg<PointT> &)>> pointscb_;
      std::vector<std::function<void(const Error &)>> excb_;
      std::shared_ptr<std::thread> lidar_thread_ptr_;
      std::shared_ptr<DecoderBase<PointT>> lidar_decoder_ptr_;
      std::shared_ptr<Input> lidar_input_ptr_;
      std::shared_ptr<ThreadPool> thread_pool_ptr_;
      std::shared_ptr<LidarScanMsg> scan_ptr_;
      uint32_t scan_seq_;
      uint32_t points_seq_;
      bool thread_flag_;
      bool difop_flag_;
      RSLiDAR_Driver_Param driver_param_;
      typename LidarPointcloudMsg<PointT>::PointCloudPtr pointcloud_ptr_;
    };

  } // namespace lidar
} // namespace robosense
