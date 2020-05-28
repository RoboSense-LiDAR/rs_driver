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
#include "common/common_header.h"
namespace robosense
{
  namespace lidar
  {
#ifdef _MSC_VER
#pragma pack(push, 2)
#endif
    template <typename PointT>
#ifdef _MSC_VER
    struct LidarPointcloudMsg
#elif __GNUC__
    struct alignas(16) LidarPointcloudMsg
#endif
    {
      typedef std::vector<PointT> PointCloud;
      typedef std::shared_ptr<PointCloud> PointCloudPtr;
      typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;

      double timestamp = 0.0;
      uint32_t seq = 0;
      std::string parent_frame_id = "";
      std::string frame_id = "";
      uint32_t height = 0;
      uint32_t width = 0;
      bool is_dense = false;
      bool is_transform = false;
      bool is_motion_correct = false;
      PointCloudPtr cloudPtr;

      LidarPointcloudMsg() = default;
      LidarPointcloudMsg(const PointCloudPtr &pointptr) : cloudPtr(pointptr)
      {
      }
      typedef std::shared_ptr<LidarPointcloudMsg> Ptr;
      typedef std::shared_ptr<const LidarPointcloudMsg> ConstPtr;
    };
#ifdef _MSC_VER
#pragma pack(pop)
#endif
  } // namespace lidar
} // namespace robosense
