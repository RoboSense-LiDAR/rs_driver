/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
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
#ifdef ROS_FOUND
#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "msg/lidar_points_msg.h"


namespace robosense
{


/************************************************************************/
/**Translation functions between Robosense message and ROS message**/
/************************************************************************/

// inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointsMsg &rs_msg)
// {
//     sensor_msgs::PointCloud2 ros_msg;
//     pcl::toROSMsg(*rs_msg.cloudPtr, ros_msg);
//     ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
//     ros_msg.header.frame_id = rs_msg.parent_frame_id;
//     ros_msg.header.seq = rs_msg.seq;
//     return ros_msg;
// }



} // namespace robosense
#endif // ROS_FOUND
