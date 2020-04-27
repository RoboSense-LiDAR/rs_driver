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
#include "common/msg/rs_msg/gnss_msg.h"
#include "common/msg/rs_msg/imu_msg.h"
#include "common/msg/rs_msg/odom_msg.h"
#include "common/msg/rs_msg/lidar_packet_msg.h"
#include "common/msg/rs_msg/lidar_points_msg.h"
#include "common/msg/rs_msg/lidar_scan_msg.h"
namespace robosense
{
namespace common
{
class msgPrinter
{
public:
    inline void printMsg(const ImuMsg &msg)
    {
        DEBUG << "*****************************************" << REND;
        DEBUG << "ImuMsg  seq: " << msg.seq << REND;
        DEBUG << "timestamp: " << msg.timestamp << REND;
        DEBUG << "frame id: " << msg.frame_id << REND;
        DEBUG << "parent frame id: " << msg.parent_frame_id << REND;
        DEBUG << "orien: " << msg.orien[0] << "   " << msg.orien[1] << "   " << msg.orien[2] << REND;
        DEBUG << "angular_vel: " << msg.angular_vel[0] << "   " << msg.angular_vel[1] << "   " << msg.angular_vel[2] << REND;
        DEBUG << "acc: " << msg.acc[0] << "   " << msg.acc[1] << "   " << msg.acc[2] << REND;
        DEBUG << "++++++++++++++++++++++++++++++++++++++++++" << REND;
    }
    inline void printMsg(const GnssMsg &msg)
    {
        DEBUG << "*****************************************" << REND;
        DEBUG << "GnssMsg  seq: " << msg.seq << REND;
        DEBUG << "timestamp: " << msg.timestamp << REND;
        DEBUG << "frame id: " << msg.frame_id << REND;
        DEBUG << "parent frame id: " << msg.parent_frame_id << REND;
        DEBUG << "type: " << msg.type << REND;
        DEBUG << "sat_cnt: " << msg.sat_cnt << REND;
        DEBUG << "pos: " << msg.pos[0] << "   " << msg.pos[1] << "   " << msg.pos[2] << REND;
        DEBUG << "orien: " << msg.orien[0] << "   " << msg.orien[1] << "   " << msg.orien[2] << REND;
        DEBUG << "linear_vel: " << msg.linear_vel[0] << "   " << msg.linear_vel[1] << "   " << msg.linear_vel[2] << REND;
        DEBUG << "++++++++++++++++++++++++++++++++++++++++++" << REND;
    }

    inline void printMsg(const OdomMsg &msg)
    {
        DEBUG << "*****************************************" << REND;
        DEBUG << "OdomMsg  seq: " << msg.seq << REND;
        DEBUG << "timestamp: " << msg.timestamp << REND;
        DEBUG << "frame id: " << msg.frame_id << REND;
        DEBUG << "parent frame id: " << msg.parent_frame_id << REND;
        DEBUG << "linear_vel: " << msg.linear_vel[0] << "   " << msg.linear_vel[1] << "   " << msg.linear_vel[2] << REND;
        DEBUG << "++++++++++++++++++++++++++++++++++++++++++" << REND;
    }

    inline void printMsg(const LidarPointsMsg &msg)
    {
        DEBUG << "*****************************************" << REND;
        DEBUG << "LidarPointsMsg  seq: " << msg.seq << REND;
        DEBUG << "timestamp: " << msg.timestamp << REND;
        DEBUG << "frame id: " << msg.frame_id << REND;
        DEBUG << "parent frame id: " << msg.parent_frame_id << REND;
        DEBUG << "motion correct: " << msg.motion_correct << REND;
        DEBUG << "points size: " << msg.cloudPtr->size() << REND;
        DEBUG << "++++++++++++++++++++++++++++++++++++++++++" << REND;
    }
};
} // namespace common
} // namespace robosense
