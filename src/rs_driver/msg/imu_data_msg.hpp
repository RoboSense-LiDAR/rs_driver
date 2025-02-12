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

#include <vector>
#include <string>
namespace robosense
{
namespace lidar
{
struct ImuData {
    bool state;
    double timestamp;  // Time in nanoseconds
    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;
    ImuData()
        : state{false},
          timestamp(0),
          orientation_x(0.0),
          orientation_y(0.0),
          orientation_z(0.0),
          orientation_w(1.0),
          angular_velocity_x(0.0),
          angular_velocity_y(0.0),
          angular_velocity_z(0.0),
          linear_acceleration_x(0.0),
          linear_acceleration_y(0.0),
          linear_acceleration_z(0.0) {}

    // Parameterized constructor to initialize all members
    ImuData(bool valid, double ts, float ori_x, float ori_y, float ori_z, float ori_w,
            float ang_vel_x, float ang_vel_y, float ang_vel_z,
            float lin_acc_x, float lin_acc_y, float lin_acc_z)
        : state{valid},
          timestamp(ts),
          orientation_x(ori_x),
          orientation_y(ori_y),
          orientation_z(ori_z),
          orientation_w(ori_w),
          angular_velocity_x(ang_vel_x),
          angular_velocity_y(ang_vel_y),
          angular_velocity_z(ang_vel_z),
          linear_acceleration_x(lin_acc_x),
          linear_acceleration_y(lin_acc_y),
          linear_acceleration_z(lin_acc_z) {}
    ImuData& operator=(const ImuData& other) {
        if (this != &other) {
            state = other.state;
            timestamp = other.timestamp;
            orientation_x = other.orientation_x;
            orientation_y = other.orientation_y;
            orientation_z = other.orientation_z;
            orientation_w = other.orientation_w;
            angular_velocity_x = other.angular_velocity_x;
            angular_velocity_y = other.angular_velocity_y;
            angular_velocity_z = other.angular_velocity_z;
            linear_acceleration_x = other.linear_acceleration_x;
            linear_acceleration_y = other.linear_acceleration_y;
            linear_acceleration_z = other.linear_acceleration_z;
        }
        return *this;
    }
    void init()
    {
      state = false;
      timestamp = 0.0;
      orientation_x = 0.0;
      orientation_y = 0.0;
      orientation_z = 0.0;
      orientation_w = 1.0;
      angular_velocity_x = 0.0;
      angular_velocity_y = 0.0;
      angular_velocity_z = 0.0;
      linear_acceleration_x = 0.0;
      linear_acceleration_y = 0.0;
      linear_acceleration_z = 0.0;
    }
};
} // namespace lidar
} // namespace robosense