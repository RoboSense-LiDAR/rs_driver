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

#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "interface/lidar_interface.h"
using namespace robosense::lidar;
ros::Publisher lidar_points_pub_;
bool start_ = true;
std::shared_ptr<LidarDriverInterface<pcl::PointXYZI>> demo_ptr;
void callback(const LidarPointsMsg<pcl::PointXYZI> &msg)
{
    sensor_msgs::PointCloud2 ros_msg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto iter : *msg.cloudPtr)
    {
        cloud2->push_back(std::move(iter));
    }
    cloud2->height =msg.height;
    cloud2->width =msg.width;
    pcl::toROSMsg(*cloud2, ros_msg);
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(msg.timestamp);
    ros_msg.header.frame_id = msg.parent_frame_id;
    ros_msg.header.seq = msg.seq;
    lidar_points_pub_.publish(ros_msg);
    std::cout << "msg: " << msg.seq << "pointcloud size: " << msg.cloudPtr->size() << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "driver", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh_;
    lidar_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("rslidar_points", 10);
    demo_ptr = std::make_shared<LidarDriverInterface<pcl::PointXYZI>>();
    RSLiDAR_Driver_Param param;
    param.input_param.read_pcap = true;
    param.input_param.msop_port = 6699;
    param.input_param.difop_port = 7788;
    param.input_param.pcap_file_dir = "/home/xzd/Downloads/30m.pcap";
    //param.calib_path = "/home/xzd/work/lidar_driver/parameter";
    param.device_type = "RS128";
    demo_ptr->init(param);
    demo_ptr->regPointRecvCallback(callback);
    demo_ptr->start();
    while (start_)
    {
        sleep(1);
    }
}
