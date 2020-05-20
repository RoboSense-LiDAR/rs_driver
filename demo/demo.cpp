#ifndef __GNUC__
#include "stdafx.h"
#include <Windows.h>
#include <boost\asio.hpp>
#endif
#include "driver/lidar_driver.hpp"
#include "msg/lidar_points_msg.h"
#include <iostream>
// #include <ros/ros.h>
// #include <ros/publisher.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
//ros::Publisher lidar_points_pub_;

bool start_ = true;
struct PointXYZI
{
    double x;
    double y;
    double z;
    double intensity;
};

void callback(const robosense::LidarPointsMsg<PointXYZI> &msg)
{
    // sensor_msgs::PointCloud2 ros_msg;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    // for (auto iter : *msg.cloudPtr)
    // {
    //     cloud2->push_back(iter);
    // }

    // pcl::toROSMsg(*cloud2, ros_msg);

    // ros_msg.header.stamp = ros_msg.header.stamp.fromSec(msg.timestamp);
    // ros_msg.header.frame_id = msg.parent_frame_id;
    // ros_msg.header.seq = msg.seq;

    // lidar_points_pub_.publish(ros_msg);
    std::cout << "mkkkkkkkkkkkksg: " << msg.seq << std::endl;
}
/*
static void sigHandler(int sig)
{
    start_ = false;
}
*/
int main(int argc, char *argv[])
{
    //    ros::init(argc, argv, "driver", ros::init_options::NoSigintHandler);
    // ros::NodeHandle nh_;
    // lidar_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points", 10);
    std::shared_ptr<robosense::sensor::LidarDriver<PointXYZI>> demo_ptr = std::make_shared<robosense::sensor::LidarDriver<PointXYZI>>();
    robosense::sensor::RSLiDAR_Driver_Param param;
    param.calib_path="/home/xzd/work/lidar_driver/conf";
    param.device_type = "RS128";
    demo_ptr->init(param);
    demo_ptr->regPointRecvCallback(callback);
    demo_ptr->start();
    while (start_)
    {
#ifdef __GNUC__
        sleep(1);
#else
        Sleep(1);
#endif
    }
}
