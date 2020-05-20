#include "stdafx.h"
#include <boost\asio.hpp>
#include "driver/lidar_driver.hpp"
#include "msg/lidar_points_msg.h"
#include <Windows.h>
#include <iostream>



bool start_=true;
struct PointXYZI
{
    double x;
    double y;
    double z;
    double intensity;
};

void callback(const robosense::LidarPointsMsg<PointXYZI> &msg)
{
#if 0
    sensor_msgs::PointCloud2 ros_msg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto iter : *msg.cloudPtr)
    {
        cloud2->push_back(iter);
    }

    pcl::toROSMsg(*cloud2, ros_msg);

    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(msg.timestamp);
    ros_msg.header.frame_id = msg.parent_frame_id;
    ros_msg.header.seq = msg.seq;

    lidar_points_pub_.publish(ros_msg);
#endif
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
    std::shared_ptr<robosense::sensor::LidarDriver<PointXYZI>> demo_ptr = std::make_shared<robosense::sensor::LidarDriver<PointXYZI>>();
    robosense::sensor::RSLiDAR_Driver_Param param;
//    param.input_param.read_pcap = true;
    param.device_type = "RS16";
 //   param.input_param.pcap_file_dir = "/media/xzd/bag/bag/sunnyvael_1014.pcap";
 //   param.calib_path = "/home/xzd/work/lidar_driver/conf";
    demo_ptr->init(param);
	demo_ptr->regPointRecvCallback(callback);
    demo_ptr->start();
    while (start_)
    {
        Sleep(1);
    }
}
