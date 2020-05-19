#include "driver/lidar_driver.hpp"
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <signal.h>

ros::Publisher lidar_points_pub_;
bool start_=true;
struct PointXYZI
{
    double x;
    double y;
    double z;
    double intensity;
};

void callback(const robosense::LidarPointsMsg<pcl::PointXYZI> &msg)
{

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

    DEBUG << "msg: " << msg.seq << REND;
}

static void sigHandler(int sig)
{
    start_ = false;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, sigHandler); ///< bind the ctrl+c signal with the the handler function
    ros::init(argc, argv, "driver", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh_;
    lidar_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points", 10);
    std::shared_ptr<robosense::sensor::LidarDriver<pcl::PointXYZI>> demo_ptr = std::make_shared<robosense::sensor::LidarDriver<pcl::PointXYZI>>();
    robosense::sensor::RSLiDAR_Driver_Param param;
    param.input_param.read_pcap = true;
    param.device_type = "RS128";
    param.input_param.pcap_file_dir = "/media/xzd/bag/bag/sunnyvael_1014.pcap";
    param.calib_path = "/home/xzd/work/lidar_driver/conf";
    demo_ptr->init(param);
    demo_ptr->regRecvCallback(callback);
    demo_ptr->start();
    while (start_)
    {
        sleep(1);
    }
}
