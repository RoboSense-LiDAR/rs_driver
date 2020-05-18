#include "driver/lidar_driver.hpp"
#include <msg/ros_msg_translator.h>
#include <ros/ros.h>
#include <ros/publisher.h>
ros::Publisher lidar_points_pub_;

struct PointXYZI
{
    double x;
    double y;
    double z;
    double intensity;
};

void callback(const robosense::LidarPointsMsg<PointXYZI> &msg)
{
    //lidar_points_pub_.publish(toRosMsg(msg));
    DEBUG<<"msg: "<<msg.seq<<REND;
}

int main(int argc, char *argv[])
{
    // ros::init(argc,argv,"driver");
    // ros::NodeHandle nh_;
    // lidar_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points", 10);



    std::shared_ptr<robosense::sensor::LidarDriver<PointXYZI>> demo_ptr = std::make_shared<robosense::sensor::LidarDriver<PointXYZI>>();
    robosense::sensor::RSLiDAR_Driver_Param param;
    param.input_param.read_pcap = true;
    param.device_type = "RS128";
    param.input_param.pcap_file_dir = "/media/xzd/bag/bag/sunnyvael_1014.pcap";
    param.calib_path = "/home/xzd/work/lidar_driver/conf";
    demo_ptr->init(param);
    demo_ptr->regRecvCallback(callback);
    demo_ptr->start();
    while (true)
    {
        sleep(1);
    }
}
