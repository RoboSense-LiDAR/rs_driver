#include "stdafx.h"
#include <boost\asio.hpp>
#include "driver/lidar_driver.hpp"
#include "msg/lidar_points_msg.h"
//#ifdef _MSC_VER

#include <Windows.h>
//#endif

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

 //   DEBUG << "msg: " << msg.seq << REND;
}
/*
static void sigHandler(int sig)
{
    start_ = false;
}
*/
int main(int argc, char *argv[])
{
//	boost::asio::io_service io;
//	std::cout << "hello, boost asio world!" << std::endl;
 //   signal(SIGINT, sigHandler); ///< bind the ctrl+c signal with the the handler function
    std::shared_ptr<robosense::sensor::LidarDriver<PointXYZI>> demo_ptr = std::make_shared<robosense::sensor::LidarDriver<PointXYZI>>();
    robosense::sensor::RSLiDAR_Driver_Param param;
    param.input_param.read_pcap = true;
    param.device_type = "RS128";
    param.input_param.pcap_file_dir = "/media/xzd/bag/bag/sunnyvael_1014.pcap";
    param.calib_path = "/home/xzd/work/lidar_driver/conf";
    demo_ptr->init(param);
	demo_ptr->regPointRecvCallback(callback);
    demo_ptr->start();
    while (start_)
    {
        Sleep(1);
    }
}
