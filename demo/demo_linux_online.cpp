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

#include "interface/lidar_interface.h"
using namespace robosense::lidar;
bool start_ = true;
struct PointXYZI
{
    double x;
    double y;
    double z;
    double intensity;
};
std::shared_ptr<LidarDriverInterface<PointXYZI>> demo_ptr;

void callback(const LidarPointcloudMsg<PointXYZI> &msg)
{
    std::cout << "msg: " << msg.seq << "pointcloud size: " << msg.cloudPtr->size() << std::endl;
}

int main(int argc, char *argv[])
{
    demo_ptr = std::make_shared<LidarDriverInterface<PointXYZI>>();
    RSLiDAR_Driver_Param param;
    param.input_param.msop_port = 6699;
    param.input_param.difop_port = 7788;
    param.calib_path = "/home/xzd/work/lidar_driver/parameter";
    param.lidar_type = LiDAR_TYPE::RS128;
    demo_ptr->init(param);
    demo_ptr->regPointRecvCallback(callback);
    demo_ptr->start();
    std::cout << "Robosense Lidar-Driver Linux online demo start......" << std::endl;
    while (start_)
    {
        sleep(1);
    }
}
