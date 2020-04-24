#include <rs_common/common.h>
#include <manager/sensor_manager.h>
#include <signal.h>
using namespace robosense::sensor;
bool start_ = true;
/**
 * @brief  signal handler
 * @note   will be called if receive ctrl+c signal from keyboard during the progress
 *         (all the threads in progress will be stopped and the progress end)
 * @param  sig: the input signal
 * @retval None
 */
static void sigHandler(int sig)
{
#ifdef ROS_FOUND
    ros::shutdown();
#endif
    start_ = false;
}

int main(int argc, char **argv)
{
    std::shared_ptr<SensorManager> demo_ptr = std::make_shared<SensorManager>();
    robosense::common::YamlParser yp;
    YAML::Node config = yp.loadFile((std::string)PROJECT_PATH + "/conf/config.yaml");
    std::string config_path = (std::string)PROJECT_PATH + "/conf"; ///< the absolute path of config file
    signal(SIGINT, sigHandler);                                    ///< bind the ctrl+c signal with the the handler function
#ifdef ROS_FOUND
    ros::init(argc, argv, "rs_sdk_demo", ros::init_options::NoSigintHandler); ///< if use_ros is true, ros::init() will be called
#endif
    demo_ptr->init(config, config_path);
    demo_ptr->start();
    TITLE<<"Robosense-LiDAR-Driver is running....."<<REND;
#ifdef ROS_FOUND
    ros::spin();
#else
    while (start_)
    {
        sleep(1);
    }
#endif
    demo_ptr.reset();
    return 0;
}