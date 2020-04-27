#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
#ifdef ROS_FOUND
    ros::init(argc, argv, "lidara_driver", ros::init_options::NoSigintHandler); ///< if use_ros is true, ros::init() will be called
#endif
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
