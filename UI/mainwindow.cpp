#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    demo_ptr = std::make_shared<robosense::sensor::SensorManager>();
    YAML::Node config = yp.loadFile((std::string)PROJECT_PATH + "/conf/config.yaml");
    std::string config_path = (std::string)PROJECT_PATH + "/conf"; ///< the absolute path of config file
    demo_ptr->init(config, config_path);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Start_clicked()
{
    demo_ptr->start();
    TITLE << "Robosense-LiDAR-Driver is running....." << REND;
    std::thread t([](){ros::spin();});
    t.detach();
}

void MainWindow::on_Stop_clicked()
{
        demo_ptr->stop();

}

void MainWindow::on_RVIZ_clicked()
{
}
