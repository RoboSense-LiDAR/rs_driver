#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <common/common.h>
#include <manager/sensor_manager.h>

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_Start_clicked();

    void on_Stop_clicked();

    void on_RVIZ_clicked();

private:
    Ui::MainWindow *ui;

private:
    std::shared_ptr<robosense::sensor::SensorManager> demo_ptr;
    robosense::common::YamlParser yp;
};
#endif // MAINWINDOW_H
