**rs_driver 测试用例**

## 1 简介

这里的测试都在Linux系统下进行。

测试用例级别：
+ P1 基本功能。失败将导致多数重要功能无法运行。
+ P2 重要功能。系统中主要特性，失败则无法使用该特性。
+ P3 可选功能。系统中次要特性，失败无法使用该特性。

## 2 功能测试

## 2.1 编译运行rs_driver

| 级别   |     功能步骤简要说明                                     |  期望结果      |
|:------:|:--------------------------------------------------------|:--------------:|
| P1     | 安装libpcap/PCL/eigen库时，编译rs_driver                 | 编译成功       |
| P3     | ENALBE_PCAP_PARSE=OFF，不安装libpcap库时，编译rs_driver  | 编译成功       |
| P3     | ENABLE_TRANSFORM=OFF，不安装eigen库时，编译rs_driver     | 编译成功       |
| P3     | COMPILE_TOOLS=OFF，不安装PCL库时，编译rs_driver          | 编译成功       |
| P1     | 用rs_driver_viewer连接在线雷达                            | rs_driver_viewer显示点云  |
| P1     | 用demo_online连接在线雷达，点为XYZI/XYZIRT，打印点。       |  点属性正常   |

## 2.2 rs_driver输出点云的正确性

这里需要测试点云正确性的雷达包括：RS16/RS32/RSBP/RSHELIOS/RS80/RS128/RSM1。

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P2     | 用rs_driver_viewer依次连接每种雷达，雷达工作在单回波模式下  | rs_driver_viewer正常显示点云  |
| P2     | 用rs_driver_viewer依次连接每种雷达，雷达工作在双回波模式下  | rs_driver_viewer正常显示点云  |

## 2.3 rs_driver连接在线雷达

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P1     | 用demo_online连接在线雷达，雷达工作在广播模式下，         | demo_online正常打印点云         |
| P2     | 用demo_online连接在线雷达，雷达工作在组播模式下           | demo_online正常打印点云         |
| P2     | 用demo_online连接在线雷达，雷达工作在单播模式下           | demo_online正常打印点云         |
| P2     | 用demo_online连接在线雷达，雷达包带USER_LAYER              | demo_online正常打印点云       |
| P2     | 用demo_online连接在线雷达，雷达包带TAIL_LAYER              | demo_online正常打印点云       |
| P2     | 启动demo_online两个实例，连接两台雷达，雷达指定不同目标端口、相同目标IP | demo_online正常打印点云       |
| P2     | 启动demo_online两个实例，连接两台雷达，雷达指定相同目标端口、不同目标IP | demo_online正常打印点云       |
| P2     | 用demo_online连接在线雷达，雷达不做时间同步，user_lidar_clock=true（或false），打印点的时间戳 | 时间戳是雷达时间（或主机时间） |
| P2     | 用demo_online连接在线雷达，dense_points = true(或false)，打印点云大小 | 点云大小是（或不是）雷达线数的整数倍 |
| P3     | 用rs_driver_viewer连接在线雷达，设置错误的DIFOP端口，wait_for_difop=false | 显示扁平点云 |
| P3     | 用rs_driver_viewer接在线雷达，设置错误的DIFOP端口，wait_for_difop=true | 不显示点云 |
| P2     | 用rs_driver_viewer连接在线雷达，config_from_file=true，angle_path=angle.csv | 正常显示（非扁平的）点云 |
| P2     | 用rs_driver_viewer连接在线雷达，split_frame_mode=by_angle，split_angle=0/90/180/270  | 正常显示点云 |
| P3     | 用rs_driver_viewer连接在线雷达，split_frame_mode=by_fixed_pkts，num_pkts_split=N  | 显示点云 |
| P3     | 用rs_driver_viewer连接在线雷达，split_frame_mode=by_custome_pkts  | 显示点云 |
| P3     | 用rs_driver_viewer连接在线雷达，start_angle=90, end_angle=180  | 显示半圈的点云 |
| P3     | 用rs_driver_viewer连接在线雷达，min_distance=1, end_angle=10  | 显示被裁剪的点云 |

## 2.4 rs_driver解析PCAP文件
| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P3     | 用demo_pcap解析PCAP文件，雷达工作在广播模式下             | 正常打印点云 |
| P2     | 用rs_driver_viewer解析PCAP文件                            | rs_driver_viewer正常显示点云  |
| P3     | 用rs_driver_viewer解析PCAP文件, pcap_repeat=true(或false)      | 循环播放PCAP中的点云(或播放一遍退出)     |
| P3     | 用rs_driver_viewer解析PCAP文件, pcap_rate=0.5/1/2              | 点云播放速度变慢（或正常播放、加快）  |
| P3     | 用rs_driver_viewer解析PCAP文件, PCAP的包带VLAN层                | demo_pcap正常打印点云           |

## 2.5 rs_driver错误输出

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P1     | 用demo_online连接在线雷达，网络设置错误。       |  驱动打印错误提示无法收到MSOP/DIFOP包  |
| P1     | 用demo_online连接在线雷达，雷达类型设置错误。       |  驱动打印错误提示包的大小或格式错误 |
| P1     | 用demo_pcap连接在线雷达，指定不存在的PCAP文件。       |  驱动打印错误提示文件路径无误 |

## 2.6 其他功能

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P1     | 用demo_online连接在线雷达，ENABLE_TRASFORM=TRUE，设置转换参数。       |  点云坐标变换正确   |

## 3 性能测试

### 3.1 针对工控机

在当下主流性能的工控机上，做如下测试。

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P1     | 用demo_online连接在线的RSHELIOS/RS128/RSM1雷达，测试demo_online的CPU占用率       |  低于期望值（值待定） |
| P1     | 用demo_online连接在线的RSHELIOS/RS128/RSM1雷达，测试输出的点云大小       |  丢包率低于期望值（值待定） |
| P1     | 启动demo_online两个实例，连接两个在线的RSHELIOS/RS128/RSM1雷达，测试demo_online的CPU占用率       |  低于期望值（值待定） |
| P1     | 用demo_online两个实例，连接两个在线的RSHELIOS/RS128/RSM1雷达，测试输出的点云大小       |  丢包率低于期望值（值待定） |

### 3.2 针对ARM板

在当下主流性能的ARM板上，做如下测试。

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P1     | 用demo_online连接在线的RSM1雷达，测试demo_online的CPU占用率       |  低于期望值（值待定） |
| P1     | 用demo_online连接在线的RSM1雷达，测试输出的点云大小       |  丢包率低于期望值（值待定） |
| P1     | 启动demo_online两个实例，连接两个在线的RSM1雷达，测试demo_online的CPU占用率       |  低于期望值（值待定） |
| P1     | 用demo_online两个实例，连接两个在线的RSM1雷达，测试输出的点云大小       |  丢包率低于期望值（值待定） |



 

