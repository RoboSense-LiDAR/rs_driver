#  24 **如何控制激光发射**

## 24.1 概述

本文描述如何通过 RS-AC1 控制 LiDAR 激光发射的开关。完整的代码可以参考示例程序
`rs_driver/demo/demo_laser_control.cpp`。

该示例程序通过 USB 连接 RS-AC1，向 AC1 固件发送 HID 命令，由 AC1 固件通过
DoIP/UDS 转发给 LiDAR（Su2310），实现激光发射的开关控制。

该方案不会重启 LiDAR，因此 USB/UVC/IMU 流不会中断，后续开启激光后点云能快速
恢复（约 1 秒）。

## 24.2 前置条件

- **LiDAR 固件版本** >= V02.00.09（版本号十六进制 `020009`）。V02.00.09 起支持
  RoutineControl 0xF003（StopAsixScan / StartAsixScan）。
- AC1 固件已集成 `LidarControl` 模块（mbiz）。

## 24.3 API

`LidarDriver` 新增以下公共 API：

```c++
// 关闭激光发射
bool setLaserOff();

// 开启激光发射
bool setLaserOn();
```

调用 `setLaserOn` / `setLaserOff` 即可控制激光发射的开关。该接口仅适配了
RS-AC1，其他型号尚未适配。

## 24.4 步骤

### 24.4.1 编译

```bash
cd rs_driver/build
cmake .. && make demo_laser_control
```

### 24.4.2 运行

```bash
# 关闭激光发射
./demo_laser_control off

# 开启激光发射
./demo_laser_control on
```

### 24.4.3 示例代码流程

```c++
// 1. 创建 driver 并启动（雷达类型为 RS_AC1）
LidarDriver<PointCloudMsg> driver;
driver.init(param);
driver.start();

// 2. 等待 DIFOP 到达
std::this_thread::sleep_for(std::chrono::seconds(2));

// 3. 根据命令行参数开启或关闭激光
if (action == "off")
{
  driver.setLaserOff();
}
else
{
  driver.setLaserOn();
}

// 4. 停止 driver
driver.stop();
```

## 24.5 注意事项

- 该方案不触发 ECUReset，LiDAR 不断电，USB 连接保持。
- 关激光后点云停止，CMOS/IMU 流不受影响；开激光后点云约 1 秒恢复。
