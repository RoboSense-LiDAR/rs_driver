# 24 How to control laser emission

## 24.1 Introduction

This document illustrates how to control the LiDAR laser emission on/off through
RS-AC1. For the complete code, please refer to the demo app
`rs_driver/demo/demo_laser_control.cpp`.

The demo connects to RS-AC1 via USB, sends HID commands to the AC1 firmware, and
the AC1 firmware forwards them to the LiDAR (Su2310) via DoIP/UDS to control the
laser emission.

This solution does not reboot the LiDAR, so USB/UVC/IMU streams are unaffected
and the point cloud resumes quickly (about 1 second) after the laser is turned
back on.

## 24.2 Prerequisites

- **LiDAR firmware version** >= V02.00.09 (hex `020009`). RoutineControl 0xF003
  (StopAsixScan / StartAsixScan) is supported starting from V02.00.09.
- AC1 firmware has integrated the `LidarControl` module (mbiz).

## 24.3 API

`LidarDriver` adds the following public APIs:

```c++
// Turn off laser emission
bool setLaserOff();

// Turn on laser emission
bool setLaserOn();
```

Call `setLaserOn` / `setLaserOff` to control the laser emission on/off. This
interface is only adapted for RS-AC1; other models have not been verified yet.

## 24.4 Steps

### 24.4.1 Build

```bash
cd rs_driver/build
cmake .. && make demo_laser_control
```

### 24.4.2 Run

```bash
# Turn off laser emission
./demo_laser_control off

# Turn on laser emission
./demo_laser_control on
```

### 24.4.3 Demo Flow

```c++
// 1. Create driver and start (lidar type is RS_AC1)
LidarDriver<PointCloudMsg> driver;
driver.init(param);
driver.start();

// 2. Wait for DIFOP to arrive
std::this_thread::sleep_for(std::chrono::seconds(2));

// 3. Turn laser on or off based on command line argument
if (action == "off")
{
  driver.setLaserOff();
}
else
{
  driver.setLaserOn();
}

// 4. Stop driver
driver.stop();
```

## 24.5 Notes

- This solution does not trigger ECUReset. The LiDAR is not power-cycled and the
  USB connection is preserved.
- After turning off the laser, the point cloud stops but CMOS/IMU streams are
  unaffected. After turning it back on, the point cloud resumes in about 1
  second.
