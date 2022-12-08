# 17 **How to avoid Packet Loss**



## 17.1 Overview

This document illustrate why and what case packet loss happens, and bring out a way to avoid that.



## 17.2 Data flow of LiDARs

### 17.2.1 Mechanical LiDARs

If the rotate speed is `600` rpm, it cost 01. second per round.

Take RS128 as an example. Its scan interval is `55.55`us, so it scans in a round.

```c++
 0.1 * 1000,000/55.55 = 1800.18
```

Approximately `1800`. A MSOP Packet is `1248` bytes, which consists of `3` blocks ( 1 block for 1 scan).

So data flow per frame is:

```c++
1800 / 3 * 1248 = 748,800 (bytes)
```

This is big. However Mechanical LiDARs send MSOP packet smoothly, so packet loss is rare.



### 17.2.2 MEMS LiDAR

M1 LiDAR finish a frame in `0.1` second too. A frame is from `630` MSOP packets. The packet length is `1210` bytes. So the data flow per frame is:

```c++
630 * 1210 = 762,300 (bytes)
```

M1 data flow is a little more than RS128.  However M1 may send many packet in a time. For example, it may send 630 packets in 10 times, 63 packet each send. Thus the possibility of loss and out of order is high.



## 17.3 Is there packet loss ? 

Run demo app`demo_online`, and check if it prints normal count of points.
+ Mechenical LiDARs should have a count close with its theoretical count. If it jump up and down, packet loss may happen.
+ M1 LiDAR should always have a count equal with its theoretical count, else packet loss happens.

To observe multiple LiDARs case, open multiple terminals, and run multiple `demo_online`. Check if every LiDARs prints normally.



## 17.4 In what cases Packet Loss happens

Packet loss may happens in below cases.
+ on some platforms, such as Windows and embedded Linux
+ Multiple LiDARs
+ CPU resources is busy.



## 17.5 Solution

The solution is to increase the receiving buffer of MSOP Packet Socket.

in `CMakeLists.txt`，CMake macro `ENABLE_DOUBLE_RCVBUF` enable this feature.

```cmake
option(ENABLE_DOUBLE_RCVBUF       "Enable double size of RCVBUF" OFF)
```

The code is as below.  Here it increases the buffer to 4 times. Please test it in your cases, and change it to a good value.

```c++
#ifdef ENABLE_DOUBLE_RCVBUF
  {
    uint32_t opt_val;
    socklen_t opt_len = sizeof(uint32_t);
    getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&opt_val, &opt_len);
    opt_val *= 4;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&opt_val, opt_len);
  }
#endif
```

On some platforms, such as Ubuntu, the maximum value of the receiving buffer is restricted, and setsockopt() may fails. 

To enlarge the maximum value, run this command. It change the value to 851,968 bytes.


```shell
sudo bash -c "echo 851968 > /proc/sys/net/core/rmem_max"
```

