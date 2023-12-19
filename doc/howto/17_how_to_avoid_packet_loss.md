# 17 **How to avoid Packet Loss**



## 17.1 Overview

This document illustrates when packet loss happens, and bring out a way to avoid that.



## 17.2 Is there packet loss ? 

Run demo app`demo_online`, and check if it prints normal count of points.
+ Mechenical LiDARs should have a count close with its theoretical count. If it jump up and down, packet loss may happen.
+ M1 LiDAR should always have a count equal with its theoretical count, else packet loss happens.

To observe multiple LiDARs case, open multiple terminals, and run multiple `demo_online`. Check if every LiDARs prints normally.



## 17.3 In what cases Packet Loss happens

Packet loss may happens in below cases.
+ on some platforms, such as Windows and embedded Linux
+ Multiple LiDARs
+ CPU resources is busy.



## 17.4 Solution

The solution is to increase the receiving buffer of MSOP Packet Socket.

in `CMakeLists.txt`，CMake macro `ENABLE_MODIFY_RECVBUF` enable this feature.

```cmake
option(ENABLE_MODIFY_RECVBUF       "Enable modify size of RCVBUF" OFF)
```

The code is as below.  Please test it in your cases, and change buffer size to a good value.

```c++
#ifdef ENABLE_MODIFY_RECVBUF
  {
    uint32_t opt_val = input_param_.socket_recv_buf, before_set_val,after_set_val = 0;
    if(opt_val < 1024)
    {
      opt_val = 106496;
    }
    socklen_t opt_len = sizeof(uint32_t);
    getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&before_set_val, &opt_len);
    RS_INFO << "before: recv buf opt_val:" <<before_set_val << std::endl;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&opt_val, opt_len);
    getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&after_set_val, &opt_len);
    RS_INFO << "aftert: recv buf opt_val:" <<after_set_val << std::endl;
  }
#endif
```

On some platforms, such as Ubuntu, the maximum value of the receiving buffer is restricted, and setsockopt() may fails. 

To enlarge the maximum value, run this command. It change the value to 851,968 bytes.


```shell
sudo bash -c "echo 851968 > /proc/sys/net/core/rmem_max"
```

