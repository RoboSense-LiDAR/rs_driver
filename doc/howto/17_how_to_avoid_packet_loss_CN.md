# **17 如何解决丢包问题**



## 17.1 概述

本文从分析雷达数据量入手，分析丢包原因，并给出解决丢包问题的方法。



## 17.2 雷达数据量分析

### 17.2.1 机械式雷达

按照转速`600`圈/分钟算，雷达转一圈为`0.1`秒。

以RS128雷达为例。它每轮扫描的间隔是`55.55`微秒，每一圈扫描的次数为：

```c++
 0.1 * 1000,000/55.55 = 1800.18（次）
```

取整为`1800`轮扫描。每个MSOP Packet为`1248`个字节，其中包括`3`个Block（每个Block对应一次扫描）。

所以每帧对应的数据量大小为：

```c++
1800 / 3 * 1248 = 748,800（字节）
```

这个数据量比较大，但由于解析式雷达发送MOSP Pakcet是持续均匀的，所以丢包少见。



### 17.2.2 MEMS雷达

M1雷达完成一帧也是`0.1`秒。它的一帧中包括`630`个MSOP Packet，每个Packet为`1210`字节。所以每帧的数据量大小为

```c++
630 * 1210 = 762,300（字节）
```

M1比RS128的每帧数据量略大一点，但是因为M1发送MSOP Packet不是理想的均匀发送，比如`630`个Packet可能分`10`次发送，每次发送`63`个包，这样的丢包和乱序的风险就明显增加了。



## 17.3 如何确定是否丢包

运行示例程序`demo_online`，观察每帧的点数是否正常。
+ 机械式雷达的每帧点数应该接近理论点数。如果有大幅波动，则可能是丢包了。
+ M1雷达的每帧点数应该就是理论点数，是固定的。如果少了，则可能是丢包了。

要观察多雷达的情况，可以打开多个终端，分别运行多个`demo_online`的实例，看看每个雷达有没有丢包。



## 17.4 丢包原因

以下情况下可能丢包。
+ 在某些平台上，如Windows和嵌入式Linux平台
+ 多雷达的情况下
+ 系统CPU资源紧张时



## 17.5 解决办法

解决丢包的办法是将接收MSOP Packet的Socket的接收缓存增大。

在`rs_driver`工程的`CMakeLists.txt`中，宏`ENABLE_DOUBLE_RCVBUF`可以使能这个特性。

```cmake
option(ENABLE_DOUBLE_RCVBUF       "Enable double size of RCVBUF" OFF)
```

代码如下，这里将接收缓存增大为原来的4倍。建议在实际场景下测试，再根据测试结果调整为合适的值。

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

在某些平台下，如Ubuntu，系统设置了接收缓存的最大值。如果`rs_driver`设置的值超过了最大值，调用setsockopt()就不会成功。

这时需要手工放宽这个限制，如下面的指令，将这个最大值改为`851,968`个字节。


```shell
sudo bash -c "echo 851968 > /proc/sys/net/core/rmem_max"
```

