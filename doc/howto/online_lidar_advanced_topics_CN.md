# 在线雷达 - 高级主题

## 1 简介

RoboSense雷达可以工作在单播/组播/广播模式下，也可以工作在VLAN环境下，也可以加入用户自己的层。

本文说明了在每种场景下如何配置rs_driver的参数。

阅读本文之前，请先阅读 [连接在线雷达](./how_to_decode_online_lidar_CN.md).

## 2 单播、组播、广播

### 2.1 广播模式

广播模式的配置最简单。

下面的图中，雷达发送MSOP/DIFOP包到主机，rs_driver运行在主机上。为了简化，图中没有画DIFOP端口。
+ 雷达发送到 `255.255.255.255` : `6699`, rs_driver绑定到端口`6699`.

![](./img/12_broadcast.png)

如下代码配置RSDriverParam。

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

### 2.2 单播模式

为减少网络流量，推荐使用单播模式。

如下的例子中，

+ 雷达发送到`192.168.1.102` : `6699`, rs_driver绑定到端口`6699`。

![](./img/12_unicast.png)

如下代码配置RSDriverParam。它与广播模式的配置完全相同。

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```


### 2.3 组播模式

雷达也可以工作在组播模式。

如下的例子中，

+ 雷达发送到`224.1.1.1`:`6699` 
+ rs_driver绑定到端口`6699`。 rs_driver让本地网卡加入组播组`224.1.1.1`. 这个网卡的地址是`192.168.1.102`。

![](./img/12_multicast.png)

如下代码配置RSDriverParam。

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.group_address = "224.1.1.1";    ///< Set the multicast group address.
param.input_param.host_address = "192.168.1.102"; ///< Set the host address.
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type. Make sure this type is correct 
```

## 3 多雷达的情况

### 3.1 雷达目的端口不同

如果要接入两个雷达，推荐给它们配置不同的目的端口。

如下的例子中，

+ 第一个雷达发送到`192.168.1.102`:`6699`，rs_driver的第一个实例绑定到端口`6699`。
+ 第二个雷达发送到`192.168.1.102`:`5599`，rs_driver的第二个实例绑定到端口`5599`。

![](./img/12_multi_lidars_port.png)



```c++
RSDriverParam param1;                              ///< Create a parameter object for Lidar 192.168.1.200
param1.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param1.input_param.msop_port = 6699;               ///< Set the lidar msop port number
param1.input_param.difop_port = 7788;              ///< Set the lidar difop port number
param1.lidar_type = LidarType::RS32;               ///< Set the lidar type.

RSDriverParam param2;                              ///< Create a parameter object for Lidar 192.168.1.201
param2.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param2.input_param.msop_port = 5599;               ///< Set the lidar msop port number
param2.input_param.difop_port = 6688;              ///< Set the lidar difop port number
param2.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

### 3.2 雷达的目的IP不同

虽然不推荐，也可以给接入的两个雷达配置不同的目的IP。
+ 主机有两个网卡，地址分别是`192.168.1.102` 和`192.168.1.103`。
+ 第一个雷达发送到`192.168.1.102`:`6699`，第一个rs_driver实例绑定到`192.168.1.102:6699`。
+ 第二个雷达发送到`192.168.1.103`:`6699`，第二个rs_driver实例绑定到`192.168.1.103:6699`。

![](./img/12_multi_lidars_ip.png)

如下代码分别配置两个rs_driver实例的RSDriverParam。

```c++
RSDriverParam param1;                              ///< Create a parameter object for Lidar 192.168.1.200
param1.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param1.input_param.host_address = "192.168.1.102"; ///< Set the host address.
param1.input_param.msop_port = 6699;               ///< Set the lidar msop port number
param1.input_param.difop_port = 7788;              ///< Set the lidar difop port number
param1.lidar_type = LidarType::RS32;               ///< Set the lidar type.

RSDriverParam param2;                              ///< Create a parameter object for Lidar 192.168.1.201
param2.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param2.input_param.host_address = "192.168.1.103"; ///< Set the host address.
param2.input_param.msop_port = 6699;               ///< Set the lidar msop port number
param2.input_param.difop_port = 7788;              ///< Set the lidar difop port number
param2.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

## 4 VLAN

有些场景下，雷达可以工作在VLAN环境下。这时MSOP/DIFOP包带VLAN层，如下图。

![](./img/12_vlan_layer.png)

rs_driver工作在应用层，接触不到VLAN层。这时需要用户创建一个虚拟网卡来剥除VLAN层。

如下是一个例子。
+ 给雷达分配的VLAN ID是`80`。雷达发送到`192.168.1.102` : `6699`。 发送的包带VLAN层。
+ 主机上装的物理网卡eno1也在VLAN ID `80`上，它接收雷达发出的带VLAN层的包。

![](./img/12_vlan.png)

要剥除VLAN层，需要用户手工创建一个虚拟网卡。如下的命令，在物理网卡eno1上创建虚拟网卡`eno1.80`，并给它指定IP地址`192.168.1.102` 。

```
sudo apt-get install vlan -y
sudo modprobe 8021q

sudo vconfig add eno1 80
sudo ifconfig eno1.80 192.168.1.102 up
```

现在rs_driver就可以从eno1.80网卡上接收MSOP/DIFOP包了，这些包不带VLAN层。

如下代码配置RSDriverParam。

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```

## 5 User Layer, Tail Layer 

某些场景下，用户可能在MSOP/DIFOP数据前后加入自己的层。
+ USER_LAYER 在MSOP/DIFOP数据之前，TAIL_LAYER在MSOP/DIFOP数据之后。

![](./img/12_user_layer.png)

这些层是UDP数据的一部分，所以rs_driver可以自己剥除他们。只需要告诉rs_driver每个层的字节数就可以。

如下的例子中，指定USER_LAYER为8字节，TAIL_LAYER为4字节。

```c++
RSDriverParam param;                              ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;       ///< get packet from online lidar
param.input_param.msop_port = 6699;               ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;              ///< Set the lidar difop port number, the default is 7788
param.input_param.user_layer_bytes = 8;           ///< user layer bytes. there is no user layer if it is 0
param.input_param.tail_layer_bytes = 4;           ///< tail layer bytes. there is no user layer if it is 0
param.lidar_type = LidarType::RS32;               ///< Set the lidar type.
```















