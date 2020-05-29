## rs_driver

### 1. Introduction
  **rs_driver** is a driver code for RoboSense LiDAR,  include **RS-LiDAR-16**、**RS-LiDAR-32**、**RS-Bpearl** and **RS-Ruby** now. It can be used to extract packets of lidar to pointcloud, and is convenient for users to do secondary development.



### 2. Dependency 

- Boost
- pthread
- pcap

Boost and pthread are libraries of system, that can be linked directly. pcap need to install as follow:

```
sudo apt-get install -y  libpcap-dev
```

### 3. Install

```sh
    cd rs_driver
    mkdir build && cd build
    cmake .. && make -j4
    sudo make install
```