# gps_slam_converter

## Document
- [gps_slam_converter - For converting coordinates between GPS <-> SLAM](#gpsslamconverter)
    - [Document](#document)
    - [Environment](#1-environment)
    - [SetUp Installation](#2-setup-installation)
        - [Prerequisites](#2-1-prerequisites)
    - [Clone & Build Project](#3-clone--build-project)
      - [Clone Project](#3-1-clone-project)
      - [Build Project](#3-2-build-project)

## 1. Environment
* <img src="https://img.shields.io/badge/ros2 foxy-22314E?style=for-the-badge&logo=ros&logoColor=white">
* <img src="https://img.shields.io/badge/c++ 11-00599C?style=for-the-badge&logo=cplusplus&logoColor=white">
* <img src="https://img.shields.io/badge/cmake 3.2.1-064F8C?style=for-the-badge&logo=cmake&logoColor=white">
* <img src="https://img.shields.io/badge/python 3.8.10-3776AB?style=for-the-badge&logo=python&logoColor=white">
* <img src="https://img.shields.io/badge/ubuntu 20.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">

## 2. SetUp Installation

### 2-1. Prerequisites

Before installing, please ensure the following software is installed and configured on your system:

- [ubuntu](https://ubuntu.com/) version required 20.04 - **INSTALL [ubuntu 20.04](https://ubuntu.com/)**

- [ROS2](https://docs.ros.org/en/foxy/Installation.html) version required foxy - **INSTALL [ROS2(foxy)](https://docs.ros.org/en/foxy/Installation.html)** 


## 3. Clone & Build Project

### 3-1. Clone Project
```bash
cd ${your workspace}/src
git clone https://github.com/reidlo5135/gps_slam_converter.git
```

## 3-2. Build Project
```bash
source /opt/ros/foxy/setup.bash
cd ${your workspace}/src/gps_slam_converter
colcon build --symlink-install
cd ../..
colcon build --packages-select gps_slam_converter
source install/setup.bash
```