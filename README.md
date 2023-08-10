# yacyac
yacyac robot description

# Dependence Package
```bash
$ sudo apt install libzbar-dev
$ sudo apt install libzmq3-dev
$ sudo apt install ros-${ROS_DISTRO}-rosbridge-server
$ sudo apt install ros-${ROS_DISTRO}-image-transport
$ sudo apt install ros-${ROS_DISTRO}-compressed-image-transport
```

## install & build

---

### sdk install

```bash
cd ~/ros2_ws/src 
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK/build
cmake ..
make
sudo make install

sudo apt-get install python swig
sudo apt-get install python-pip
cd .. && pip install .
```

### driver install

```bash
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
colcon build --symlink-install
```

## Dynamixel

### workbench install

```bash

cd ~/ros2_ws/src 
git clone -b galactic_devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```


### cartographer

```bash
sudo apt install ros-galactic-cartographer-ros
```

### nav2

```bash
sudo apt-get install ros-galatic-nav2*
```

## robot setup

---

```bash
chmod 0777 ros2_ws/src/yacyac/yacyac_setup/*
cd ~/ros2_ws/src/yacyac/yacyac_setup
sudo sh initenv.sh
```