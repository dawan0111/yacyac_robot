# YacYac Robot
[![image.png](https://i.postimg.cc/Rh8DLRtS/image.png)](https://postimg.cc/GTFKcvzV)
YACYAC is an innovative solution aimed at reducing the workload of nurses and ensuring precise medicine dispensing to enhance patient safety. This project was developed for the 21st Embedded Software Contest.

## Demo
https://www.youtube.com/watch?v=_hdeFKSbxH8

## HW architecture
[![2023-ESWContest-1114-1.png](https://i.postimg.cc/Fz2LZdH5/2023-ESWContest-1114-1.png)](https://postimg.cc/kRvGM5Sf)

## SW architecture
[![2023-ESWContest-1114-1.png](https://i.postimg.cc/FRXFDyjx/2023-ESWContest-1114-1.png)](https://postimg.cc/kVwd5tYV)
[![image.png](https://i.postimg.cc/yYrTWj4x/image.png)](https://postimg.cc/kBKSpx5P)


## Clone

This repository manages dependencies through submodules.

```
git clone --recursive https://github.com/dawan0111/yacyac.git
```

## Dependence package install

```bash
sudo apt install libzbar-dev libzmq3-dev ros-${ROS_DISTRO}-rosbridge-server ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-compressed-image-transport ros-${ROS_DISTRO}-cartographer-ros ros-${ROS_DISTRO}-nav2*
```

## LiDAR SDK install

```bash
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK/cmake
cmake ..
make
sudo make install

sudo apt-get install python swig
sudo apt-get install python-pip
cd .. && pip install .
```

## GCP(Google Cloud Platform) TTS dependence install

```bash
pip install --upgrade google-cloud-texttospeech
pip install playsound
```

## Setup USB rules

```bash
sudo sh initenv.sh
```
