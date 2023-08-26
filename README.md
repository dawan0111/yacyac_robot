# YacYac Robot

yacyac robot description

# Clone

This repository manages dependencies through submodules.

```
git clone --recursive https://github.com/dawan0111/yacyac.git
```

# Dependence package install

```bash
sudo apt install libzbar-dev libzmq3-dev ros-${ROS_DISTRO}-rosbridge-server ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-compressed-image-transport ros-${ROS_DISTRO}-cartographer-ros ros-${ROS_DISTRO}-nav2*
```

# LiDAR SDK install

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

# GCP(Google Cloud Platform) TTS dependence install

```bash
pip install --upgrade google-cloud-texttospeech
pip install playsound
```

# Setup USB rules

```bash
sudo sh initenv.sh
```
