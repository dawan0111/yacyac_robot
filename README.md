# Dependence Package

```jsx
sudo apt install libzbar-dev
sudo apt install libzmq3-dev
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
sudo apt install ros-${ROS_DISTRO}-image-transport
sudo apt install ros-${ROS_DISTRO}-compressed-image-transport
```

## install & build

---

## YDLIDAR

### sdk install

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

### driver install

```bash
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
colcon build --symlink-install
```

## Dynamixel

### workbench install

```jsx

cd ~/ros2_ws/src 
git clone -b galactic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

### cartographer

```bash
sudo apt install ros-galactic-cartographer-ros
sudo apt install ros-galactic-cartographer
```

### nav2

```bash
sudo apt-get install ros-galactic-nav2*
```

## robot setup

---

```bash
chmod 0777 ~/ros2_ws/src/yacyac/*
cd ~/ros2_ws/src/yacyac
sudo sh initenv.sh
```

## tts

---

```bash
cd ~/ros2_ws/src/yacyac/yacyac_io/yacyac_io
python3 -m venv env
source env/bin/activate
pip install --upgrade google-cloud-texttospeech
pip install playsound
```

### Qt serial port

```jsx
sudo apt-get install libqt5serialport5-dev
```

## bashrc

---

```jsx
alias eb='nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias gs='git status'
alias gp='git pull'

alias gala="source /opt/ros/galactic/setup.bash; echo \"ROS2 galactic\""
alias killgazebo="killall gzserver gzclient"
alias ros_domain="export ROS_DOMAIN_ID=13"
alias ros2study="gala; source ~/ros2_ws/install/local_setup.bash; echo \"ros2 ws is activated.!!\""

alias cb="cd ~/ros2_ws && colcon build --symlink-install && source install/local_setup.bash"
export ROS_DISTRO=galactic

export YDLIDAR_SDK=/path/to/YDLidar-SDK
export RCUTILS_COLORIZED_OUTPUT=1
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/local_setup.bash
source /opt/ros/galactic/setup.bash

export GOOGLE_APPLICATION_CREDENTIALS=~/yacyac-gcp.json
#export ROS_LOCALHOST_ONLY=1
```