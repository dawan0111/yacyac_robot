# YACYAC : Self-driving medicine distribution robot

병원 내 간호사의 업무 부담은 갈수록 커지고 있습니다. 한 간호사가 책임져야 하는 환자 수가 많아지면서 개별적인 환자 관리가 어려워지고, 작은 실수가 심각한 의료사고로 이어질 수 있는 위험성도 높아집니다. 이러한 문제를 해결하고자 "약약(YACYAC)" 프로젝트를 시작하게 되었습니다.

## **개발 환경**

- **Software**: Ubuntu 20.04 LTS, ROS2 Galactic
- **Programming Languages**: Python 3.10, C++ 17

### **Libraries & Tools**

- **Image Processing**: OpenCV 4.5.5
- **Mapping**: Cartographer
- **Path Planning & Control**: NAV2
- **Motor Control**: Dynamixel SDK
- **Robot Behavior Control**: Behavior Tree v.4

## **프로젝트 목표**

- **자율주행 로봇의 약 배달 자동화**:
    - 간호사의 업무 부담을 줄이기 위해 약 배달 및 배급 과정을 자동화하는 로봇을 개발합니다.
- **환자 중심의 정보 제공**:
    - 로봇에 탑재된 디스플레이를 통해 환자는 자신에게 투여되는 약에 대한 정보를 쉽게 파악할 수 있습니다.
- **통합 관제 시스템 구축**:
    - 중앙 관제 시스템을 통해 로봇의 작동 상태 및 약 배급 상황을 모니터링하고 원격으로 관리할 수 있습니다.

### #### 약약의 기능은 다음과 같습니다.

로봇은 LiDAR를 활용해 주변 환경을 인식하며, QR 코드를 스캔하여 환자 정보를 파악합니다. 인식된 정보를 기반으로 로봇은 경로 계획 및 약 배급을 수행하며, 정밀한 모터 제어로 한알씩 약을 배급합니다. 전체 동작은 Behavior Tree로 관리되어 복잡한 상황에서도 안정적으로 작동합니다.

## install & build

### Dependence Package

```jsx
sudo apt install libzbar-dev
sudo apt install libzmq3-dev
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
sudo apt install ros-${ROS_DISTRO}-image-transport
sudo apt install ros-${ROS_DISTRO}-compressed-image-transport
```

---

### YDLIDAR

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

### Dynamixel

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

## launch

```bash
ros2 launch yacyac_navigation2 navigation.launch.py
```

```bash
ros2 launch yacyac_core core.launch.py
```
