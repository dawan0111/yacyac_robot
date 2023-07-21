# yacyac
yacyac robot description

# Dependence Package
```bash
$ sudo apt install libzbar-dev
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