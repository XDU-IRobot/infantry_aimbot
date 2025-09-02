# 步兵自瞄（重构进行中）

## 依赖

- ROS2 `>=humble`

```cmake
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
```

```xml
<depend>cv_bridge</depend>
```

```shell
apt update
apt install libtbb-dev libopencv-dev libeigen3-dev libboost-all-dev ros-${ROS_DISTRO}-image-pipeline ros-${ROS_DISTRO}-cv-bridge
```

## 标定相机

按实际情况修改calibrate_camera.sh里的参数

```shell
scripts/calibrate_camera.sh
```

## BUILD

```shell
colcon build --merge-install --symlink-install
```

## RUN

```shell
source install/setup.bash
ros2 launch infantry_aimbot ia.launch.py
```