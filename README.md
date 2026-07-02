# 步兵自瞄

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
apt install libtbb-dev libopencv-dev libeigen3-dev libboost-all-dev ros-${ROS_DISTRO}-image-pipeline ros-${ROS_DISTRO}-cv_bridge
```

## 架构

```
相机(Daheng) → DetectionPipeline → ArmorSolver → Tracker → Aimer → Shooter → Command发布
                    ↑                   ↑             ↑
              传统CV检测            IMU四元数      弹道解算
```

### 模块说明

| 模块 | 目录 | 职责 |
|------|------|------|
| **DetectionPipeline** | `include/detection/pipeline.hpp` | 传统视觉检测流水线：二值化→灯条提取→装甲板配对→PnP→数字分类 |
| **ArmorSolver** | `include/detection/armor_solver.hpp` | 坐标变换链：PnP→相机坐标系→云台坐标系→世界坐标系 + yaw重投影优化 |
| **Tracker** | `include/estimation/tracker.hpp` | 目标追踪状态机：lost→detecting→tracking→temp_lost，EKF整车状态估计 |
| **Target** | `include/estimation/target.hpp` | 11维EKF状态向量：[x,vx,y,vy,z,vz,yaw,omega,r,l,h]，卡方检验收敛检测 |
| **Aimer** | `include/decision/aimer.hpp` | 瞄准点选择（coming/leaving逻辑）+ 弹道飞行时间迭代求解 + 开火双阈值决策 |
| **EKF** | `include/tools/ekf.hpp` | 扩展卡尔曼滤波器（Joseph形式后验协方差 + NIS/NEES卡方检验） |
| **Trajectory** | `include/tools/trajectory.hpp` | 弹道解算（无空气阻力，g=9.7833，选短飞行时间解） |
| **MathTools** | `include/tools/math_tools.hpp` | 数学工具：欧拉角/四元数/旋转矩阵转换、坐标系变换、雅可比矩阵 |

### 坐标变换链

```
solvePnP(image_points) → xyz_in_camera
  → xyz_in_gimbal = R_camera2gimbal * xyz_in_camera + t_camera2gimbal
  → xyz_in_world = R_gimbal2world * xyz_in_gimbal
  → R_gimbal2world = R_gimbal2imubody^T * R_imubody2imuabs * R_gimbal2imubody
  → ypr_in_world = Eulers(R_armor2world, zyx)
  → ypd_in_world = Xyz2Ypd(xyz_in_world)
  → OptimizeYaw: ±70° 网格搜索最小重投影误差
```

### EKF 状态向量 (11维)

```
[x, vx, y, vy, z, vz, yaw, omega, r, l, h]
 0   1   2   3   4   5   6     7     8  9  10

x,y,z:   旋转中心世界坐标 (m)
vx,vy,vz: 旋转中心速度 (m/s)
yaw:     旋转角度 (rad)
omega:   角速度 (rad/s)
r:       装甲板半径 (m)
l:       r2 - r1 对向装甲板半径差 (m)
h:       z2 - z1 对向装甲板高度差 (m)
```

### 追踪器状态机

```
lost → (found) → detecting → (连续min_detect_count帧) → tracking
tracking → (lost) → temp_lost → (超max_temp_lost_count) → lost
temp_lost → (found) → tracking
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

### 实车运行  

```shell
source install/setup.bash
ros2 launch infantry_aimbot ia.launch.py
```

### 视频离线测试

用录制视频+IMU数据测试完整流水线，无需相机硬件和ROS运行环境：

```shell
source install/setup.bash
./install/infantry_aimbot/lib/infantry_aimbot/video_test <video.avi> <imu.txt> [bullet_speed] [delay_time]
```

参数说明：
- `video.avi` — 录制的视频文件
- `imu.txt` — IMU四元数文件，格式：每行 `timestamp w x y z`（空格分隔，时间戳单位：秒）
- `bullet_speed` — 子弹初速 m/s（默认 23.0）
- `delay_time` — 预测延迟时间 s（默认 0.005）

交互按键：
- `q` / `ESC` — 退出
- `空格` — 暂停/继续

显示内容：
- 绿色矩形框 — 检测到的装甲板灯条
- 黄色数字 — 装甲板ID和距离
- 蓝色十字 — 瞄准点重投影位置
- 左上角文字 — 追踪状态、耗时、指令角度、EKF状态

## 参数配置

参数通过 ROS2 parameter 机制加载（`config/settings.yaml`），运行时可通过 `ros2 param set` 动态调整。

### 检测器参数 (`detector`)
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `bin_threshold` | 200.0 | 二值化阈值 |
| `enemy_color` | 1 | 敌方颜色 (0=红, 1=蓝) |
| `angle_to_vertical_max` | 35.0 | 灯条最大倾斜角 (deg) |
| `width_height_min_ratio` | 1.1 | 装甲板宽高比下限 |
| `width_height_max_ratio` | 6.5 | 装甲板宽高比上限 |

### 解算器参数 (`solver`)
| 参数 | 说明 |
|------|------|
| `R_gimbal2imubody` | 云台→IMU本体旋转矩阵 (3×3行优先) |
| `R_camera2gimbal` | 相机→云台旋转矩阵 |
| `t_camera2gimbal` | 相机→云台平移向量 (m) |

### 追踪器参数 (`tracker`)
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `min_detect_count` | 5 | 最小连续检测帧数 |
| `max_temp_lost_count` | 15 | 最大临时丢失帧数 |
| `outpost_max_temp_lost_count` | 75 | 前哨站最大临时丢失帧数 |

### 瞄准器参数 (`aimer`)
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `yaw_offset` | 2.0 | yaw偏置 (deg) |
| `pitch_offset` | 6.5 | pitch偏置 (deg) |
| `comming_angle` | 55.0 | 来袭角阈值 (deg) |
| `leaving_angle` | 20.0 | 离去角阈值 (deg) |
| `decision_speed` | 7.0 | 高速/低速决策阈值 (rad/s) |

### 射击参数 (`shooter`)
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `first_tolerance` | 3.0 | 近距离容差 (deg) |
| `second_tolerance` | 2.0 | 远距离容差 (deg) |
| `judge_distance` | 2.0 | 远近距离阈值 (m) |
| `auto_fire` | false | 是否自动开火 |
| `bullet_speed` | 23.0 | 子弹初速 (m/s) |

## 数据结构

### Command
```cpp
struct Command {
  bool control{false};  // 是否有有效控制指令
  bool shoot{false};    // 是否开火
  double yaw{0.0};      // yaw 角度 (rad)
  double pitch{0.0};    // pitch 角度 (rad)
};
```

### Armor（扩展字段）
```cpp
// 世界坐标系成员（由 ArmorSolver 填充）
Eigen::Vector3d xyz_in_gimbal;   // 云台坐标 (m)
Eigen::Vector3d xyz_in_world;    // 世界坐标 (m)
Eigen::Vector3d ypr_in_world;    // 世界系欧拉角 (rad)
Eigen::Vector3d ypd_in_world;    // 球坐标 (yaw,pitch,distance)
ArmorName name;                   // 装甲板名称
ArmorKind kind;                   // 装甲板类型
ArmorPriority priority;           // 优先级
```
