# racecar 智能小车 ROS2 工作空间

基于 **ROS2（Foxy）+ Ubuntu 20.04** 的竞速小车整体工程，包含激光雷达循迹避障、SLAM 建图、导航、遥控、红绿灯识别等功能。整车源码头文件与算法节点均备注了中文说明。

> 程序在车载工控机上的运行方式：`bash racecar/car.sh`（见 `命令.txt`，主机名 `davinci-mini`）。

---

## 1. 硬件平台

| 部件 | 型号 | 说明 |
| --- | --- | --- |
| 激光雷达 | lslidar M10 系列 | 环境感知、尺度测距、锥桶检测（`/scan`） |
| IMU | hipnuc | 姿态与航向（`tf`/`Odometry`） |
| 相机 | USB 摄像头 | 红绿灯识别、颜色检测、循迹辅助 |
| 底盘 | 舵机 + 轮毂电机 | 由 `racecar_driver` 底层驱动 |
| 编码器 | — | 里程计反馈（`encoder`） |

---

## 2. 工作空间目录结构

```
racecar/
├── car.sh            # 一键启动（ros2 launch racecar Run_car.launch.py）
├── gmapping.sh       # SLAM 一键建图
├── nav.sh / nav_one.sh  # Nav2 导航运行
├── save.sh           # 保存地图
├── racecar_init.sh   # 初始化
└── src/              # 源码与功能包 (colcon 工作空间)
    ├── racecar            # 主包：launch/config/map/rviz/scripts
    ├── lidar_tracking     # C++ 激光雷达锥桶循迹避障节点（含 PID）
    ├── racecar_driver     # 底盘串口驱动节点
    ├── encoder            # 轮式编码器里程计节点
    ├── hipnuc_imu         # IMU 驱动 + EKF 滤波
    ├── lslidar_driver     # lslidar M10 系列激光驱动
    ├── lslidar_msgs       # 激光雷达自定义消息
    ├── nav2_waypoint_cycle  # Nav2 多航点循环导航节点
    ├── openslam_gmapping  # Gmapping SLAM（openslam 移植）
    ├── slam_gmapping      # slam_gmapping 包
    └── serial-foxy        # ROS2 串口通信库（Foxy）
```

---

## 3. 功能包详解

| 包 | 语言 | 作用 |
| --- | --- | --- |
| `racecar` | Python / Python | 主包。含 `launch/` 各启动脚本、`config/` 相机/EKF/导航参数、`map/` 地图、`scripts/` 循迹与识别节点、`rviz/` 配置 |
| `lidar_tracking` | C++ | **激光雷达锥桶循迹避障节点**：`/scan` → 左右锥桶检测 → 路径规划 → PID 打舵 → `/teleop_cmd_vel` |
| `racecar_driver` | C++（/Python） | 底层底盘驱动，把速度指令下发到舵机与电机 |
| `encoder` | Python | 编码器里程计节点（发布 `Odometry` / 速度） |
| `hipnuc_imu` | C++ | IMU 驱动 + EKF（`config/ekf_params.yaml`） |
| `lslidar_driver` | C++ | lslidar M10 系列雷达驱动（`lslidar_launch.py`） |
| `nav2_waypoint_cycle` | Python | 基于 Nav2 的**多航点循环**导航，`waypoint_cycle.py` |
| `openslam_gmapping` / `slam_gmapping` | C++ | Gmapping SLAM 建图 |

---

## 4. 核心避障 / 循迹算法

### 4.1 `lidar_tracking`（C++，激光锥桶循迹）

- **输入**：`/scan`（激光雷达扫描，频率 1440Hz）
- **核心逻辑**：从每一帧激光数据提取左右锥桶的直角坐标 → 按权重 `rate_p[3]` 做路径规划 → 使用 `control.hpp` 的 PID 控制器打舵 → 发布 `Twist`
- **多种状态机**：
  - 正常循迹：左右 `max_right_dis` / `max_left_dis`（默认 1.5m）内检测锥桶并跟踪
  - **直角转弯模式**：检测前方锥桶分布触发，自动减速转向（`right_angle_*` 参数）
  - **无锥桶巡航模式**：①扇形范围（默认 1.0m / 60°）内无锥桶时按固定角度寻路
- **关键参数**（`lidar_tracking_node`）：`speed`、`kp`/`ki`/`kd`、`rate1~rate3`、`no_cone_*`、`right_angle_*`、`max_right_dis`/`max_left_dis`

### 4.2 `racecar/scripts`（Python，循迹 / 识别节点）

| 脚本 | 作用 |
| --- | --- |
| `line_follow.py` / `line_follow2.py` | 循迹（`xunxian.launch.py` 启动 `line_follow`） |
| `straight_forward_control.py` | 直行 / 基础控制 |
| `zhuitong.py` / `go.py` / `detct.py` | 锥桶追踪、行进、检测 |
| `racecar_teleop.py` | 键盘遥控 |
| `camer.py` | 摄像头图像处理 |
| `hsv_show.py` | HSV 颜色调试 |
| `red_light_*` / `ai_*.py` | 红绿灯识别、AI 相关脚本 |

> 说明：另有按日期命名的独立集控脚本（`main.py`、`integrated_control.py` 等，含目标点地图循迹 + 红绿灯 + 判圈）位于工程同级 `11_24/` 目录，本 README 仅覆盖 `racecar` 工作空间内容。

---

## 5. 编译

```bash
cd racecar
colcon build
source install/setup.bash
```

---

## 6. 运行

**一键启动整车（含 driver / encoder / tf / 循迹）**
```bash
bash car.sh                    # 等价于 ros2 launch racecar Run_car.launch.py
```

**循迹单独运行**
```bash
ros2 launch racecar xunxian.launch.py     # 启动 racecar / line_follow
```

**激光雷达锥桶循迹**
```bash
ros2 launch lidar_tracking <launch>       # 或单独运行 lidar_tracking_node
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 1   # 打开雷达
```

**SLAM 建图 / 保存地图**
```bash
bash gmapping.sh
bash save.sh                 # 保存地图
```

**Nav2 导航（多航点循环）**
```bash
bash nav.sh                  # 或 nav_one.sh
```

**键盘遥控**
```bash
ros2 run racecar racecar_teleop
```

---

## 7. 常用话题

| 话题 | 类型 | 方向 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | lidar → 算法 |
| `/teleop_cmd_vel` | `geometry_msgs/Twist` | 算法 → 驱动 |
| `/odom` | `nav_msgs/Odometry` | encoder/IMU → 上层 |
| `/cmd_vel` | `geometry_msgs/Twist` | 控制指令 |

---

## 8. 备注

- 工作空间根目录的 `build/`、`install/`、`log/` 及 `__pycache__` 已通过 `.gitignore` 排除，仅保留源码。
- 各算法节点含大量中文注释，可直接作为调试与二次开发参考。
