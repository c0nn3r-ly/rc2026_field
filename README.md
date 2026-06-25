# RC2026 Gazebo Harmonic 仿真场地功能包

![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=gazebo&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)

![image](./assets/overview.png)

本项目的场地模型基于重邮开源的 Blender 场地文件，场地以 `.dae` 模型加载到 Gazebo Harmonic / gz-sim 中，KFS 小方块模型由贴图生成并通过 ROS 2 GUI 控制。

> 注意：当前目标栈为 ROS 2 Humble + Gazebo Harmonic。官方更推荐 Jazzy + Harmonic；如果使用 Humble，请安装 `ros-humble-ros-gzharmonic`，避免同时安装会冲突的 Humble 默认 Fortress 版 `ros-humble-ros-gz*` 包。

## 快速开始

1. 安装依赖

先添加 Gazebo 官方 OSRF apt 源。Harmonic 的 Ubuntu 22.04 包不在默认 ROS 2 Humble apt 源中。

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt install ros-humble-ros-gzharmonic libgz-msgs10-dev libgz-transport13-dev python3-scipy python3-tk python3-yaml
pip install --user ttkbootstrap
```

2. 编译项目

推荐使用 symlink 方式，便于资源和 Python 代码改动后直接生效。

```bash
colcon build --symlink-install --packages-select rc2026_field
source install/setup.bash
```

如果要同时使用 MMRobot 仿真，请在工作空间内一起编译描述包：

```bash
colcon build --symlink-install --packages-select rc2026_field mmrobot_description
source install/setup.bash
```

3. 启动仿真

仅启动场地：

```bash
ros2 launch rc2026_field rc2026_field_sim.launch.py
```

启动场地仿真 + GUI 控制：

```bash
ros2 launch rc2026_field rc2026_field_sim_with_controller.launch.py
```

启动场地 + MMRobot + LiDAR + 手控驱动，推荐从工作空间根目录使用脚本入口：

```bash
cd /home/c0nn3r/桌面/rc_ws
./src/rc2026_field/scripts/start_mmrobot_sim.sh
```

默认 Gazebo 只跑 server。这样可以避开部分 Wayland/X11 + OpenGL 环境下 Gazebo GUI 和 RViz 同时启动导致的显示连接崩溃。

需要同时打开 Gazebo 图形界面时：

```bash
./src/rc2026_field/scripts/start_mmrobot_sim.sh gz_headless:=false
```

修改机器人初始位置时，通过启动参数设置，不建议在 Gazebo GUI 中手动拖车。当前简化位姿驱动会持续接管 `mmrobot` 的 Gazebo 位姿，手拖后会被写回驱动节点记录的位置。

```bash
./src/rc2026_field/scripts/start_mmrobot_sim.sh robot_x:=1.0 robot_y:=0.5 robot_yaw:=1.57
```

`robot_x` / `robot_y` 单位是 m，`robot_yaw` 单位是 rad；`robot_z` 默认是 `0.03`，需要从空中落体测试时可临时传更高值。

另开一个终端启动 RViz 观察：

```bash
./src/rc2026_field/scripts/start_mmrobot_view.sh
```

需要建图时，改用建图入口：

```bash
./src/rc2026_field/scripts/start_mmrobot_mapping.sh
```

手动控制机器人：

```bash
./src/rc2026_field/scripts/start_mmrobot_teleop.sh
```

默认手控模式是 Gazebo 物理版：ROS `/cmd_vel` 会桥接到 Gazebo 的 `/model/mmrobot/cmd_vel`，由 `MecanumDrive` 插件通过四个驱动轮推动车体。车体带简化碰撞体，会和场地 mesh 碰撞，并由 Gazebo `OdometryPublisher` 生成 `/odom`，再广播 `odom -> base_footprint` TF。

为了避免没有真实关节控制器时舵向和机械臂在物理里乱摆，当前导航测试版先固定这些非驱动关节，四个驱动轮仍参与 Gazebo 物理运动。需要把车出生在地形中间时，建议把 `robot_z` 设高一点，让车靠重力落到地形上：

```bash
./src/rc2026_field/scripts/start_mmrobot_sim.sh robot_x:=1.0 robot_y:=0.5 robot_z:=1.0
```

如果需要回到旧的强制位姿写入模式，可显式启动静态模型和 legacy pose driver：

```bash
./src/rc2026_field/scripts/start_mmrobot_sim.sh robot_static:=true pose_driver:=true
```

旧模式适合排查显示/话题问题，但会绕过物理碰撞。

可视化/建图相关话题：

- `/cmd_vel`：手动速度控制输入。
- `/odom`、`/tf`：Gazebo 物理里程计和 `odom -> base_footprint`。
- `/obstacle_scan`：`front_mid360` 坐标系下的 2D LaserScan。
- `/livox/lidar`：`front_mid360` 坐标系下的 PointCloud2。

Gazebo LiDAR 传感器直接挂在 MMRobot 自身的 `front_mid360` link 上，不再额外生成独立的 `mmrobot_lidar` 模型。

`front_mid360` 是仿真扫描和导航使用的水平 frame；雷达外壳显示在 `front_mid360_body` 上，保持车前横梁处向前俯仰 45 度的安装效果。这样点云能覆盖远处场地，不会因为扫描面俯仰 45 度而只打到车周围地面。

仿真 LiDAR 的最小距离设置为 `0.30m`，用于过滤雷达外壳和车体自身造成的贴车近点。

如果要开启第一版 2D 建图，请先安装：

```bash
sudo apt install ros-humble-slam-toolbox
```

原始 launch 入口仍保留，便于调试参数：

```bash
ros2 launch rc2026_field rc2026_mmrobot_sim.launch.py
ros2 launch rc2026_field rc2026_mmrobot_rviz.launch.py
ros2 launch rc2026_field rc2026_mmrobot_mapping.launch.py
```

## GUI 控制功能介绍

该控制器支持以下功能：

- 根据种子随机生成梅林的 KFS，也可在配置文件中开启手动指定摆放方式。
- 移除梅林中的 KFS，增加己方九宫格 KFS，并通过武器攻击对方 KFS。
- 九宫格支持模拟红蓝对抗，可通过 GUI 或 API 接口实现算法验证。

![image](./assets/panel.png)

### 通过点击 GUI 中对应的梅林格子移除 KFS

![image](./assets/MF.png)

### 九宫格 KFS 的放置

需要先选择队伍，然后点击对应格子执行放置本队 KFS 的操作。

武器可用于攻击另外一队的 KFS。通过点击另一队已有 KFS 的格子执行攻击操作，会消耗本队武器数量。

![image](./assets/grid.png)

### 全流程仿真设置说明

开启全流程仿真模式后，九宫格放置的 KFS 来源于已经在梅林区拾取的 KFS。独立模式下没有此限制，便于调试。

## 关键配置参数

配置文件路径：`config/kfs_config.yaml`

- 红蓝武器数量设置
- 梅林随机种子设置
- 是否开启手动设置梅林 KFS 分布
- 九宫格 KFS 坐标随机化参数

详细配置请参考配置文件内的注释。

## API 说明

主要外部接口保持不变：

- `/simulation/gui_event`：发送 GUI/控制指令。
- `/simulation/status`：查询当前场上 KFS 分布和状态。
- `/simulation/reset_kfs`：重置 KFS 随机布局和武器数量。

Gazebo Harmonic 内部姿态控制由 `gz_pose_bridge` 提供 `/simulation/set_entity_pose`，它会调用 Gazebo Transport 的 `/world/robocon2026_world_scene/set_pose/blocking` 服务。一般用户不需要直接调用该内部服务。

详情请看 `API.md`。

## scripts 目录说明

- `kill_gazebo.sh`：用于清理 gz-sim 和本项目 launch 进程。
- `random_kfs_in_MF.py`：用于随机生成梅林区的 KFS。
- `random_kfs_on_grid.py`：用于随机生成九宫格的 KFS。
- `random_all.py`：用于随机生成所有 KFS。

## 场地模型说明

本项目使用重庆邮电大学 HXC 战队提供的 Robocon 2026 比赛场地模型，并进行了部分删改：

- 修正 RC2026 场地梅林场地摆放错误。
- 将武馆的九宫格边框贴上黑色亚光乙烯胶带。
- 删除广告牌与文字。
- 修复 dae 文件材质错误。

## 参考文档

- [Gazebo Harmonic ROS 安装说明](https://gazebosim.org/docs/harmonic/ros_installation/)
- [ROS 2 启动 Gazebo](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/)
- [ROS 2 与 Gazebo 集成](https://gazebosim.org/docs/harmonic/ros2_integration/)
