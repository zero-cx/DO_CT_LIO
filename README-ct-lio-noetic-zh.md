# CT-LIO Noetic / Gazebo 使用说明

这份工作区位于：

```bash
/home/ubuntu20/CT_LIO_ws
```

当前配置目标是接入 `/home/ubuntu20/gazebo` 的 Scout + VLP16 + 原始 IMU 仿真数据，并支持在 Docker 中运行 CT-LIO。

## 当前话题

CT-LIO 订阅：

```text
/velodyne_points
/imu/data_raw
```

Gazebo 真值可用于评估：

```text
/ground_truth/odom
/ground_truth/path
```

CT-LIO 输出：

```text
/odom
/odometry_path
/scan
/tf  map -> base_link
```

实时轨迹 CSV 默认保存到：

```text
/home/ubuntu20/CT_LIO_ws/downloads/trajectory.csv
```

## 关键配置位置

算法参数：

```text
/home/ubuntu20/CT_LIO_ws/src/ct_lio/config/mapping.yaml
```

当前传感器外参：

```yaml
mapping:
  extrinsic_est_en: false
  extrinsic_T: [-0.17, 0.0, 0.2927]
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

这里的 `extrinsic_T` 表示 LiDAR 在 IMU 坐标系下的位置，和当前 Gazebo Scout 传感器安装关系对应。

Gazebo raw IMU 初始化阈值：

```yaml
imu_initialization:
  init_time_seconds: 1.0
  use_speed_for_static_checking: false
  max_static_gyro_var: 2.0
  max_static_acce_var: 30.0
  gravity_norm: 9.81
```

这组阈值是为了适配 Gazebo 原始 IMU。换真实 IMU 后，建议先观察静止噪声，再逐步收紧。

## 编译 Docker 镜像

宿主机执行：

```bash
cd /home/ubuntu20/CT_LIO_ws
bash docker/build_ct_lio_noetic.sh
```

进入容器：

```bash
cd /home/ubuntu20/CT_LIO_ws
bash docker/run_ct_lio_noetic.sh
```

容器内编译：

```bash
cd /home/ubuntu20/CT_LIO_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 实时 Gazebo 仿真

终端 1，宿主机启动 Gazebo：

```bash
cd /home/ubuntu20/gazebo
source devel/setup.bash
roslaunch gazebo_bringup scout_ct_lio_sim.launch
```

如果要使用退化世界：

```bash
roslaunch gazebo_bringup scout_ct_lio_sim.launch world:=/home/ubuntu20/gazebo/src/scout_gazebo/worlds/medium_degenerate.world
```

终端 2，进入 CT-LIO Docker 并启动算法：

```bash
cd /home/ubuntu20/CT_LIO_ws
bash docker/run_ct_lio_noetic.sh

source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ct_lio run_gazebo_mapping.launch rviz:=true
```

需要键盘控制时，在宿主机另开终端：

```bash
cd /home/ubuntu20/gazebo
source devel/setup.bash
roslaunch gazebo_bringup keyboard_teleop.launch
```

## 离线 bag 跑法

终端 1，容器内启动 CT-LIO：

```bash
cd /home/ubuntu20/CT_LIO_ws
bash docker/run_ct_lio_noetic.sh

source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ct_lio run_gazebo_mapping.launch rviz:=true
```

终端 2，宿主机播放 bag：

```bash
source /opt/ros/noetic/setup.bash
rosparam set /use_sim_time true
rosbag play --clock -r 1.0 /home/ubuntu20/gazebo/downloads/medium_degenerate_rawimu.bag
```

新 bag：

```bash
rosbag play --clock -r 1.0 /home/ubuntu20/gazebo/downloads/medium_degenerate_rawimu_new.bag
```

## 本次验证结果

测试在 Docker 中完成，使用 `/ground_truth/odom` 作为真值，并使用 0.05 秒最大时间匹配窗口。

| 数据包 | raw RMSE / P95 | SE(2) RMSE / P95 |
| --- | --- | --- |
| `medium_degenerate_rawimu.bag` | 0.0906 m / 0.1352 m | 0.0820 m / 0.1165 m |
| `medium_degenerate_rawimu_new.bag` | 0.0790 m / 0.1252 m | 0.0727 m / 0.1108 m |

说明：

- raw 指轨迹起点平移对齐后的直接 XY 误差。
- SE(2) 指允许整体平移和 yaw 对齐后的轨迹形状误差。
- CT-LIO 当前没有接入回环；这组结果主要反映前端连续时间 LiDAR-IMU 里程计在该退化仿真环境中的表现。

## 注意事项

- 如果 CT-LIO 一直不输出 `/odom`，优先检查 `/imu/data_raw` 和 `/velodyne_points` 是否正在发布。
- 如果日志出现“加计测量噪声太大”，说明 raw IMU 静止初始化阈值太严，可以在 `mapping.yaml` 的 `imu_initialization` 中放宽。
- 当前 RViz 配置来自 CT-LIO 原工程，主要看 `/scan`、`/odom`、`/odometry_path`。
