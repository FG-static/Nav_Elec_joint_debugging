# ace_ass

这是一个 ROS 2 机器人导航与定位工作区，当前重点是面向实车的底盘状态估计、Livox 点云 GICP 匹配、Nav2 联调和串口通信。项目主要围绕一台全向轮小车展开：从 MCU/IMU/LiDAR 数据接入开始，输出 `/odom`、`odom -> base_footprint` TF、路径轨迹和可选的 GICP 点云叠图，用于 RViz 可视化与 Nav2 控制。

## 项目方向

项目当前的核心方向不是完整 SLAM 建图，而是“稳定的实时里程计 + 点云相对匹配辅助校正”：

- 使用 IMU、轮速和零倾斜约束构建 ESKF/IESKF 状态估计。
- 使用 Livox 点云做帧间或局部子地图 GICP 配准，得到相对运动约束。
- 将 GICP 的速度/yaw 残差融合进 ESKF，抑制长距离绕圈后的累计误差。
- 发布 `/gicp/map_cloud` 作为诊断叠图，用于观察点云配准质量。
- 接入 Nav2 的 planner/controller/bt_navigator，用 `/odom` 和 TF 支撑导航调试。

## 包结构

- `my_nav2_robot`
  - `data_handle_node`：主定位节点，处理 `/tracker/gimbal`、`/livox/lidar/pointcloud`，发布 odom、path、GICP 诊断。
  - `imu_adapter_node`：将 `/livox/imu` 转成 `/tracker/gimbal`，用于无 MCU/轮速时的 rosbag 或 Livox 直接测试。
  - `launch/`：实车、Nav2、rosbag 调试启动文件。
  - `config/config.yaml`：ESKF、GICP、deskew、local submap 和叠图参数。
  - `config/nav2_params.yaml`：Nav2 参数。
  - `urdf/robot.urdf.xacro`：机器人模型。
- `serial_driver`
  - 串口驱动节点，和 MCU 通信。
  - 发布 `/tracker/gimbal`，订阅 `/tracker/target`。
- `rm_interfaces`
  - 自定义消息，主要包括 `Gimbal.msg` 和 `Target.msg`。

## 已实现功能

- 串口接入 MCU 数据，解析 IMU、轮速和测试消息。
- Livox IMU 适配，支持在没有 MCU 轮速时用 `/livox/imu` 生成 `/tracker/gimbal`。
- ESKF/IESKF 状态估计：
  - IMU predict。
  - 轮速观测。
  - pitch/roll 零倾斜约束。
  - 直线行驶 yaw rate 软约束。
  - 加速度计和陀螺仪零偏标定。
- 点云 GICP：
  - 点云体素降采样。
  - 每点时间戳旋转 deskew。
  - 可选 GICP 校正后的平移 deskew。
  - 帧间 GICP 和局部滑窗子地图 GICP。
  - GICP 速度、yaw、fitness、innovation 多级门控。
  - 只在最终速度和 yaw 校验通过后写入 local submap，避免坏帧污染历史。
- RViz/诊断输出：
  - `/odom`
  - `/path`
  - `/odom_raw`
  - `/path_raw`
  - `/gicp/vel_body`
  - `/gicp/innovation_body`
  - `/gicp/yaw_debug`
  - `/gicp/status`
  - `/gicp/aligned_cloud`
  - `/gicp/map_cloud`
- TF：
  - `map -> odom` 静态 TF。
  - `odom -> base_footprint` 动态 TF。
  - 可选 `odom -> gicp_map` 叠图 TF。

## 环境要求

当前代码按 ROS 2 + colcon 工作区组织，主要依赖：

- ROS 2 Humble 或 Jazzy
- Nav2
- PCL
- Eigen3
- Sophus
- `small_gicp`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`

安装依赖后，在工作区根目录构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

如果使用 Humble，把第一行改成：

```bash
source /opt/ros/humble/setup.bash
```

## 快速启动

### 1. 实车完整启动

实车模式会启动串口驱动、定位节点、IMU 适配节点、Nav2、robot state publisher 和 RViz。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_nav2_robot full.launch.py
```

串口默认设备在 `src/serial_driver/config/serial_driver.yaml`：

```yaml
device_name: /tmp/ttyACM0
baud_rate: 115200
```

如果实车串口不是 `/tmp/ttyACM0`，先修改这个配置。

### 2. 只启动导航与定位，不启动串口

适合 rosbag、Livox 直接输入或外部节点已经提供 `/tracker/gimbal` 的情况。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_nav2_robot full_rosbag.launch.py
```

### 3. 单独启动定位节点

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run my_nav2_robot data_handle_node --ros-args --params-file src/my_nav2_robot/config/config.yaml
```

### 4. 单独启动 Livox IMU 适配

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run my_nav2_robot imu_adapter_node
```

该节点订阅 `/livox/imu`，发布 `/tracker/gimbal`。注意它会把轮速填 0，因此更适合 rosbag/GICP/IMU 调试，不等价于完整实车轮速闭环。

## RViz 查看建议

常规定位查看：

- Fixed Frame：`map` 或 `odom`
- 轨迹：查看 `/path`、`/path_raw`
- 里程计：查看 `/odom`
- TF：确认 `map -> odom -> base_footprint` 连通

GICP 点云叠图查看：

- 先在 `config.yaml` 打开：

```yaml
lidar.publish_gicp_map: true
```

- RViz Fixed Frame 可设为 `gicp_map`，显示 `/gicp/map_cloud`。
- 如果 Fixed Frame 使用 `map` 或 `odom`，确认 `odom -> gicp_map` TF 正常发布。

## 关键参数

主要参数位于 `src/my_nav2_robot/config/config.yaml`。

常用 GICP 参数：

```yaml
lidar.icp_leaf_size: 0.15
lidar.icp_fitness_threshold: 2.0
lidar.max_gicp_velocity: 2.5
lidar.max_gicp_yaw_rate: 1.5
lidar.max_gicp_yaw_delta: 0.12
lidar.max_gicp_yaw_innovation: 0.025
lidar.max_gicp_velocity_innovation: 0.75
lidar.enable_gicp_local_submap: true
```

常用 deskew 参数：

```yaml
lidar.enable_deskew: true
lidar.state_history_duration: 1.5
```

点云数量过大时，优先调大：

```yaml
lidar.icp_leaf_size
lidar.gicp_map_leaf_size
lidar.gicp_local_submap_leaf_size
```

## 当前效果

从目前调试现象看，直线行驶已经比较稳定，角度漂移较小。GICP local submap 和多级门控能减少明显错配准对状态估计的影响，点云叠图可以用于观察闭环绕圈后的误差。长距离绕圈测试中，约 350 m 路径后出现过 3 m 级别回到起点误差，也出现过误差较小的运行结果，说明系统具备可用基础，但结果仍受点云质量、转弯畸变、GICP 接受门控和参数影响。

当前更可靠的部分：

- 小车直走稳定。
- `/odom`、`/path`、TF 链路基本可用于 Nav2/RViz。
- GICP 坏帧已经有 fitness、速度、yaw、innovation 门控。
- local submap 使用 GICP 校正后的历史帧拼接，而不是直接使用 ESKF 预测拼接。

仍需要重点观察的部分：

- 边走边转时，角度误差可能明显放大。
- 运动畸变仍是主要风险源，尤其是平移 deskew 打开后可能引入额外误差。
- GICP 对环境结构、点云密度、车辆速度和初值较敏感。
- 长距离绕圈后误差有随机性，说明仍存在偶发匹配质量波动或时序/数据同步问题。

## 不足之处

- 当前不是完整闭环 SLAM，没有全局回环优化；绕一大圈回来后的误差只能靠局部匹配和滤波抑制，不能从图优化层面消除。
- `imu_adapter_node` 场景下轮速为 0，和实车 MCU 轮速输入的状态估计效果不同。
- GICP 叠图主要是诊断工具，不应直接等价为高质量全局地图。
- 参数仍偏实验性，不同场地、速度、点云密度下需要重新调门控和噪声。
- `package.xml` 中仍有 TODO 描述和 license，工程元信息还不完整。
- 当前缺少自动化测试和标准化 rosbag 回归脚本，调参效果主要依赖人工观察 RViz 和日志。

## 常用诊断命令

查看 topic：

```bash
ros2 topic list -t
```

查看 TF：

```bash
ros2 run tf2_tools view_frames
```

查看 GICP 状态：

```bash
ros2 topic echo /gicp/status
ros2 topic echo /gicp/innovation_body
ros2 topic echo /gicp/yaw_debug
```

单包构建：

```bash
colcon build --packages-select my_nav2_robot
```
