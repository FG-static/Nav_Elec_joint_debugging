# PLAN

## 目标

将当前项目的点云去畸变流程，从“ESKF 历史姿态旋转 deskew + GICP 通过后补平移 deskew”，逐步改成更接近 DLIO 的“基于 scan 内连续时间运动轨迹的 6DoF deskew，再做 GICP”。

本计划只针对最小可落地版本，不一次性重构成完整 DLIO。第一阶段目标是先解决边走边转时的点云畸变和角度误差放大问题，并尽量保持现有 ESKF、GICP 门控和 local submap 结构不变。

## 当前实现概况

- 当前主入口在 `src/my_nav2_robot/src/data_handle.cpp`
- 当前点云处理流程：
  - `parseLidarFrame()` 解析点云并做旋转 deskew
  - `estimateLidarMotion()` 用状态历史给 GICP 初值
  - `estimate_motion_with_gicp()` 做 scan-to-prev / scan-to-local-submap 配准
  - `applyGicpTranslationDeskew()` 在 GICP 成功后做补偿式平移 deskew
- 当前状态历史只保存 `p/q`，不具备 scan 内连续时间轨迹表达能力

## 第一阶段原则

- 先把完整 6DoF deskew 前移到 GICP 之前
- 先保留现有 GICP 门控、local submap、ESKF 框架
- 暂不同时引入 keyframe map、pose graph、回环优化
- 一次只替换一层，避免多个变量同时变化导致无法诊断

## 任务清单

### 1. 扩展状态历史数据

需要修改：

- `src/my_nav2_robot/include/my_nav2_robot/data_handle.hpp`
- `src/my_nav2_robot/src/data_handle.cpp`

具体内容：

- 扩展 `StateSnapshot`
- 在当前的 `stamp_ns / p / q` 基础上，至少增加：
  - `v`
  - `b_a`
  - `b_g`
- 可选增加：
  - `acc_unbiased`
  - `gyro_unbiased`

目的：

- 为 scan 内连续时间轨迹构建提供足够的状态量

### 2. 扩展状态历史写入逻辑

需要修改：

- `pushStateHistory()`

具体内容：

- 当前只保存 `p_` 和 `q_`
- 改为保存当前 ESKF 名义状态：
  - `p_`
  - `q_`
  - `v_`
  - `b_a_`
  - `b_g_`

目的：

- 让后续轨迹重建不再依赖简化的两端位姿

### 3. 增加完整状态插值接口

需要修改：

- `interpolateState()`
- 或新增 `interpolateStateFull()`

具体内容：

- 保留现有 `p/q` 插值接口，避免影响旧逻辑
- 新增一个返回完整状态的插值接口，用于：
  - `p`
  - `q`
  - `v`
  - `b_a`
  - `b_g`

目的：

- 给 scan trajectory 构造提供统一接口

### 4. 引入 scan 内轨迹数据结构

需要修改：

- `src/my_nav2_robot/include/my_nav2_robot/data_handle.hpp`

具体内容：

- 新增一个 scan 轨迹采样结构，例如：
  - `stamp_ns`
  - `p`
  - `q`
  - `v`

目的：

- 将“状态历史”与“本帧点云内部轨迹”区分开

### 5. 新增 scan trajectory 构建函数

需要新增函数：

- `buildScanTrajectory(const LidarFrame &frame, std::vector<ScanPoseSample> &traj)`

建议位置：

- `src/my_nav2_robot/src/data_handle.cpp`

第一版实现要求：

- 输入一帧 scan 的 `min_point_stamp_ns` 和 `max_point_stamp_ns`
- 从状态历史中提取覆盖该时间段的状态
- 生成一条 scan 内部可查询的连续轨迹

第一版可以接受的简化：

- 先做“稠密状态插值轨迹”
- 不要求第一版就实现严格 IMU 重积分

后续增强方向：

- 基于 scan 内 IMU propagation 重建更真实的连续时间轨迹

### 6. 统一 deskew 参考时刻

需要明确的设计选择：

- 全部点 deskew 到 `frame.max_point_stamp_ns`

原因：

- 与当前工程逻辑最接近
- 可最小化对 GICP、观测融合和历史点云流程的改动

要求：

- `parseLidarFrame()`
- GICP 输出解释
- 观测对齐时刻

以上都必须采用同一个参考时刻语义

### 7. 重写 parseLidarFrame() 中的 deskew 主体

需要修改：

- `parseLidarFrame()`

当前逻辑：

- 每点读取 `q_i`
- 只做旋转 deskew
- 平移 deskew 被推迟到 `applyGicpTranslationDeskew()`

目标逻辑：

- 基于 scan trajectory，对每个点执行完整 6DoF deskew
- 每个点都使用：
  - 该点时刻的 `p_i / q_i`
  - 参考时刻的 `p_ref / q_ref`

核心计算流程：

- `point_body = R_lidar_to_body_ * point_lidar`
- `point_world = q_i * point_body + p_i`
- `point_ref_body = q_ref.conjugate() * (point_world - p_ref)`
- `point_ref_lidar = R_lidar_to_body_.transpose() * point_ref_body`

注意事项：

- 不要再只做旋转 deskew
- 不要把平移 deskew 留到 GICP 之后

### 8. deskew 过程中不要逐点直接查状态历史

需要优化：

- 避免在 `parseLidarFrame()` 中对每个点直接调用一次 `interpolateState()`

建议做法：

- 先构造 scan trajectory
- 再按时间索引或局部插值查 `p_i / q_i`

目的：

- 降低逐点查表开销
- 让 deskew 语义变成“scan 内连续轨迹”，而不是“离散历史点随用随查”

### 9. 废弃补偿式平移 deskew 路径

需要处理：

- `applyGicpTranslationDeskew()`
- `deskew_translation_`
- `gicp_deskew_anchor_`

第一阶段建议：

- 停用 `applyGicpTranslationDeskew()` 主路径
- 保留代码一段时间用于 A/B 对比，但不再作为默认流程

原因：

- 该路径属于“GICP 成功后补偿式 deskew”
- 不符合 DLIO 的“先连续时间 6DoF deskew，再 GICP”思想

### 10. 统一历史点云来源

需要检查：

- `cloud_for_gicp`
- `cloud_for_history`
- `prev_cloud_`
- `prev_lidar_frame_.cloud`
- local submap 插入逻辑
- `/gicp/map_cloud` 发布逻辑

目标：

- 统一使用“完整 6DoF deskew 后”的点云
- 不再混用：
  - 原始 raw cloud
  - 仅旋转 deskew cloud
  - GICP 后补平移 deskew cloud

### 11. 保留现有 GICP 初值和门控逻辑

第一阶段先保留：

- `estimateLidarMotion()`
- `fitness` 门控
- `velocity_gate`
- `yaw_gate`
- `velocity_valid`
- `yaw_valid`
- local submap 机制

原因：

- 第一阶段目的是隔离 deskew 对系统的影响
- 避免同时修改太多因素导致无法判断收益来源

### 12. 第二阶段再考虑的内容

第一阶段完成后再评估是否需要继续做：

- 将 local submap 从最近几帧滑窗改成 keyframe submap
- 将 GICP 从“速度/yaw观测”升级成“pose 观测”
- 用更严格的 IMU propagation 替代当前插值式 scan trajectory
- 引入退化检测
- 引入全局回环或图优化

## 实施顺序

建议按下面顺序进行：

1. 扩展 `StateSnapshot`
2. 扩展 `pushStateHistory()`
3. 增加完整状态插值接口
4. 新增 scan trajectory 数据结构
5. 实现 `buildScanTrajectory()`
6. 重写 `parseLidarFrame()`，完成 6DoF deskew
7. 暂停 `applyGicpTranslationDeskew()` 主路径
8. 统一 `prev_cloud_ / local submap / map cloud` 的点云来源
9. 保持现有 GICP 和门控不变，进行 A/B 测试

## 验证项目

修改完成后重点观察：

1. RViz 中转弯时 `/livox/lidar/pointcloud` 的拖影是否明显减少
2. `/gicp/yaw_debug` 是否更平滑
3. `/gicp/status` 中 reject 是否减少
4. 绕圈回起点时的角度误差是否缩小
5. 直走性能是否保持稳定，不出现新的系统性偏航

## 风险点

- 点时间戳字段语义不一致：绝对时间、相对时间、offset_time 需要统一确认
- `R_lidar_to_body_` 外参一旦方向用错，6DoF deskew 会在转弯时明显恶化
- 若 scan trajectory 参考时刻定义不统一，GICP 结果和滤波器时刻会错位
- 如果一次性同时修改 deskew、submap 和 GICP 融合方式，将很难定位退化来源

## 结论

第一阶段不追求完整复刻 DLIO，而是先把当前项目从“后验补偿式 deskew”升级成“GICP 前的连续时间 6DoF deskew”。这是当前最有可能改善转弯畸变和角度误差的改动，也是对现有工程侵入最小的一条路径。
