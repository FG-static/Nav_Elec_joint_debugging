# Plan: GICP 迭代校正 deskew 轨迹

## Context

首次 deskew 已经固定为从 `frame.raw_cloud` 出发，使用 IMU scan trajectory 做 rotation-only deskew。
下一步目标是在第一次 GICP 成功后，用 GICP 与 IMU 预测之间的相对运动残差校正 scan 内轨迹，
再从 `frame.raw_cloud` 重新 deskew 一次并重跑 GICP。

这个 plan 不改变 ESKF 主状态，只改 LiDAR 当前帧的 deskew 输入和 GICP 当前帧观测。

## 必须保持的约束

- `deskewCloud()` 必须继续从 `frame.raw_cloud` 读点，禁止从已 deskew 的 `frame.cloud` 叠加处理。
- IMU-only 首次 deskew 继续使用 `deskewCloud(frame, traj, false)`，禁止启用平移 deskew。
- 二次 deskew 只在 GICP 第一次结果通过基础验证后执行，使用 `deskewCloud(frame, corrected_traj, true)`。
- 二次 GICP 的 `score`、transform finite/determinant、速度/yaw gate 必须和一次 GICP 走同一套验证逻辑。
- 最终用于 GICP、history、local submap、map update 的 cloud 必须一致。

## GICP T 约定

当前代码已经把 `T` 作为后续速度观测和 map 更新的统一变换使用：

- frame-to-frame 分支：
  `T = estimate_motion_with_gicp(prev_cloud_, cloud_for_gicp, score, aligned_cloud)`
- local-submap 分支：
  `current_to_submap = estimate_motion_with_gicp(cloud_for_gicp, local_submap_cloud, ...)`
  `T = current_to_submap.inverse() * prev_to_submap`

后续代码对 `T` 的使用方式：

- `v_icp = T.translation() / lidar_dt`
- `updateGicpMap(cloud_for_gicp, T, stamp)`，内部再取 `T.inverse()` 得到 current-to-prev
- `insert_current_to_submap = insert_prev_to_submap * T.inverse()`

因此本轮实现必须沿用现有语义：**`T_gicp` 就是当前代码认可的 prev/current 相对运动观测，不再额外取
`T.inverse()` 作为“真实运动”**。如果后续要彻底重命名方向，必须连速度符号、yaw 符号、map update 一起改，
不能只在 deskew 迭代里反向。

## 改动 1: 保存首次 IMU scan trajectory

在 `data_handle.hpp` 增加成员：

```cpp
std::vector<ScanPoseSample> current_scan_traj_;
```

在 `parseLidarFrame()` 里显式构建首次 IMU trajectory 并保存，避免 `deskewCloud()` 内部构建后外部拿不到：

```cpp
current_scan_traj_.clear();

StateSnapshot anchor;
if (findAnchorState(frame.min_point_stamp_ns, anchor) &&
    buildImuTrajectory(
        anchor, frame.min_point_stamp_ns, frame.max_point_stamp_ns,
        current_scan_traj_)) {

    deskewCloud(frame, current_scan_traj_, false);
} else {

    deskewCloud(frame, {}, false);
}
```

注意：如果 `buildImuTrajectory()` 失败，`current_scan_traj_` 必须保持 empty，后续不得做二次 deskew。

## 改动 2: 增加 GICP 迭代参数

增加参数 `lidar.gicp_iter_count`：

- `1`: 只跑当前一次 GICP，默认值，保持现有行为
- `2`: 第一次 GICP 通过基础验证后，做一次 trajectory correction + re-deskew + 第二次 GICP

先默认 `1`。rosbag A/B 验证后再决定是否把默认值改成 `2`。

## 改动 3: 用相对残差校正 scan trajectory

不要对整条轨迹左乘同一个刚体变换。那只是 world gauge 变化，deskew 依赖的 scan 内相对位姿会大部分抵消。

新增方法建议签名：

```cpp
bool correctScanTrajectoryWithGicp(
    const Eigen::Matrix4d &T_gicp,
    const std::vector<ScanPoseSample> &traj,
    std::vector<ScanPoseSample> &corrected_traj) const;
```

实现原则：

1. 从 `traj.front()` 和 `traj.back()` 构造 body/world pose。
2. 计算 IMU 预测的 scan 首尾相对运动，并转换到和 `T_gicp` 一致的 LiDAR frame 语义。
3. 计算 GICP 相对运动相对 IMU 预测的残差 `T_residual`。
4. 将 `T_residual` 按点时间比例从 0 到 1 分摊到 scan 内：
   - 参考时刻残差为 identity。
   - scan 另一端残差累积到 `T_residual`。
   - 旋转用 quaternion slerp 或 SO(3) log/exp。
   - 平移用线性插值即可，后续需要更严谨时再换 SE(3) log/exp。
5. 把每个分摊残差转换回 body/world pose 后写入 `corrected_traj`。

比例定义必须和 `deskewCloud()` 的参考时刻一致。当前 deskew 参考时刻是 `frame.max_point_stamp_ns`，
所以建议让 max stamp 的校正残差为 identity，让 min stamp 承担完整首尾残差，避免移动参考端。

伪代码结构：

```cpp
// T_pred_lidar: IMU 轨迹预测出的 prev/current LiDAR 相对运动，语义必须与 T_gicp 一致。
Eigen::Matrix4d T_pred_lidar = ...;
Eigen::Matrix4d T_residual = T_gicp * T_pred_lidar.inverse();

for each sample:
    double alpha = (frame.max_point_stamp_ns - sample.stamp_ns) / scan_duration_ns;
    alpha = clamp(alpha, 0.0, 1.0);

    Eigen::Quaterniond q_res_i =
        Eigen::Quaterniond::Identity().slerp(alpha, q_residual);
    Eigen::Vector3d t_res_i = alpha * t_residual;

    // 将 LiDAR frame 残差映射到该 sample 对应的 body/world pose 修正。
    // 这里要显式处理 R_lidar_to_body_，不要把 lidar-frame T 直接左乘 world pose。
```

实现时如果坐标系推导不够确定，先只做 rotation residual correction，平移残差保留但不启用；
这比把错方向平移写进 trajectory 更安全。

## 改动 4: 抽出 GICP 单次运行与基础验证

为了让第一次和第二次 GICP 走同一套基础验证，抽一个局部 lambda 或私有方法：

```cpp
struct GicpRunResult {
    bool ok = false;
    double score = 1e9;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d current_to_submap = Eigen::Matrix4d::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;
};
```

`runGicpOnce(cloud)` 负责：

- frame-to-frame 和 local-submap 两个分支都返回统一语义的 `T`
- 复用已有 `gicp_init_guess_`
- 检查 `score`、`lidar_dt`、`T.allFinite()`、rotation determinant
- 失败时返回 `ok=false`，外层执行现有 clear/reset/status 逻辑

第二次 GICP 只有在第一次 `ok=true` 后才允许执行。第二次如果失败：

- 保守策略：回退第一次 GICP 结果和第一次 deskew cloud
- 不用失败的二次结果更新观测、map、submap 或 history

## 改动 5: `lidarCallback()` 迭代流程

目标流程：

```cpp
auto first_cloud_full = frame.cloud;
auto first_cloud_for_gicp = limitCloudIfNeeded(first_cloud_full);
auto first_cloud_for_history = first_cloud_for_gicp;

GicpRunResult best = runGicpOnce(first_cloud_for_gicp);
auto final_cloud_for_gicp = first_cloud_for_gicp;
auto final_cloud_for_history = first_cloud_for_history;

if (best.ok &&
    lidar_gicp_iter_count_ >= 2 &&
    frame.deskewed &&
    !current_scan_traj_.empty()) {

    std::vector<ScanPoseSample> corrected_traj;
    if (correctScanTrajectoryWithGicp(best.T, current_scan_traj_, corrected_traj) &&
        deskewCloud(frame, corrected_traj, true)) {

        auto second_cloud_for_gicp = limitCloudIfNeeded(frame.cloud);
        GicpRunResult second = runGicpOnce(second_cloud_for_gicp);

        if (second.ok) {
            best = second;
            final_cloud_for_gicp = second_cloud_for_gicp;
            final_cloud_for_history = second_cloud_for_gicp;
        } else {
            frame.cloud = first_cloud_full;
            final_cloud_for_gicp = first_cloud_for_gicp;
            final_cloud_for_history = first_cloud_for_history;
        }
    }
}

// 后续 v_icp / yaw / observe / map / submap / prev_cloud_
// 全部使用 best.T、best.score、final_cloud_for_gicp、final_cloud_for_history。
```

注意：

- 二次 re-deskew 后必须重新走点数限制，不能直接把完整 `frame.cloud` 塞给 GICP。
- `prev_cloud_`、`prev_lidar_frame_.cloud`、local submap 插入、`updateGicpMap()` 必须使用最终胜出的 cloud。
- 如果二次失败并回退第一次，`frame.cloud` 也要恢复到 first deskew cloud，保证日志和 history 一致。

## 改动 6: 日志与诊断

增加节流日志，方便 rosbag 对比：

- `gicp_iter_count`
- first/second score
- first/second cost ms
- 是否采用 second result
- second reject reason
- re-deskew 是否成功

原有 `ICP RESULT` 可以扩展为：

```text
ICP RESULT: score=... first_score=... iter=1/2 iter_used=0/1 ...
```

## 验证

1. 静态检查：
   - `deskewCloud()` 仍然只从 `frame.raw_cloud` 读点。
   - IMU-only 首次 deskew 没有开启 translation。
   - 二次 GICP 失败不会污染 `prev_cloud_`、map、local submap。
2. 构建：
   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select my_nav2_robot
   ```
3. rosbag A/B：
   - `lidar.gicp_iter_count=1` vs `2`
   - GICP reject 率
   - first/second score 分布
   - `gicp` 耗时和 dropped scan 数
   - 转弯段轨迹平滑度
   - 一圈闭环误差
4. 安全回归：
   - 高速转弯、原地旋转、点云缺 timestamp、IMU trajectory 构建失败、local submap 开/关都要覆盖。
