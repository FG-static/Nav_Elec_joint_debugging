#!/usr/bin/env python3
"""
Foxglove CSV 转弯分析工具
对比「wz 积分角度」和「odom 四元数提取的 yaw 变化」，量化 ESKF 转弯精度。

用法 (Kaggle / 本地):
    # 方式1: 直接改下面的文件路径，运行
    # 方式2: python analyze_turns.py /path/to/plot_data.csv -t 0.15
"""
# ====== Kaggle 直接用：在这里填文件路径 ======
CSV_FILE = "/kaggle/input/datasets/firegoose/datefx6/plot_data.csv"
WZ_THRESHOLD = 0.15   # wz 转弯检测阈值 (rad/s)
# =============================================

import sys
import numpy as np
import pandas as pd
from pathlib import Path


def quat_to_yaw(x, y, z, w):
    """
    从四元数提取 yaw (ZYX Euler, ENU convention)
    yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
    yaw ∈ [-π, π]
    """
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw


def unwrap_angles(angles):
    """解包裹角度，消除 ±π 跳变。"""
    diff = np.diff(angles)
    jumps = np.where(np.abs(diff) > np.pi)[0]
    offset = np.zeros(len(angles))
    for j in jumps:
        offset[j + 1:] += -np.sign(diff[j]) * 2 * np.pi
    return angles + offset


def detect_turn_events(t, wz, threshold=0.15, min_duration=0.1, min_gap=0.2):
    """
    检测转弯脉冲事件。
    threshold: |wz| 超过此值视为转弯 (rad/s)
    min_duration: 脉冲最短时长 (s)
    min_gap: 两个脉冲之间最小间隔 (s)，小于此值合并
    """
    active = np.abs(wz) > threshold
    # 找上升沿和下降沿
    edges = np.diff(active.astype(int))
    starts = np.where(edges == 1)[0] + 1
    ends = np.where(edges == -1)[0] + 1

    # 处理边界：记录开始或结束时已在脉冲中
    if active[0]:
        starts = np.concatenate([[0], starts])
    if active[-1]:
        ends = np.concatenate([ends, [len(wz) - 1]])

    if len(starts) == 0 or len(ends) == 0:
        return [], []

    # 合并间隔太短的脉冲
    merged_starts = [starts[0]]
    merged_ends = [ends[0]]
    for i in range(1, len(starts)):
        if t[starts[i]] - t[merged_ends[-1]] < min_gap:
            merged_ends[-1] = ends[i]
        else:
            merged_starts.append(starts[i])
            merged_ends.append(ends[i])

    # 过滤太短的脉冲
    final_starts, final_ends = [], []
    for s, e in zip(merged_starts, merged_ends):
        if t[e] - t[s] >= min_duration:
            final_starts.append(s)
            final_ends.append(e)

    return final_starts, final_ends


def analyze(file_path, threshold=0.15):
    """
    读取 Foxglove CSV，对比 wz 积分与 odom yaw 变化。
    CSV 需包含以下 topic 列：
      /wz.x          — 轮速计 wz (rad/s)
      /odom.pose.pose.orientation.x
      /odom.pose.pose.orientation.y
      /odom.pose.pose.orientation.z
      /odom.pose.pose.orientation.w
    """
    df = pd.read_csv(file_path)

    # 检查必要列
    if 'topic' not in df.columns or 'elapsed time' not in df.columns or 'value' not in df.columns:
        print("错误: CSV 缺少 'topic', 'elapsed time', 'value' 列。请检查 Foxglove 导出格式。")
        return

    # 提取各 topic 数据
    wz_raw = df[df['topic'] == '/wz.x'][['elapsed time', 'value']].copy()
    qx_raw = df[df['topic'] == '/odom.pose.pose.orientation.x'][['elapsed time', 'value']].copy()
    qy_raw = df[df['topic'] == '/odom.pose.pose.orientation.y'][['elapsed time', 'value']].copy()
    qz_raw = df[df['topic'] == '/odom.pose.pose.orientation.z'][['elapsed time', 'value']].copy()
    qw_raw = df[df['topic'] == '/odom.pose.pose.orientation.w'][['elapsed time', 'value']].copy()

    for name, d in [('/wz.x', wz_raw), ('qx', qx_raw), ('qy', qy_raw),
                     ('qz', qz_raw), ('qw', qw_raw)]:
        if len(d) == 0:
            print(f"错误: 未找到 topic {name} 的数据")
            return

    # 重采样到 wz 的时间轴（四元数采样率通常更低，用线性插值）
    t_wz = wz_raw['elapsed time'].values
    wz = wz_raw['value'].values

    # 四元数: 找最晚的 start time 和最早的 end time 作为有效区间
    t_q_min = max(qx_raw['elapsed time'].min(), qy_raw['elapsed time'].min(),
                  qz_raw['elapsed time'].min(), qw_raw['elapsed time'].min())
    t_q_max = min(qx_raw['elapsed time'].max(), qy_raw['elapsed time'].max(),
                  qz_raw['elapsed time'].max(), qw_raw['elapsed time'].max())

    # 限制 wz 在四元数有效时间区间内
    mask = (t_wz >= t_q_min) & (t_wz <= t_q_max)
    t_wz = t_wz[mask]
    wz = wz[mask]

    if len(t_wz) < 10:
        print("有效数据不足")
        return

    # 插值四元数到 wz 时间轴
    qx = np.interp(t_wz, qx_raw['elapsed time'].values, qx_raw['value'].values)
    qy = np.interp(t_wz, qy_raw['elapsed time'].values, qy_raw['value'].values)
    qz = np.interp(t_wz, qz_raw['elapsed time'].values, qz_raw['value'].values)
    qw = np.interp(t_wz, qw_raw['elapsed time'].values, qw_raw['value'].values)

    # 归一化四元数（插值后可能不归一）
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx /= norm; qy /= norm; qz /= norm; qw /= norm

    # 提取 yaw
    yaw = quat_to_yaw(qx, qy, qz, qw)
    yaw_unwrapped = unwrap_angles(yaw)

    # 检测转弯脉冲
    starts, ends = detect_turn_events(t_wz, wz, threshold=threshold)

    if len(starts) == 0:
        print(f"未检测到转弯脉冲 (threshold={threshold:.2f} rad/s)")
        return

    # --- 计算每个转弯的积分对比 ---
    print(f"\n转弯分析: wz 积分 vs odom yaw 变化 (threshold={threshold:.2f})")
    print(f"{'#':<3} {'时间区间':<18} {'时长':<7} {'wz积分':<10} {'yaw变化':<10} {'差值':<10} {'评估'}")
    print("-" * 75)

    total_imu = 0.0
    total_yaw = 0.0

    for i, (s, e) in enumerate(zip(starts, ends)):
        t0, t1 = t_wz[s], t_wz[e]
        dur = t1 - t0

        # wz 积分 (cumsum × dt)
        dt = np.diff(t_wz[s:e + 1])
        wz_mid = 0.5 * (wz[s:e] + wz[s + 1:e + 1])
        imu_rad = np.sum(wz_mid * dt)
        imu_deg = np.degrees(imu_rad)

        # yaw 变化 (from quaternion)
        yaw_start = yaw_unwrapped[s]
        yaw_end = yaw_unwrapped[e]
        yaw_delta_deg = np.degrees(yaw_end - yaw_start)

        # 差值 (wz积分 - yaw变化，正数 = wz 转过头)
        diff_deg = imu_deg - yaw_delta_deg

        total_imu += imu_deg
        total_yaw += yaw_delta_deg

        # 评估
        abs_err = abs(diff_deg)
        if abs_err < 3:
            grade = "优秀"
        elif abs_err < 8:
            grade = "可接受"
        elif abs_err < 20:
            grade = "偏差大"
        else:
            grade = "严重偏差"

        print(f"{i:<3} {t0:>7.2f}-{t1:<7.2f} {dur:<5.2f}s "
              f"{imu_deg:>8.1f}° {yaw_delta_deg:>8.1f}° {diff_deg:>8.1f}°  {grade}")

    print("-" * 75)
    total_diff = total_imu - total_yaw
    print(f"汇总: wz总积分={total_imu:.1f}°, yaw总变化={total_yaw:.1f}°, "
          f"累计误差={total_diff:.1f}° ({abs(total_diff/max(abs(total_yaw),1e-6))*100:.1f}%)")

    return starts, ends, t_wz, wz, yaw_unwrapped


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='分析 Foxglove CSV 转弯精度')
    ap.add_argument('file', nargs='?', default=None,
                    help='Foxglove 导出的 CSV 文件路径（不传则用脚本顶部 CSV_FILE）')
    ap.add_argument('-t', '--threshold', type=float, default=None,
                    help=f'wz 脉冲检测阈值 (rad/s), 默认 {WZ_THRESHOLD}')
    args = ap.parse_args()

    csv_path = args.file or CSV_FILE
    threshold = args.threshold if args.threshold is not None else WZ_THRESHOLD

    if not Path(csv_path).exists():
        print(f"文件不存在: {csv_path}")
        print("用法: 修改脚本顶部 CSV_FILE 变量，或命令行传入路径")
        sys.exit(1)

    analyze(csv_path, threshold=threshold)
