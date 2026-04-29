#!/usr/bin/env python3
"""深度 /wz 分析：回放 bag → 收集 /wz + /odom → 逐帧对比三通道"""
import subprocess, time, os, sys
import numpy as np

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

SETUP = 'source /opt/ros/jazzy/setup.bash && source /home/goose/ace_ass/install/setup.bash'
CONFIG = '/home/goose/ace_ass/src/my_nav2_robot/config/config.yaml'


class Collector(Node):
    def __init__(self):
        super().__init__('collector')
        self.wz = {'t': [], 'x': [], 'y': [], 'z': []}
        self.odom = {'t': [], 'x': [], 'y': [], 'qz': [], 'qw': []}
        self.create_subscription(Vector3, '/wz', self.wz_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def wz_cb(self, msg):
        self.wz['t'].append(self.get_clock().now().nanoseconds * 1e-9)
        self.wz['x'].append(msg.x)
        self.wz['y'].append(msg.y)
        self.wz['z'].append(msg.z)

    def odom_cb(self, msg):
        self.odom['t'].append(self.get_clock().now().nanoseconds * 1e-9)
        self.odom['x'].append(msg.pose.pose.position.x)
        self.odom['y'].append(msg.pose.pose.position.y)
        o = msg.pose.pose.orientation
        self.odom['qz'].append(o.z)
        self.odom['qw'].append(o.w)


def run_bag(bag_path, label):
    subprocess.run(['pkill', '-9', '-f', 'data_handle_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'ros2.bag'], capture_output=True)
    time.sleep(1)

    eskf = subprocess.Popen(
        ['bash', '-c', f'{SETUP} && ros2 run my_nav2_robot data_handle_node '
         f'--ros-args -p use_sim_time:=true --params-file {CONFIG}'],
        env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

    rclpy.init()
    col = Collector()

    bp = subprocess.Popen(
        ['bash', '-c', f'{SETUP} && ros2 bag play {bag_path} --clock -r 1.0'],
        env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    last_n = 0; stall = 0
    while True:
        rclpy.spin_once(col, timeout_sec=0.05)
        cur = len(col.wz['t'])
        if cur > last_n: last_n = cur; stall = 0
        else:
            stall += 1
            if stall > 60 and cur > 10: break

    bp.terminate(); bp.wait()
    rclpy.shutdown()
    eskf.terminate(); eskf.wait()
    time.sleep(1)

    wz = {k: np.array(v) for k, v in col.wz.items()}
    od = {k: np.array(v) for k, v in col.odom.items()}
    wz['t'] -= wz['t'][0]
    if len(od['t']) > 0: od['t'] -= od['t'][0]

    np.savez(f'/home/goose/ace_ass/analysis/{label}_deep.npz', **{f'wz_{k}': v for k, v in wz.items()},
             **{f'od_{k}': v for k, v in od.items()})
    return wz, od


def analyze(wz, od, label):
    t = wz['t']
    wx, wy, wz_f = wz['x'], wz['y'], wz['z']
    N = len(t)
    dt = np.median(np.diff(t))

    print(f"\n{'='*70}")
    print(f"  {label}  ({N} 帧, {t[-1]:.1f}s, dt≈{dt*1000:.1f}ms)")
    print(f"{'='*70}")

    # ====== 基础统计 ======
    print(f"\n  全局统计:")
    for name, sig in [('x(wheel)', wx), ('y(imu)', wy), ('z(fused)', wz_f)]:
        print(f"    {name:12s}: mean={sig.mean():+.6f}  std={sig.std():.6f}  "
              f"min={sig.min():.4f}  max={sig.max():.4f}")

    # ====== 直走 vs 转弯 分段 ======
    # 用 max(|x|,|y|) 来判断（两传感器同时看）
    wz_abs = np.maximum(np.abs(wx), np.abs(wy))
    straight = wz_abs < 0.1
    turning = wz_abs > 0.2

    n_s = np.sum(straight); n_t = np.sum(turning)
    print(f"\n  分段: 直走 {n_s} 帧 ({100*n_s/N:.0f}%), 转弯 {n_t} 帧 ({100*n_t/N:.0f}%)")

    if n_s > 100:
        print(f"\n  直走段统计:")
        for name, sig in [('x(wheel)', wx), ('y(imu)', wy), ('z(fused)', wz_f)]:
            s = sig[straight]
            print(f"    {name:12s}: mean={s.mean():+.8f}  std={s.std():.6f}  "
                  f"RMS={np.sqrt(np.mean(s**2)):.6f}")

        # x vs y 差异
        diff_xy = wx[straight] - wy[straight]
        print(f"    x-y 差值:     mean={diff_xy.mean():+.8f}  std={diff_xy.std():.6f}")

        # z vs x, z vs y
        diff_zx = wz_f[straight] - wx[straight]
        diff_zy = wz_f[straight] - wy[straight]
        print(f"    z-x 差值:     mean={diff_zx.mean():+.8f}  std={diff_zx.std():.6f}")
        print(f"    z-y 差值:     mean={diff_zy.mean():+.8f}  std={diff_zy.std():.6f}")

    # ====== 转弯事件逐个分析 ======
    if n_t > 50:
        edges = np.diff(turning.astype(int))
        starts = np.where(edges == 1)[0] + 1
        ends = np.where(edges == -1)[0] + 1
        if turning[0]: starts = np.concatenate([[0], starts])
        if turning[-1]: ends = np.concatenate([ends, [N-1]])

        # 合并近邻
        ms, me = [starts[0]], [ends[0]]
        for i in range(1, len(starts)):
            if t[starts[i]] - t[me[-1]] < 0.3:
                me[-1] = ends[i]
            else:
                ms.append(starts[i]); me.append(ends[i])

        print(f"\n  转弯事件 ({len(ms)} 次):")
        print(f"  {'#':>2} {'时间':<14} {'时长':>5} {'wheel':>8} {'imu':>8} {'fused':>8}  {'fused-imu':>10}")
        for i, (s, e) in enumerate(zip(ms, me)):
            dur = t[e] - t[s]
            if dur < 0.05: continue
            dt_seg = np.diff(t[s:e+1])
            d_x = np.degrees(np.sum(0.5*(wx[s:e]+wx[s+1:e+1]) * dt_seg))
            d_y = np.degrees(np.sum(0.5*(wy[s:e]+wy[s+1:e+1]) * dt_seg))
            d_z = np.degrees(np.sum(0.5*(wz_f[s:e]+wz_f[s+1:e+1]) * dt_seg))
            diff = d_z - d_y
            flag = " ⚠️" if abs(diff) > 10 else ""
            print(f"  {i:>2} {t[s]:>6.1f}-{t[e]:<6.1f} {dur:>4.2f}s "
                  f"{d_x:>7.1f}° {d_y:>7.1f}° {d_z:>7.1f}°  {diff:>+8.1f}°{flag}")

    # ====== odom yaw 积分 vs fused 积分 对比 ======
    if len(od['t']) > 10:
        yaw_od = np.arctan2(2*od['qw']*od['qz'], 1 - 2*od['qz']**2)
        # unwrap
        dy = np.diff(yaw_od); dy = np.arctan2(np.sin(dy), np.cos(dy))
        yaw_uw = np.concatenate([[yaw_od[0]], yaw_od[0] + np.cumsum(dy)])

        # 积分 fused
        dt_wz = np.diff(t)
        fused_int = np.concatenate([[0], np.cumsum(0.5*(wz_f[:-1]+wz_f[1:]) * dt_wz)])

        # 积分 imu (y channel)
        imu_int = np.concatenate([[0], np.cumsum(0.5*(wy[:-1]+wy[1:]) * dt_wz)])

        # 积分 wheel (x channel)
        wheel_int = np.concatenate([[0], np.cumsum(0.5*(wx[:-1]+wx[1:]) * dt_wz)])

        print(f"\n  累积角度 (相对起点):")
        print(f"    wheel 积分:  {np.degrees(wheel_int[-1]):+.1f}°")
        print(f"    imu 积分:    {np.degrees(imu_int[-1]):+.1f}°")
        print(f"    fused 积分:  {np.degrees(fused_int[-1]):+.1f}°")

        # odom yaw at wz timestamps
        yaw_on_t = np.interp(t, od['t'], yaw_uw) - yaw_uw[0]
        print(f"    odom yaw:    {np.degrees(yaw_on_t[-1]):+.1f}°")

        err_fused = np.degrees(fused_int[-1] - yaw_on_t[-1])
        err_imu = np.degrees(imu_int[-1] - yaw_on_t[-1])
        print(f"    fused-odom:  {err_fused:+.1f}°")
        print(f"    imu-odom:    {err_imu:+.1f}°")


if __name__ == '__main__':
    bags = {'nav_debug1': '/home/goose/ace_ass/nav_debug1',
            'nav_debug2': '/home/goose/ace_ass/nav_debug2'}

    for name, path in bags.items():
        wz, od = run_bag(path, name)
        analyze(wz, od, name)
