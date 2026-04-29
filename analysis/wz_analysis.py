#!/usr/bin/env python3
"""回放 bag + 启动 ESKF + 记录 /wz 三通道融合数据"""
import subprocess, time, os, sys
import numpy as np

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

SETUP = 'source /opt/ros/jazzy/setup.bash && source /home/goose/ace_ass/install/setup.bash'
CONFIG = '/home/goose/ace_ass/src/my_nav2_robot/config/config.yaml'


class WzCollector(Node):
    def __init__(self):
        super().__init__('wz_collector')
        self.wz_x = []; self.wz_y = []; self.wz_z = []
        self.t = []
        self.sub = self.create_subscription(Vector3, '/wz', self.cb, 10)

    def cb(self, msg):
        self.wz_x.append(msg.x); self.wz_y.append(msg.y); self.wz_z.append(msg.z)
        self.t.append(self.get_clock().now().nanoseconds * 1e-9)


def run(bag_path, label):
    print(f"\n>>> {label}")

    subprocess.run(['pkill', '-9', '-f', 'data_handle_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'ros2.bag'], capture_output=True)
    time.sleep(1)

    # start ESKF
    subprocess.Popen(
        ['bash', '-c',
         f'{SETUP} && ros2 run my_nav2_robot data_handle_node '
         f'--ros-args -p use_sim_time:=true --params-file {CONFIG}'],
        env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

    rclpy.init()
    col = WzCollector()

    # start bag
    bp = subprocess.Popen(
        ['bash', '-c', f'{SETUP} && ros2 bag play {bag_path} --clock -r 1.0'],
        env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    last_n = 0; stall = 0
    while True:
        rclpy.spin_once(col, timeout_sec=0.05)
        cur = len(col.t)
        if cur > last_n: last_n = cur; stall = 0
        else:
            stall += 1
            if stall > 60 and cur > 10: break

    bp.terminate(); bp.wait()
    rclpy.shutdown()
    time.sleep(1)

    t = np.array(col.t); t -= t[0]
    x, y, z = np.array(col.wz_x), np.array(col.wz_y), np.array(col.wz_z)

    # --- analysis ---
    print(f"  帧数: {len(t)}, 时长: {t[-1]:.1f}s")

    # straight vs turn
    turning = np.abs(x) > 0.15
    straight = (~turning) & (np.abs(x) < 0.05)
    s_idx = np.where(straight)[0]

    for name, sig in [('x(wheel)', x), ('y(imu)', y), ('z(fused)', z)]:
        rms = np.sqrt(np.mean(sig**2))
        mean = np.mean(sig)
        if len(s_idx) > 100:
            srms = np.sqrt(np.mean(sig[s_idx]**2))
            smean = np.mean(sig[s_idx])
            print(f"  {name:12s}: 全局 RMS={rms:.4f} mean={mean:.4f}  "
                  f"直走 RMS={srms:.4f} mean={smean:.4f}")
        else:
            print(f"  {name:12s}: 全局 RMS={rms:.4f} mean={mean:.4f}")

    # x vs y correlation in straight
    if len(s_idx) > 100:
        corr = np.corrcoef(x[s_idx], y[s_idx])[0, 1]
        diff = x[s_idx] - y[s_idx]
        print(f"  x-y 直走相关性: {corr:.3f}, 差值 RMS={np.sqrt(np.mean(diff**2)):.4f}")

    # find largest discrepancy segments
    diff_xy = np.abs(x - y)
    bad = np.where(diff_xy > 0.1)[0]
    if len(bad) > 0:
        print(f"  x-y 差异 >0.1 rad/s: {len(bad)} 帧 ({100*len(bad)/len(t):.1f}%)")

    # find z (fused) vs raw difference
    diff_zx = z - x  # fused minus wheel
    diff_zy = z - y  # fused minus imu
    print(f"  z(fused)-x(wheel): mean={np.mean(diff_zx):.4f}, std={np.std(diff_zx):.4f}")
    print(f"  z(fused)-y(imu):   mean={np.mean(diff_zy):.4f}, std={np.std(diff_zy):.4f}")

    # 转弯分析
    t_idx = np.where(turning)[0]
    if len(t_idx) > 50:
        # 每次转弯的积分
        edges = np.diff(turning.astype(int))
        starts = np.where(edges == 1)[0] + 1
        ends = np.where(edges == -1)[0] + 1
        if turning[0]: starts = np.concatenate([[0], starts])
        if turning[-1]: ends = np.concatenate([ends, [len(t)-1]])
        print(f"\n  转弯事件 ({len(starts)} 次):")
        for i, (s, e) in enumerate(zip(starts, ends)):
            dt = np.diff(t[s:e+1]); mid_x = 0.5*(x[s:e]+x[s+1:e+1])
            d_wheel = np.sum(mid_x*dt)  # wheel integral
            mid_z = 0.5*(z[s:e]+z[s+1:e+1])
            d_fused = np.sum(mid_z*dt)  # fused integral
            d_imu = np.sum(0.5*(y[s:e]+y[s+1:e+1])*dt)
            print(f"    转弯{i}: {t[s]:.1f}-{t[e]:.1f}s ({t[e]-t[s]:.2f}s) "
                  f"wheel={np.degrees(d_wheel):.0f}° imu={np.degrees(d_imu):.0f}° "
                  f"fused={np.degrees(d_fused):.0f}°")

    np.savez(f'/home/goose/ace_ass/analysis/{label}_wz.npz', t=t, x=x, y=y, z=z)
    return t, x, y, z


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default='both')
    args = ap.parse_args()

    bags = {'nav_debug1': '/home/goose/ace_ass/nav_debug1',
            'nav_debug2': '/home/goose/ace_ass/nav_debug2'}
    names = ['nav_debug1', 'nav_debug2'] if args.bag == 'both' else [args.bag]

    for name in names:
        run(bags[name], name)
