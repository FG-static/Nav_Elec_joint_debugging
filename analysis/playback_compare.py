#!/usr/bin/env python3
"""回放两个 bag → 启动 ESKF → 订阅 odom → 对比轨迹。"""
import subprocess, time, os, signal, sys, threading
import numpy as np
from pathlib import Path

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
sys.path.insert(0, '/home/goose/ace_ass/install/rm_interfaces/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

SETUP = 'source /opt/ros/jazzy/setup.bash && source /home/goose/ace_ass/install/setup.bash'
CONFIG = '/home/goose/ace_ass/src/my_nav2_robot/config/config.yaml'


class OdomCollector(Node):
    def __init__(self):
        super().__init__('odom_collector')
        self.positions = []; self.orientations = []; self.t_ros = []
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, msg):
        self.positions.append([msg.pose.pose.position.x,
                               msg.pose.pose.position.y,
                               msg.pose.pose.position.z])
        self.orientations.append([msg.pose.pose.orientation.x,
                                  msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w])
        self.t_ros.append(self.get_clock().now().nanoseconds * 1e-9)

    def result(self):
        if len(self.positions) < 10:
            return None
        p = np.array(self.positions); o = np.array(self.orientations)
        x, y, z, w = o[:, 0], o[:, 1], o[:, 2], o[:, 3]
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        t = np.array(self.t_ros); t -= t[0]
        return {'t': t, 'x': p[:, 0], 'y': p[:, 1], 'z': p[:, 2],
                'yaw': yaw, 'n': len(p)}


def run_test(bag_path, label):
    print(f"\n{'='*50}\n  {label}\n{'='*50}")

    # 清理旧进程
    subprocess.run(['pkill', '-9', '-f', 'data_handle_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'ros2.bag'], capture_output=True)
    time.sleep(1)

    # 启动 ESKF 节点
    eskf = subprocess.Popen(
        ['bash', '-c',
         f'{SETUP} && ros2 run my_nav2_robot data_handle_node '
         f'--ros-args -p use_sim_time:=true --params-file {CONFIG}'],
        env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

    # 启动 rclpy collector
    rclpy.init()
    collector = OdomCollector()

    # 启动 bag play
    bag = subprocess.Popen(
        ['bash', '-c', f'{SETUP} && ros2 bag play {bag_path} --clock -r 1.0'],
        env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    # 循环收 odom
    last_n = 0; stall = 0; max_wait = 120
    t_start = time.time()
    while time.time() - t_start < max_wait:
        rclpy.spin_once(collector, timeout_sec=0.05)
        cur = len(collector.positions)
        if cur > last_n:
            last_n = cur; stall = 0; t_start = time.time()
        else:
            stall += 1
            if stall > 60 and cur > 10:
                rclpy.spin_once(collector, timeout_sec=1.0)
                if len(collector.positions) == cur:
                    break

    r = collector.result()
    rclpy.shutdown()

    bag.terminate(); bag.wait()
    eskf.terminate(); eskf.wait()
    time.sleep(1)

    if r:
        dx = r['x'][-1] - r['x'][0]; dy = r['y'][-1] - r['y'][0]
        dist = np.sqrt(dx**2 + dy**2)
        dyaw = np.degrees(r['yaw'][-1] - r['yaw'][0])
        dyaw = (dyaw + 180) % 360 - 180
        max_d = np.degrees(np.max(np.abs(r['yaw'] - r['yaw'][0])))
        print(f"  帧数={r['n']} | 位移={dist:.2f}m | "
              f"航向变化={dyaw:.1f}° | 最大航向偏离={max_d:.1f}°")
        print(f"  起点=({r['x'][0]:.2f},{r['y'][0]:.2f}) "
              f"终点=({r['x'][-1]:.2f},{r['y'][-1]:.2f})")
        np.savez(f'/home/goose/ace_ass/analysis/{label}_odom.npz', **r)
    return r


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default='both')
    args = ap.parse_args()

    bags = {'nav_debug1': '/home/goose/ace_ass/nav_debug1',
            'nav_debug2': '/home/goose/ace_ass/nav_debug2'}
    names = ['nav_debug1', 'nav_debug2'] if args.bag == 'both' else [args.bag]

    results = {}
    for name in names:
        results[name] = run_test(bags[name], name)
