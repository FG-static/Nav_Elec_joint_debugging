#!/usr/bin/env python3
"""回放 bag + 订阅 Gimbal + 保存 numpy 数据供离线分析。"""
import sys
import signal
import numpy as np
from pathlib import Path
import subprocess
import time
import os

sys.path.insert(0, '/home/goose/ace_ass/install/rm_interfaces/lib/python3.12/site-packages')
sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import Gimbal


class BagRecorder(Node):
    def __init__(self, output_path):
        super().__init__('bag_recorder')
        self.data = {'t_sec': [], 't_ms': [],
                     'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
                     'acc_x': [], 'acc_y': [], 'acc_z': [],
                     'w_fl': [], 'w_fr': [], 'w_rl': [], 'w_rr': []}
        self.output_path = output_path
        self.sub = self.create_subscription(
            Gimbal, '/tracker/gimbal', self.cb, 10)
        self.done = False
        self.start_time = None

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        self.data['t_sec'].append(now - self.start_time)
        self.data['t_ms'].append(msg.t_ms)
        self.data['gyro_x'].append(msg.angular_velocity.x)
        self.data['gyro_y'].append(msg.angular_velocity.y)
        self.data['gyro_z'].append(msg.angular_velocity.z)
        self.data['acc_x'].append(msg.linear_acceleration.x)
        self.data['acc_y'].append(msg.linear_acceleration.y)
        self.data['acc_z'].append(msg.linear_acceleration.z)
        self.data['w_fl'].append(msg.wheel_velocity.x)
        self.data['w_fr'].append(msg.wheel_velocity.y)
        self.data['w_rl'].append(msg.wheel_velocity.z)
        self.data['w_rr'].append(msg.wheel_velocity.w)

    def save(self):
        d = {k: np.array(v) for k, v in self.data.items()}
        np.savez(self.output_path, **d)
        self.get_logger().info(f"保存 {len(d['t_sec'])} 帧 → {self.output_path}")


def main():
    rclpy.init()
    bag_path = sys.argv[1] if len(sys.argv) > 1 else '/home/goose/ace_ass/nav_debug1'
    output = sys.argv[2] if len(sys.argv) > 2 else '/tmp/bag_data.npz'

    recorder = BagRecorder(output)

    # 启动 bag play
    bag_proc = subprocess.Popen(
        ['ros2', 'bag', 'play', bag_path, '--clock', '-r', '1.0'],
        env={**os.environ}
    )

    # 等待 bag 播放完成
    try:
        timeout = 120
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            if bag_proc.poll() is not None and len(recorder.data['t_sec']) > 10:
                time.sleep(0.5)
                break
    except KeyboardInterrupt:
        pass

    rclpy.spin_once(recorder, timeout_sec=0.1)

    recorder.save()
    bag_proc.terminate()
    bag_proc.wait()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
