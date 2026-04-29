#!/usr/bin/env python3
"""对比 /odom vs /odom_raw：找出 ESKF 观测更新到底破坏了多少 heading"""
import subprocess, time, os, sys
import numpy as np
sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

SETUP = 'source /opt/ros/jazzy/setup.bash && source /home/goose/ace_ass/install/setup.bash'
CONFIG = '/home/goose/ace_ass/src/my_nav2_robot/config/config.yaml'

class C(Node):
    def __init__(self):
        super().__init__('odom_cmp')
        self.od = {'t':[], 'qz':[], 'qw':[], 'qx':[], 'qy':[], 'x':[], 'y':[]}
        self.raw = {'t':[], 'qz':[], 'qw':[], 'qx':[], 'qy':[], 'x':[], 'y':[]}
        self.create_subscription(Odometry, '/odom', lambda m: self._cb(m, self.od), 10)
        self.create_subscription(Odometry, '/odom_raw', lambda m: self._cb(m, self.raw), 10)
    def _cb(self, m, d):
        d['t'].append(self.get_clock().now().nanoseconds*1e-9)
        o = m.pose.pose.orientation
        d['qx'].append(o.x); d['qy'].append(o.y)
        d['qz'].append(o.z); d['qw'].append(o.w)
        d['x'].append(m.pose.pose.position.x)
        d['y'].append(m.pose.pose.position.y)

def yaw_from_q(qx, qy, qz, qw):
    return np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

def unwrap(yaw):
    dy = np.diff(yaw)
    dy = np.arctan2(np.sin(dy), np.cos(dy))
    return np.concatenate([[yaw[0]], yaw[0] + np.cumsum(dy)])

def run(bag_path, label):
    subprocess.run(['pkill','-9','-f','data_handle_node'], capture_output=True)
    subprocess.run(['pkill','-9','-f','ros2.bag'], capture_output=True)
    time.sleep(1)

    eskf = subprocess.Popen(['bash','-c',
        f'{SETUP} && ros2 run my_nav2_robot data_handle_node '
        f'--ros-args -p use_sim_time:=true --params-file {CONFIG}'],
        env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

    rclpy.init()
    c = C()

    bag = subprocess.Popen(['bash','-c',
        f'{SETUP} && ros2 bag play {bag_path} --clock -r 1.0'],
        env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    last=0; stall=0
    while True:
        rclpy.spin_once(c, timeout_sec=0.05)
        cur = len(c.od['t'])
        if cur > last: last=cur; stall=0
        else:
            stall += 1
            if stall > 60 and cur > 10: break

    bag.terminate(); bag.wait()
    rclpy.shutdown()
    eskf.terminate(); eskf.wait()
    time.sleep(1)

    print(f"\n{'='*60}\n  {label}\n{'='*60}")
    for name, d in [('/odom', c.od), ('/odom_raw', c.raw)]:
        if len(d['t']) < 10:
            print(f"  {name}: 数据不足 ({len(d['t'])} 帧)")
            continue
        t = np.array(d['t']); t -= t[0]
        qx,qy,qz,qw = [np.array(d[k]) for k in ['qx','qy','qz','qw']]
        x, y = np.array(d['x']), np.array(d['y'])
        yaw = yaw_from_q(qx, qy, qz, qw)
        yaw_uw = unwrap(yaw)
        disp = np.sqrt((x[-1]-x[0])**2 + (y[-1]-y[0])**2)
        dyaw = np.degrees(yaw_uw[-1] - yaw_uw[0])
        print(f"  {name:12s}: {len(t):>5} 帧 | 位移={disp:.2f}m | "
              f"yaw 变化={dyaw:+.1f}° | 终点=({x[-1]:.2f},{y[-1]:.2f})")

    # 保存对比数据
    np.savez(f'/home/goose/ace_ass/analysis/{label}_cmp.npz',
             od_t=np.array(c.od['t']), od_qz=np.array(c.od['qz']), od_qw=np.array(c.od['qw']),
             od_qx=np.array(c.od['qx']), od_qy=np.array(c.od['qy']),
             od_x=np.array(c.od['x']), od_y=np.array(c.od['y']),
             raw_t=np.array(c.raw['t']), raw_qz=np.array(c.raw['qz']), raw_qw=np.array(c.raw['qw']),
             raw_qx=np.array(c.raw['qx']), raw_qy=np.array(c.raw['qy']),
             raw_x=np.array(c.raw['x']), raw_y=np.array(c.raw['y']))

if __name__ == '__main__':
    bags = {'nav_debug1': '/home/goose/ace_ass/nav_debug1',
            'nav_debug2': '/home/goose/ace_ass/nav_debug2'}
    for name, path in bags.items():
        run(path, name)
