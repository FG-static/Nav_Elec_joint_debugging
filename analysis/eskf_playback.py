#!/usr/bin/env python3
"""回放 rosbag → 运行 ESKF → 录制 odom → 对比路径。"""
import subprocess, time, os, sys, signal
import numpy as np
from pathlib import Path

ROS2_SETUP = 'source /opt/ros/jazzy/setup.bash && source /home/goose/ace_ass/install/setup.bash'
BAGS = {
    'nav_debug1': '/home/goose/ace_ass/nav_debug1',
    'nav_debug2': '/home/goose/ace_ass/nav_debug2',
}
OUT_DIR = Path('/home/goose/ace_ass/analysis')


def run_playback(bag_name, bag_path, out_bag):
    """播放 bag + 启动 ESKF + 录制 /odom。返回录制的 bag 路径。"""
    out_bag = str(OUT_DIR / out_bag)
    # 清理旧输出
    if Path(out_bag).exists():
        subprocess.run(['rm', '-rf', out_bag])

    # 启动 ros2 bag record（先启动，确保不丢帧）
    record_cmd = (
        f'{ROS2_SETUP} && '
        f'ros2 bag record -o {out_bag} /odom /bias_gyro /bias_acc '
        f'--max-cache-size 104857600'
    )
    record_env = os.environ.copy()
    record_proc = subprocess.Popen(
        ['bash', '-c', record_cmd],
        env=record_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(2)

    # 启动 ESKF data_handle_node
    eskf_cmd = (
        f'{ROS2_SETUP} && '
        f'ros2 run my_nav2_robot data_handle_node '
        f'--ros-args -p use_sim_time:=true '
        f'--params-file /home/goose/ace_ass/src/my_nav2_robot/config/config.yaml'
    )
    eskf_env = os.environ.copy()
    eskf_proc = subprocess.Popen(
        ['bash', '-c', eskf_cmd],
        env=eskf_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(3)

    # 播放 bag
    play_cmd = f'{ROS2_SETUP} && ros2 bag play {bag_path} --clock -r 1.0'
    play_env = os.environ.copy()
    start = time.time()
    result = subprocess.run(['bash', '-c', play_cmd], env=play_env,
                            capture_output=True, text=True, timeout=120)
    elapsed = time.time() - start
    print(f"  bag 播放完成，耗时 {elapsed:.1f}s")

    time.sleep(2)
    record_proc.send_signal(signal.SIGINT)
    eskf_proc.send_signal(signal.SIGINT)
    time.sleep(1)
    record_proc.terminate()
    eskf_proc.terminate()
    record_proc.wait()
    eskf_proc.wait()
    return out_bag


def extract_odom(bag_path):
    """从录制的 bag 提取 /odom 的 position 和 heading 数组。"""
    raw_path = Path(bag_path)
    # 找 mcap 文件
    mcap_files = list(raw_path.glob('*.mcap'))
    if not mcap_files:
        # check metadata for actual filename
        import yaml
        meta = yaml.safe_load((raw_path / 'metadata.yaml').read_text())
        rel = meta['rosbag2_bagfile_information']['relative_file_paths'][0]
        mcap_path = raw_path / rel
    else:
        mcap_path = mcap_files[0]

    if not mcap_path.exists():
        print(f"  mcap 文件不存在: {mcap_path}")
        return None

    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    reader = SequentialReader()
    storage = StorageOptions(uri=str(raw_path), storage_id='mcap')
    converter = ConverterOptions(input_serialization_format='cdr',
                                  output_serialization_format='cdr')
    reader.open(storage, converter)

    import struct
    # nav_msgs/msg/Odometry CDR layout (cdr wrapper 4B):
    #   header(8B stamp + frame_id_string) + child_frame_id_string
    #   pose: position(3×float64) + orientation(4×float64)
    #   twist: linear(3×float64) + angular(3×float64)
    #   covariance[36] float64
    #
    # 简化：搜 position x,y,z 三连 float64
    # 已知 frame_id="odom", child_frame_id="base_footprint" 都是固定字符串

    positions = []; orientations = []; stamps = []
    while reader.has_next():
        topic, msg, stamp = reader.read_next()
        if topic != '/odom':
            continue

        # header: stamp(8B = int32+uint32) + frame_id(string)
        # 先找 frame_id 长度字段
        fid_len = struct.unpack_from('<I', msg, 12)[0]  # frame_id="odom" len=5
        # frame_id string: 5B + null + 2B pad = 8B
        # child_frame_id: after frame_id
        cid_off = 12 + 4 + ((fid_len + 1 + 3) & ~3)  # 4B len + aligned string
        cid_len = struct.unpack_from('<I', msg, cid_off)[0]
        cid_data_end = cid_off + 4 + ((cid_len + 1 + 3) & ~3)

        # position: 3×float64 = 24B
        pos = struct.unpack_from('<3d', msg, cid_data_end)
        # orientation: 4×float64 = 32B
        ori = struct.unpack_from('<4d', msg, cid_data_end + 24)

        positions.append(pos)
        orientations.append(ori)
        stamps.append(stamp)

    if not positions:
        print(f"  未找到 /odom 消息")
        return None

    pos = np.array(positions);  # (N, 3)
    ori = np.array(orientations)  # (N, 4) — xyzw quaternion

    # 从四元数提取 yaw
    # q = [x, y, z, w]
    x, y, z, w = ori[:, 0], ori[:, 1], ori[:, 2], ori[:, 3]
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    # 时间轴 - stamp is nanoseconds (int)
    if len(stamps) > 0 and isinstance(stamps[0], int):
        t = np.array([(s - stamps[0]) * 1e-9 for s in stamps])
    else:
        t = np.array([s[0] + s[1]*1e-9 for s in stamps])
        t = t - t[0]

    return {'t': t, 'x': pos[:, 0], 'y': pos[:, 1], 'z': pos[:, 2],
            'yaw': yaw, 'qx': x, 'qy': y, 'qz': z, 'qw': w, 'n': len(pos)}


def compare_results(r1, r2, label1, label2):
    """对比两个 odom 结果"""
    print(f"\n{'='*60}")
    print(f"  ODOM 对比: {label1} vs {label2}")
    print(f"{'='*60}")
    for name, r in [(label1, r1), (label2, r2)]:
        if r is None:
            continue
        total_dist = np.sqrt((r['x'][-1] - r['x'][0])**2 +
                            (r['y'][-1] - r['y'][0])**2)
        total_yaw = np.degrees(r['yaw'][-1] - r['yaw'][0])
        # 包裹到 [-180, 180]
        total_yaw = (total_yaw + 180) % 360 - 180
        max_yaw_drift = np.max(np.abs(r['yaw'] - r['yaw'][0]))
        print(f"  {name}: {r['n']} 帧, "
              f"终点位移={total_dist:.3f}m, "
              f"总航向变化={total_yaw:.1f}°, "
              f"最大航向偏差={np.degrees(max_yaw_drift):.1f}°")


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', choices=['nav_debug1', 'nav_debug2', 'both'],
                     default='both')
    ap.add_argument('--skip-playback', action='store_true',
                     help='跳过回放，只分析已有 odom bag')
    args = ap.parse_args()

    results = {}
    bags_to_run = ['nav_debug1', 'nav_debug2'] if args.bag == 'both' else [args.bag]

    for name in bags_to_run:
        out_name = f'{name}_odom'
        if not args.skip_playback:
            print(f"\n>>> 回放 {name} ...")
            run_playback(name, BAGS[name], out_name)
        r = extract_odom(OUT_DIR / out_name)
        results[name] = r
        if r is not None:
            np.savez(OUT_DIR / f'{out_name}.npz', **r)

    if args.bag == 'both' and results.get('nav_debug1') and results.get('nav_debug2'):
        compare_results(results['nav_debug1'], results['nav_debug2'],
                       'nav_debug1', 'nav_debug2')
