#!/usr/bin/env python3
"""离线解析 rosbag mcap 文件，提取 Gimbal 传感器数据并分析。"""
import sys, struct, math
import numpy as np
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# CDR 布局: rosbag2 cdr wrapper(4B) + header(8B stamp + 4B frame_id_len + 4B null)
#           + t_ms(4B+4B pad) + angular_velocity(3×float64)
#           + linear_acceleration(3×float64) + wheel_velocity(4×float64) + log(string)
# float64 需8字节对齐，t_ms 后 4B padding
OFF_SEC         =  4          # int32
OFF_NSEC        =  8          # uint32
OFF_FRAMEID_LEN = 12          # uint32
OFF_T_MS        = 20          # uint32 (after frame_id null+pad)
OFF_ANG_VEL     = 28          # 3×float64 (gyro x,y,z)
OFF_LIN_ACC     = 52          # 3×float64 (acc x,y,z)
OFF_WHEEL_VEL   = 76          # 4×float64 (fl, fr, rl, rr)

KWHEEL = 0.0815 / (4.0 * 0.7071067811865476)
KWZ    = 0.0815 / (4.0 * 0.2125)


def parse_bag(bag_dir):
    reader = SequentialReader()
    storage = StorageOptions(uri=str(bag_dir), storage_id='mcap')
    converter = ConverterOptions(input_serialization_format='cdr',
                                  output_serialization_format='cdr')
    reader.open(storage, converter)

    # 流式收集全部帧
    data = {'t_sec': [], 't_ros': [], 't_ms': [],
            'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
            'acc_x': [], 'acc_y': [], 'acc_z': [],
            'w_fl': [], 'w_fr': [], 'w_rl': [], 'w_rr': []}

    while reader.has_next():
        topic, msg, stamp = reader.read_next()
        if topic != '/tracker/gimbal':
            continue

        sec = struct.unpack_from('<i', msg, OFF_SEC)[0]
        nsec = struct.unpack_from('<I', msg, OFF_NSEC)[0]
        t_ms = struct.unpack_from('<I', msg, OFF_T_MS)[0]
        ang = struct.unpack_from('<3d', msg, OFF_ANG_VEL)
        acc = struct.unpack_from('<3d', msg, OFF_LIN_ACC)
        whl = struct.unpack_from('<4d', msg, OFF_WHEEL_VEL)

        data['t_ros'].append(sec + nsec * 1e-9)
        data['t_ms'].append(t_ms)
        data['gyro_x'].append(ang[0]); data['gyro_y'].append(ang[1]); data['gyro_z'].append(ang[2])
        data['acc_x'].append(acc[0]); data['acc_y'].append(acc[1]); data['acc_z'].append(acc[2])
        data['w_fl'].append(whl[0]); data['w_fr'].append(whl[1])
        data['w_rl'].append(whl[2]); data['w_rr'].append(whl[3])

    for k in data:
        data[k] = np.array(data[k])

    data['t_rel'] = data['t_ros'] - data['t_ros'][0]
    return data


def compute_derived(data):
    """计算 dt, wz_wheel, heading 积分等"""
    t_ms = data['t_ms']
    dt = np.zeros(len(t_ms))
    for i in range(1, len(t_ms)):
        d = int(t_ms[i]) - int(t_ms[i-1])
        if d < 0:
            d += 2**32
        dt[i] = d / 1000.0
    dt[0] = dt[1] if dt[1] > 0 else 0.005
    dt = np.clip(dt, 0.001, 0.5)

    wz_wheel = KWZ * (-data['w_fl'] + data['w_fr'] - data['w_rl'] + data['w_rr'])
    vx = KWHEEL * (data['w_fl'] + data['w_fr'] + data['w_rl'] + data['w_rr'])
    vy = KWHEEL * (-data['w_fl'] + data['w_fr'] + data['w_rl'] - data['w_rr'])

    # 静态 gyro bias (前 300 帧均值)
    n_cal = min(300, len(dt) // 3)
    b_g_z_0 = np.mean(data['gyro_z'][:n_cal])
    wz_gyro_cal = data['gyro_z'] - b_g_z_0

    heading_gyro = np.cumsum(data['gyro_z'] * dt)
    heading_gyro_cal = np.cumsum(wz_gyro_cal * dt)
    heading_wheel = np.cumsum(wz_wheel * dt)

    return {
        'dt': dt, 'wz_wheel': wz_wheel, 'vx': vx, 'vy': vy,
        'wz_gyro_cal': wz_gyro_cal, 'b_g_z_0': b_g_z_0,
        'heading_gyro': heading_gyro,
        'heading_gyro_cal': heading_gyro_cal,
        'heading_wheel': heading_wheel,
    }


def analyze(data, deriv, label):
    t = data['t_rel']; dt = deriv['dt']
    wz_gyro = data['gyro_z']; wz_wheel = deriv['wz_wheel']
    wz_cal = deriv['wz_gyro_cal']; vx = deriv['vx']

    is_moving = np.abs(vx) > 0.1
    is_turning = (np.abs(wz_gyro) > 0.2) | (np.abs(wz_wheel) > 0.2)
    is_straight = is_moving & ~is_turning

    s_idx = np.where(is_straight)[0]
    t_idx = np.where(is_turning)[0]

    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"{'='*60}")
    print(f"  帧数: {len(t)},  时长: {t[-1]:.1f}s,  标称 dt: {np.median(dt)*1000:.1f}ms")
    print(f"  静态 gyro bias z: {deriv['b_g_z_0']:.6f} rad/s")
    print(f"  转弯帧: {len(t_idx)} ({100*len(t_idx)/len(t):.0f}%)")
    print(f"  直走帧: {len(s_idx)} ({100*len(s_idx)/len(t):.0f}%)")

    if len(s_idx) > 100:
        s_wz_g = wz_cal[s_idx]
        s_wz_w = wz_wheel[s_idx]
        s_dt   = dt[s_idx]
        print(f"  直走 gyro wz: RMS={np.sqrt(np.mean(s_wz_g**2)):.4f}, "
              f"mean={np.mean(s_wz_g):.6f} rad/s")
        print(f"  直走 wheel wz: RMS={np.sqrt(np.mean(s_wz_w**2)):.4f}, "
              f"mean={np.mean(s_wz_w):.6f} rad/s")
        gyro_drift = np.cumsum(s_wz_g * s_dt)
        wheel_drift = np.cumsum(s_wz_w * s_dt)
        print(f"  直走 gyro  最大漂移: {np.degrees(gyro_drift[-1]):.2f}°")
        print(f"  直走 wheel 最大漂移: {np.degrees(wheel_drift[-1]):.2f}°")

        # 找漂移最严重的长直走段
        min_frames = 200
        segs = []; seg_s = s_idx[0]
        for i in range(1, len(s_idx)):
            if s_idx[i] - s_idx[i-1] > 3:
                if s_idx[i-1] - seg_s > min_frames:
                    segs.append((seg_s, s_idx[i-1]))
                seg_s = s_idx[i]
        if s_idx[-1] - seg_s > min_frames:
            segs.append((seg_s, s_idx[-1]))
        print(f"  长直走段 (>1s): {len(segs)} 个")
        for j, (a, b) in enumerate(segs[:5]):
            seg_dur = np.sum(dt[a:b])
            seg_drift = np.sum(wz_cal[a:b] * dt[a:b])
            print(f"    段{j}: t={t[a]:.1f}-{t[b]:.1f}s dur={seg_dur:.1f}s "
                  f"漂移={np.degrees(seg_drift):.2f}°")

    if len(t_idx) > 100:
        # 识别转弯事件
        turn_events = []
        in_turn = False; turn_start = 0
        for i in range(len(t)):
            if is_turning[i] and not in_turn:
                in_turn = True; turn_start = i
            elif not is_turning[i] and in_turn:
                in_turn = False
                if i - turn_start > 20:
                    turn_events.append((turn_start, i))
        if in_turn:
            turn_events.append((turn_start, len(t)-1))

        print(f"  转弯事件: {len(turn_events)} 次")
        for j, (a, b) in enumerate(turn_events):
            peak_g = np.max(np.abs(wz_gyro[a:b]))
            peak_w = np.max(np.abs(wz_wheel[a:b]))
            d_gyro = np.sum(wz_gyro[a:b] * dt[a:b])
            d_wheel = np.sum(wz_wheel[a:b] * dt[a:b])
            print(f"    转弯{j}: t={t[a]:.1f}-{t[b]:.1f}s "
                  f"dur={np.sum(dt[a:b]):.1f}s "
                  f"peak(wz_g={peak_g:.2f}, wz_w={peak_w:.2f}) "
                  f"转角(gyro={np.degrees(d_gyro):.0f}°, wheel={np.degrees(d_wheel):.0f}°)")

    return deriv


if __name__ == '__main__':
    for bag_path in ['/home/goose/ace_ass/nav_debug1',
                      '/home/goose/ace_ass/nav_debug2']:
        p = Path(bag_path)
        if not p.exists():
            print(f"跳过: {bag_path} (不存在)")
            continue
        data = parse_bag(bag_path)
        deriv = compute_derived(data)
        analyze(data, deriv, p.name)
        npz_path = f'/home/goose/ace_ass/analysis/{p.name}.npz'
        np.savez(npz_path, **data, **deriv)
        print(f"  → 已保存 {npz_path}")
