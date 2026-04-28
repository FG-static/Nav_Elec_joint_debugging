#!/bin/bash
# ============================================================
# Docker 入口脚本
# 用法:
#   docker run ... full      → 启动完整栈（串口+ESKF+Nav2）
#   docker run ... eskf      → 仅 ESKF + Nav2（需外部串口）
#   docker run ... shell     → 进入 bash 调试
# ============================================================

set -e

# 串口软链（宿主机 /dev/ttyACM0 → 容器内 /tmp/ttyACM0）
# 如果 /dev/ttyACM0 存在且 /tmp/ttyACM0 不存在，则创建软链
if [ -e /dev/ttyACM0 ] && [ ! -e /tmp/ttyACM0 ]; then
    ln -sf /dev/ttyACM0 /tmp/ttyACM0
    echo ">>> 串口设备已映射: /dev/ttyACM0 -> /tmp/ttyACM0"
elif [ -e /dev/ttyUSB0 ] && [ ! -e /tmp/ttyACM0 ]; then
    ln -sf /dev/ttyUSB0 /tmp/ttyACM0
    echo ">>> 串口设备已映射: /dev/ttyUSB0 -> /tmp/ttyACM0"
fi

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

MODE="${1:-full}"

case "$MODE" in
    full)
        echo ">>> 启动完整栈：串口驱动 + ESKF + Nav2"
        exec ros2 launch my_nav2_robot full.launch.py
        ;;
    eskf)
        echo ">>> 启动 ESKF + Nav2（不含串口驱动）"
        exec ros2 launch my_nav2_robot full_navigation.launch.py
        ;;
    serial)
        echo ">>> 仅启动串口驱动"
        exec ros2 launch rm_serial_driver serial_driver.launch.py
        ;;
    shell)
        exec bash
        ;;
    *)
        echo "用法: $0 {full|eskf|serial|shell}"
        exit 1
        ;;
esac
