# ============================================================
# RoboMaster 导航系统 Docker 镜像
# 包含: ROS2 Jazzy + Nav2 + ESKF + 串口驱动 + Sophus
# ============================================================

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ── 系统依赖 ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # 构建工具
    build-essential cmake git wget python3-pip python3-colcon-common-extensions \
    # ROS2 串口
    ros-jazzy-serial-driver \
    # Nav2 全套（导航 + 行为树 + MPPI）
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-common \
    # TF / URDF / 关节
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-visualization-msgs \
    # Eigen3（头文件）
    libeigen3-dev \
    # fmt（Sophus 依赖）
    libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Sophus (Lie 群库，源码构建) ───────────────────────────
RUN cd /tmp && \
    git clone --depth 1 --branch v1.22.0 \
        https://github.com/strasdat/Sophus.git && \
    cd Sophus && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SOPHUS_TESTS=OFF && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/Sophus

# ── 工作空间 ──────────────────────────────────────────────
ENV ROS_WS=/ros2_ws
WORKDIR ${ROS_WS}

# 先复制依赖描述文件（利用 Docker 层缓存）
COPY src/rm_interfaces/ ${ROS_WS}/src/rm_interfaces/
COPY src/serial_driver/   ${ROS_WS}/src/serial_driver/
COPY src/my_nav2_robot/   ${ROS_WS}/src/my_nav2_robot/

# 移除 serial_driver 中未使用的 auto_aim_interfaces 依赖
RUN sed -i '/auto_aim_interfaces/d' \
    ${ROS_WS}/src/serial_driver/package.xml

# ── 构建 ──────────────────────────────────────────────────
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-up-to my_nav2_robot && \
    rm -rf build/

# ── 入口 ──────────────────────────────────────────────────
# 复制启动脚本
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

# 环境变量
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["full"]
