#include "my_nav2_robot/data_handle.hpp"

namespace nav_data_handle {

    NavDataHandle::NavDataHandle() : rclcpp::Node("DataHandleNode") {

        gimbal_sub_ = this->create_subscription<rm_interfaces::msg::Gimbal>(
            "/tracker/gimbal", rclcpp::QoS(1).best_effort(),
            [this](const rm_interfaces::msg::Gimbal::SharedPtr msg) {
                gimbalCallBack(msg);
            }
        );
        target_pub_ = this->create_publisher<rm_interfaces::msg::Target>(
            "/tracker/target", 10
        );
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/path", 10
        );
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", 10
        );
        wz_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/wz", 10
        );
        odom_raw_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom_raw", 10
        );
        path_raw_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/path_raw", 10
        );
        odom_raw_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_raw", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odomRawCallback(msg);
            }
        );
        bias_acc_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/bias_acc", 10
        );
        bias_gyro_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/bias_gyro", 10
        );
        acc_compensated_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/acc_compensated", 10
        );
        gyro_compensated_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/gyro_compensated", 10
        );
        wheel_vel_raw_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/wheel_vel/raw", 10
        );
        wheel_vel_filtered_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/wheel_vel/filtered", 10
        );
        aligned_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/gicp/aligned_cloud", 20
        );
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(* this);

        // 点云订阅：独立回调组，GICP 在此线程执行，不阻塞 IMU 200Hz 回调
        auto lidar_cb_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions lidar_opts;
        lidar_opts.callback_group = lidar_cb_group;
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar/pointcloud", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                lidarCallback(msg);
            }, lidar_opts
        );

        // 初始化ESKF变量
        p_.setZero();
        v_.setZero();
        b_a_.setZero();
        b_g_.setZero();
        q_.setIdentity();
        delta_x_.setZero();

        // 初始化原始位姿变量
        p_raw_.setZero();
        v_raw_.setZero();
        q_raw_.setIdentity();

        // IMU 外参标定矩阵（IMU 系 → 车体系），初始化为单位阵
        // 实际值将在零偏标定阶段根据重力方向自动计算（仅 pitch/roll，忽略 yaw）
        R_imu_to_body_.setIdentity();
        R_lidar_to_body_.setIdentity();

        // 从配置文件加载 P、Q、R、R_tilt 参数
        loadESKFParams();

        RCLCPP_INFO(this->get_logger(), "ESKF Data Handle Node Initialized");
    }

    void NavDataHandle::loadESKFParams() {

        // 声明
        this->declare_parameter("eskf.P_init", 0.01);
        this->declare_parameter("eskf.Q_init", 0.005);
        this->declare_parameter("eskf.q_pose", 0.005);
        this->declare_parameter("eskf.q_theta", 0.0001);
        this->declare_parameter("eskf.q_b_a", 0.001);
        this->declare_parameter("eskf.q_b_g", 0.001);
        this->declare_parameter("eskf.r_11", 0.005);
        this->declare_parameter("eskf.r_22", 0.005);
        this->declare_parameter("eskf.r_33", 0.005);
        this->declare_parameter("eskf.r_44", 0.005);
        this->declare_parameter("eskf.r_tilt_11", 0.005);
        this->declare_parameter("eskf.r_tilt_22", 0.005);
        this->declare_parameter("eskf.calibration_duration", 1.5);
        this->declare_parameter("eskf.alpha_lowpass",      0.3); // 加速度计+轮速，重滤波
        this->declare_parameter("eskf.alpha_lowpass_gyro", 0.05);  // 陀螺仪，轻滤波
        this->declare_parameter("eskf.iter_max", 3);
        this->declare_parameter("eskf.eps_dx", 0.0001);

        // 读取
        double P_init     = this->get_parameter("eskf.P_init").as_double();
        double Q_init     = this->get_parameter("eskf.Q_init").as_double();
        double q_pose     = this->get_parameter("eskf.q_pose").as_double();
        double q_theta    = this->get_parameter("eskf.q_theta").as_double();
        double q_b_a      = this->get_parameter("eskf.q_b_a").as_double();
        double q_b_g      = this->get_parameter("eskf.q_b_g").as_double();
        double r_11       = this->get_parameter("eskf.r_11").as_double();
        double r_22       = this->get_parameter("eskf.r_22").as_double();
        double r_33       = this->get_parameter("eskf.r_33").as_double();
        double r_44       = this->get_parameter("eskf.r_44").as_double();
        double r_tilt_11  = this->get_parameter("eskf.r_tilt_11").as_double();
        double r_tilt_22  = this->get_parameter("eskf.r_tilt_22").as_double();
        calibration_duration_  = this->get_parameter("eskf.calibration_duration").as_double();
        alpha_lowpass_         = this->get_parameter("eskf.alpha_lowpass").as_double();
        alpha_lowpass_gyro_    = this->get_parameter("eskf.alpha_lowpass_gyro").as_double();
        iter_max_              = this->get_parameter("eskf.iter_max").as_int();
        eps_dx_                = this->get_parameter("eskf.eps_dx").as_double();

        // 参数有效性检查
        if (P_init <= 0.0 || Q_init <= 0.0 || q_b_a <= 0.0 || q_b_g <= 0.0 ||
            r_11 <= 0.0 || r_22 <= 0.0 ||
            r_33 <= 0.0 || r_44 <= 0.0 ||
            r_tilt_11 <= 0.0 || r_tilt_22 <= 0.0) {

            RCLCPP_ERROR(
                this->get_logger(),
                "ESKF 噪声参数必须为正数！收到 P=%.6f, Q=%.6f, q_b_a=%.6f, q_b_g=%.6f, "
                "r_11=%.6f, r_22=%.6f, r_33=%.6f, r_44=%.6f, "
                "r_tilt_11=%.6f, r_tilt_22=%.6f，将使用默认值",
                P_init, Q_init, q_b_a, q_b_g,
                r_11, r_22, r_33, r_44,
                r_tilt_11, r_tilt_22);
            P_init     = 0.01;
            Q_init     = 0.005;
            q_b_a = q_b_g = 0.001;
            r_11 = r_22 = r_33 = r_44 = 0.005;
            r_tilt_11 = r_tilt_22 = 0.005;
        }
        P_      = Eigen::Matrix<double, 15, 15>::Identity() * P_init;
        Q_      = Eigen::Matrix<double, 15, 15>::Identity() * Q_init;
        // 位置部分独立噪声（δp: 0-2）
        Q_.block<3, 3>(0,  0) = Eigen::Matrix3d::Identity() * q_pose;
        // δθ 部分独立噪声（6-8），压制 v_anti heading 随机游走
        Q_.block<3, 3>(6,  6) = Eigen::Matrix3d::Identity() * q_theta;
        // 零偏部分使用独立噪声量（b_a: 9-11, b_g: 12-14）
        Q_.block<3, 3>(9,  9) = Eigen::Matrix3d::Identity() * q_b_a;
        Q_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * q_b_g;
        R_      = Eigen::Matrix4d::Zero();
        R_(0, 0) = r_11;
        R_(1, 1) = r_22;
        R_(2, 2) = r_33;
        R_(3, 3) = r_44;
        R_tilt_ = Eigen::Matrix2d::Zero();
        R_tilt_(0, 0) = r_tilt_11;
        R_tilt_(1, 1) = r_tilt_22;

        // 点云 ICP 观测参数
        this->declare_parameter("lidar.icp_leaf_size", 0.1);
        this->declare_parameter("lidar.icp_fitness_threshold", 0.5);
        this->declare_parameter("lidar.r_lidar_vx", 0.05);
        this->declare_parameter("lidar.r_lidar_vy", 0.05);
        this->declare_parameter("lidar.r_lidar_vz", 0.05);
        this->declare_parameter("lidar.r_lidar_wz", 0.02);
        this->declare_parameter("lidar.publish_aligned_cloud", false);
        this->declare_parameter("lidar.max_points_before_gicp", 0);
        voxel_leaf_size_       = this->get_parameter("lidar.icp_leaf_size").as_double();
        icp_fitness_threshold_ = this->get_parameter("lidar.icp_fitness_threshold").as_double();
        publish_aligned_cloud_ = this->get_parameter("lidar.publish_aligned_cloud").as_bool();
        max_points_before_gicp_ = this->get_parameter("lidar.max_points_before_gicp").as_int();
        if (max_points_before_gicp_ < 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "lidar.max_points_before_gicp=%d 无效，已按 0（不限制）处理",
                max_points_before_gicp_);
            max_points_before_gicp_ = 0;
        }
        R_lidar_ = Eigen::Matrix<double, 4, 4>::Zero();
        R_lidar_(0, 0) = this->get_parameter("lidar.r_lidar_vx").as_double();
        R_lidar_(1, 1) = this->get_parameter("lidar.r_lidar_vy").as_double();
        R_lidar_(2, 2) = this->get_parameter("lidar.r_lidar_vz").as_double();
        R_lidar_(3, 3) = this->get_parameter("lidar.r_lidar_wz").as_double();

        // 雷达系 → 车体系旋转（欧拉角 ZYX）
        this->declare_parameter("lidar.lidar_to_body_roll", 0.0);
        this->declare_parameter("lidar.lidar_to_body_pitch", 0.0);
        this->declare_parameter("lidar.lidar_to_body_yaw", 3.14159265);
        double lb_roll  = this->get_parameter("lidar.lidar_to_body_roll").as_double();
        double lb_pitch = this->get_parameter("lidar.lidar_to_body_pitch").as_double();
        double lb_yaw   = this->get_parameter("lidar.lidar_to_body_yaw").as_double();
        R_lidar_to_body_ = (
            Eigen::AngleAxisd(lb_yaw,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(lb_pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(lb_roll,  Eigen::Vector3d::UnitX())
        ).toRotationMatrix();
        RCLCPP_INFO(this->get_logger(),
            "R_lidar_to_body: yaw=%.2f pitch=%.2f roll=%.2f",
            lb_yaw, lb_pitch, lb_roll);

        RCLCPP_INFO(
            this->get_logger(),
            "ESKF 参数已加载: P=%.6f, Q=%.6f, Q_bias=[%.8f %.8f], "
            "R=[%.6f %.6f %.6f %.6f], R_tilt=[%.6f %.6f], calibration=%.2fs",
            P_init, Q_init, q_b_a, q_b_g,
            r_11, r_22, r_33, r_44,
            r_tilt_11, r_tilt_22, calibration_duration_);
    }

    void NavDataHandle::gimbalCallBack(
        const rm_interfaces::msg::Gimbal::SharedPtr msg
    ) {

        // ========== 零偏标定阶段 ==========
        if (calib_state_ == CalibState::CALIBRATING) {

            // 记录标定起始帧时间戳
            if (calib_count_ == 0) {
                calib_start_t_ms_ = msg->t_ms;
            }

            // 累积 IMU 原始数据
            calib_acc_sum_ += Eigen::Vector3d(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z
            );
            calib_gyro_sum_ += Eigen::Vector3d(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z
            );
            calib_count_++;

            // 检查标定时长是否已到
            double elapsed = static_cast<double>(msg->t_ms - calib_start_t_ms_) / 1000.0;
            if (elapsed >= calibration_duration_ && calib_count_ > 0) {

                // 计算均值
                Eigen::Vector3d mean_acc  = calib_acc_sum_  / calib_count_;
                Eigen::Vector3d mean_gyro = calib_gyro_sum_ / calib_count_;

                // ====== 从重力方向自动计算 R_imu_to_body_（Rodrigues 旋转法） ======
                // 与 imu_static_level_calibrate.cpp 方法一致
                // 静止时 mean_acc ≈ R_imu_to_body_^T * [0,0,+g]（反作用力沿体系 Z 轴向上）
                // 归一化 mean_acc 得到 IMU 帧下的反作用力方向 s
                // 目标方向 t = [0,0,±1]，使得 R * s = t（旋转后反作用力沿 Z 轴）
                // 用 Rodrigues 旋转公式构造最小旋转 R：source→target
                // yaw 不可观测，Rodrigues 法自然只修正 pitch/roll 分量
                double acc_norm = mean_acc.norm();
                double acc_near_gravity = std::abs(acc_norm - 9.8);
                if (acc_norm > 1.0 && acc_near_gravity < 2.0) {

                    Eigen::Vector3d s = mean_acc / acc_norm;  // source：IMU帧反作用力方向
                    Eigen::Vector3d t(0.0, 0.0, s.z() >= 0.0 ? 1.0 : -1.0);  // target：体系Z轴

                    // 旋转轴 v = s × t
                    Eigen::Vector3d v = s.cross(t);
                    double sin_theta = v.norm();
                    double cos_theta = s.dot(t);

                    if (sin_theta < 1e-9) {
                        // 已对齐，使用单位阵
                        R_imu_to_body_ = Eigen::Matrix3d::Identity();
                    } else {
                        // 反对称矩阵 K
                        Eigen::Matrix3d K;
                        K <<      0, -v.z(),  v.y(),
                                v.z(),      0, -v.x(),
                            -v.y(),  v.x(),      0;

                        // Rodrigues: R = I + K + ((1-cos)/sin²) * K²
                        double gain = (1.0 - cos_theta) / (sin_theta * sin_theta);
                        R_imu_to_body_ = Eigen::Matrix3d::Identity() + K + gain * K * K;
                    }

                    RCLCPP_INFO(
                        this->get_logger(),
                        "R_imu_to_body 自动计算完成(Rodrigues): cos=%.6f, sin=%.6f",
                        cos_theta, sin_theta);
                } else {

                    RCLCPP_WARN(
                        this->get_logger(),
                        "mean_acc 范数异常 (%.2f)，R_imu_to_body 保持单位阵",
                        acc_norm);
                }

                // 零偏设定：
                // 陀螺仪零偏 = 静止时陀螺仪输出的均值
                // 加速度计零偏 = 静止时原始帧测量均值 - 原始帧下重力反作用力
                // 静止时 acc_raw = R_imu_to_body_^T * [0,0,+g] + b_a_raw
                // b_a_raw = mean_acc - R_imu_to_body_^T * [0,0,+g]
                //         = mean_acc + R_imu_to_body_^T * G_VEC_
                // 注意：G_VEC_ 是体坐标系重力向量，需先旋转到原始 IMU 帧再与 mean_acc 相减
                b_g_ = mean_gyro;
                b_a_ = mean_acc + R_imu_to_body_.transpose() * G_VEC_;

                calib_state_ = CalibState::RUNNING;
                last_t_ms_ = msg->t_ms; // 初始化 dt 起始帧

                // 用标定结束帧的原始值初始化低通滤波器（避免第一帧阶跃）
                gyro_filtered_  = Eigen::Vector3d(
                    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                acc_filtered_   = Eigen::Vector3d(
                    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                wheel_filtered_ = Eigen::Vector4d(
                    msg->wheel_velocity.x, msg->wheel_velocity.y,
                    msg->wheel_velocity.z, msg->wheel_velocity.w);

                RCLCPP_INFO(
                    this->get_logger(),
                    "零偏标定完成（%d 帧，%.2fs）: b_a=[%.4f, %.4f, %.4f], b_g=[%.6f, %.6f, %.6f]",
                    calib_count_, elapsed,
                    b_a_.x(), b_a_.y(), b_a_.z(),
                    b_g_.x(), b_g_.y(), b_g_.z());
            }
            return; // 标定期间不执行 ESKF
        }

        // ========== 正常运行阶段 ==========
        // 初始化：记录第一帧 t_ms，等下一帧才能算 dt
        if (last_t_ms_ == 0) {

            last_t_ms_ = msg->t_ms;
            return;
        }

        // 用 MCU 采样时间戳差分计算 dt（单位 s）
        // uint32_t 减法自然处理溢出回绕（约 49 天才绕一圈）
        double dt = static_cast<double>(msg->t_ms - last_t_ms_) / 1000.0;
        if (dt <= 0.0 || dt >= 0.7) {

            // t_ms 异常（通常为第一帧或回绕），跳过本轮 ESKF
            last_t_ms_ = msg->t_ms;
            RCLCPP_WARN(this->get_logger(), "dt=%.4fs 异常，跳过解算", dt);
            return;
        }
        if (dt > 0.5) {

            RCLCPP_WARN(
                this->get_logger(),
                "dt=%.4fs 严重偏大，ESKF 积分精度下降，请检查串口丢帧", dt);
        } else if (dt > 0.05) {
            
            RCLCPP_WARN(this->get_logger(), "dt=%.4fs 偏大，仍执行ESKF", dt);
        }

        // 低通滤波（dt 自适应）
        // 从配置的 alpha（针对标称 5ms 帧间隔）换算时间常数 tau
        //   alpha = 1 - exp(-dt_nom / tau)  →  tau = -dt_nom / log(1 - alpha)
        // 然后对实际 dt 计算自适应 alpha：alpha_adapt = 1 - exp(-dt / tau)
        // 这样大 dt 时滤波减弱（更贴近原始数据），避免滞后放大 innovation
        constexpr double DT_NOM = 0.005;  // 标称帧间隔 5ms（200Hz）

        // 缓存原始未滤波数据（供 publishRawOdometry 使用）
        acc_raw_   = Eigen::Vector3d(
                         msg->linear_acceleration.x,
                         msg->linear_acceleration.y,
                         msg->linear_acceleration.z);
        gyro_raw_  = Eigen::Vector3d(
                         msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);
        wheel_raw_ = Eigen::Vector4d(
                         msg->wheel_velocity.x, msg->wheel_velocity.y,
                         msg->wheel_velocity.z, msg->wheel_velocity.w);

        // 计算自适应 alpha（仅在 alpha != 1 时，1 表示不过滤）
        // 大 dt 时 alpha 会暴涨，导致噪声穿通，必须加上限
        double alpha_gyro = alpha_lowpass_gyro_;
        double alpha_wheel = alpha_lowpass_;
        if (alpha_lowpass_gyro_ < 1.0) {
            double tau_gyro = -DT_NOM / std::log(1.0 - alpha_lowpass_gyro_);
            alpha_gyro = std::min(1.0 - std::exp(-dt / tau_gyro), 0.2);
        }
        if (alpha_lowpass_ < 1.0) {
            double tau_wheel = -DT_NOM / std::log(1.0 - alpha_lowpass_);
            alpha_wheel = std::min(1.0 - std::exp(-dt / tau_wheel), 0.5);
        }
        //alpha_gyro = alpha_lowpass_gyro_;
        //alpha_wheel = alpha_lowpass_;

        gyro_filtered_ = alpha_gyro * Eigen::Vector3d(
                             msg->angular_velocity.x,
                             msg->angular_velocity.y,
                             msg->angular_velocity.z)
                       + (1.0 - alpha_gyro) * gyro_filtered_;

        acc_filtered_  = alpha_wheel * Eigen::Vector3d(
                             msg->linear_acceleration.x,
                             msg->linear_acceleration.y,
                             msg->linear_acceleration.z)
                       + (1.0 - alpha_wheel) * acc_filtered_;

        wheel_filtered_ = alpha_wheel * Eigen::Vector4d(
                              msg->wheel_velocity.x, msg->wheel_velocity.y,
                              msg->wheel_velocity.z, msg->wheel_velocity.w)
                        + (1.0 - alpha_wheel) * wheel_filtered_;

        // 发布滤波前后的轮速解算速度 (vx, vy)，x 分量 = vx，y 分量 = vy
        constexpr double kWv = 0.0815 / (4.0 * 0.7071067811865476);
        {
            auto pub_vec = [&](const Eigen::Vector4d &wv, auto &pub) {
                geometry_msgs::msg::Vector3 msg;
                msg.x = kWv * ( wv[0] + wv[1] + wv[2] + wv[3]);
                msg.y = kWv * (-wv[0] + wv[1] + wv[2] - wv[3]);
                msg.z = 0.0;
                pub->publish(msg);
            };
            pub_vec(wheel_raw_, wheel_vel_raw_pub_);
            pub_vec(wheel_filtered_, wheel_vel_filtered_pub_);
        }

        // ESKF predict — 大 dt 时拆成多个子步，防止 P_ 协方差膨胀导致 Kk 暴增
        constexpr double DT_MAX = 0.005;  // 子步最大 20ms
        int n_steps = std::max(1, static_cast<int>(std::ceil(dt / DT_MAX)));
        double dt_sub = dt / n_steps;
        last_dt_ = dt_sub;  // 供 injectAndReset 反算融合 wz
        for (int i = 0; i < n_steps; i ++) predict(dt_sub);
        // IESKF 迭代观测更新
        iteratedObserve(dt_sub);

        // 发布补偿后的 IMU 数据（IMU 系，已扣除零偏）
        geometry_msgs::msg::Vector3 acc_comp_msg, gyro_comp_msg;
        Eigen::Vector3d acc_comp = acc_filtered_ - b_a_;
        Eigen::Vector3d gyro_comp = gyro_filtered_ - b_g_;
        acc_comp_msg.x = acc_comp.x(); acc_comp_msg.y = acc_comp.y(); acc_comp_msg.z = acc_comp.z();
        gyro_comp_msg.x = gyro_comp.x(); gyro_comp_msg.y = gyro_comp.y(); gyro_comp_msg.z = gyro_comp.z();
        acc_compensated_pub_->publish(acc_comp_msg);
        gyro_compensated_pub_->publish(gyro_comp_msg);

        // visualizer（用 ROS 系统时间戳，保持 TF/Nav2 兼容性）
        publishOdometry(msg->header.stamp);
        publishRawOdometry(msg->header.stamp);

        // update
        last_t_ms_ = msg->t_ms;
    }

    Eigen::Matrix3d NavDataHandle::skew_symmetric(
        const Eigen::Vector3d vec
    ) {

        Eigen::Matrix3d anti;
        anti <<        0, -vec.z(),  vec.y(),
                 vec.z(),        0, -vec.x(),
                -vec.y(),  vec.x(),        0;
        return anti;
    }

    // SO3 左雅可比（A_matrix），IESKF 核心
    // A(v) = I + (1-cos||v||)/||v||² · [v]× + (||v||-sin||v||)/||v||³ · [v]×²
    // 小角度（||v|| < 1e-6）时用泰勒展开避免除零：A ≈ I - ½[v]× + (1/6)[v]×²
    Eigen::Matrix3d NavDataHandle::A_matrix(const Eigen::Vector3d &v) {

        double nv = v.norm();
        Eigen::Matrix3d vx = skew_symmetric(v);
        if (nv < 1e-6) {
            
            return Eigen::Matrix3d::Identity() - 0.5 * vx + (1.0 / 6.0) * vx * vx;
        }
        double nv2 = nv * nv;
        return Eigen::Matrix3d::Identity()
               + (1.0 - std::cos(nv)) / nv2 * vx
               + (nv - std::sin(nv)) / (nv2 * nv) * vx * vx;
    }

    void NavDataHandle::predict(double dt) {

        // 补偿零偏（使用低通滤波后的数据），得到 IMU 系下的干净测量
        Eigen::Vector3d acc_imu = acc_filtered_  - b_a_;
        Eigen::Vector3d w_imu   = gyro_filtered_ - b_g_;

        // 预处理旋转：IMU 系 → 车体系（使用离线标定的外参矩阵）
        Eigen::Vector3d acc = R_imu_to_body_ * acc_imu;
        Eigen::Vector3d w   = R_imu_to_body_ * w_imu;

        // Gimbal->Base 坐标系转换 - 自动处理四元数运算，无需乘两次
        // 可惜没有云台
        // tf2::Quaternion q;
        // q.setRPY(0, msg->pitch, msg->yaw);
        // Eigen::Quaterniond Eq;
        // tf2::convert(q, Eq);
        // acc = Eq.inverse() * acc;
        // w = Eq.inverse() * w;

        // 名义状态积分
        p_ = p_ + v_ * dt + 0.5 * (q_ * acc + G_VEC_) * dt * dt;
        v_ = v_ + (q_ * acc + G_VEC_) * dt;
        Eigen::Vector3d dtheta = w * dt;
        if (dtheta.norm() > 1e-10) {

            q_ = q_ * Eigen::Quaterniond(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized())); // 归一化
        }

        // 协方差矩阵更新（IESKF：用 A_matrix 替代一阶近似）
        Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
        Eigen::Matrix3d R = q_.toRotationMatrix();
        Eigen::Matrix3d A = A_matrix(dtheta);  // SO3 左雅可比，精确传递 δθ

        Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        Fx.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * A * dt;   // A 修正 δθ 列
        Fx.block<3, 3>(3, 9) = -R * R_imu_to_body_ * dt;            // b_a_ 在 IMU 帧，Euclidean 不变
        Fx.block<3, 3>(6, 6) = A;                                     // A_matrix 替代 I - skew(w*dt)
        Fx.block<3, 3>(6, 12) = -A * R_imu_to_body_ * dt;            // A 修正 δθ 行

        P_ = Fx * P_ * Fx.transpose() + Q_;
    }  

    void NavDataHandle::observeWheel() {

        // 解算底盘速度（使用低通滤波后的轮速数据）
        Eigen::Vector<double, 4> wheel_v = wheel_filtered_;

        // 轮速全为零时跳过观测（无轮速传感器 / rosbag 回放场景）
        if (wheel_v.squaredNorm() < 1e-12) {
            return;
        }
        // 全向轮运动学（正交布局，辊子与底盘xy轴呈45°）：
        //   vx = r/(4*cos45°) * ( w_fl + w_fr + w_rl + w_rr)
        //   vy = r/(4*cos45°) * (-w_fl + w_fr + w_rl - w_rr)
        //   wz = r/(4*L)      * (-w_fl + w_fr - w_rl + w_rr)
        // wheel_velocity 字段映射：x=fl, y=fr, z=rl, w=rr
        //
        // 实车机械参数：
        //   轮半径 r = 81.5mm = 0.0815m
        //   全向轮正交布局：辊子与底盘径向轴呈45°，投影系数 cos45°
        //   L = 0.2125m（轮心到车体中心距离在旋转方向的投影）
        constexpr double kWheel = 0.0815 / (4.0 * 0.7071067811865476); // r/(4*cos45°)，全向轮正交布局
        constexpr double kWz    = 0.0815 / (4.0 * 0.2125);             // r/(4*L)，L=0.2125m

        // 观测量：车体系速度(vx, vy, vz=0) + 车体系角速度(wz)，共 4 维
        Eigen::Vector4d y;
        y(0) = kWheel * ( wheel_v[0] + wheel_v[1] + wheel_v[2] + wheel_v[3]);  // vx
        y(1) = kWheel * (-wheel_v[0] + wheel_v[1] + wheel_v[2] - wheel_v[3]);  // vy
        y(2) = 0.0;                                                               // vz=0 零速观测
        y(3) = kWz   * (-wheel_v[0] + wheel_v[1] - wheel_v[2] + wheel_v[3]);    // wz

        // 观测模型 h(x)：
        //   速度部分（前 3 维）：h_v = R^T * v_，将世界系速度转到车体系
        //   角速度部分（第 4 维）：h_w = R^T * [0,0,wz_imu]^T 的 z 分量
        //     但我们直接用陀螺仪测量的 gyro_z 作为 h_w 的基线更好
        //     这里用更直接的方式：h_w = (R^T * q_omega)_z = (R^T * R * w_body)_z = w_body_z
        //     简化后 h_w = gyro_z - b_g_z（已补偿零偏的陀螺仪 z 轴输出）

        observeVelocity(y, R_);

        // 发布wz: x=轮速wz, y=IMU gyro wz, z=ESKF卡尔曼融合wz
        Eigen::Vector3d w_imu  = gyro_filtered_ - b_g_;
        Eigen::Vector3d w_body = R_imu_to_body_ * w_imu;
        geometry_msgs::msg::Vector3 wz_msg;
        wz_msg.x = y(3);
        wz_msg.y = w_body(2);
        wz_msg.z = fused_wz_;
        wz_pub_->publish(wz_msg);
    }

    void NavDataHandle::observeVelocity(
        const Eigen::Vector4d &y_obs, const Eigen::Matrix<double, 4, 4> &R_obs
    ) {

        // 4 维观测模型：y = [vx, vy, vz, wz]（车体系）
        // h(x) = [R^T * v_world,  w_body_z]
        Eigen::Matrix3d R = q_.toRotationMatrix();
        Eigen::Matrix3d R_T = R.transpose();
        Eigen::Vector3d v_body = R_T * v_;  // 世界系速度 → 车体系速度

        // 观测预测值 h(x)
        Eigen::Vector4d h_x;
        h_x.head<3>() = v_body;
        // 角速度观测：使用预处理旋转后的车体系 z 轴角速度
        // h_w = [R_imu_to_body * (gyro_filtered_ - b_g_)]_z
        Eigen::Vector3d w_body_clean = R_imu_to_body_ * (gyro_filtered_ - b_g_);
        h_x(3) = w_body_clean.z();

        // 雅可比矩阵 H (4×15)
        Eigen::Matrix<double, 4, 15> H =
            Eigen::Matrix<double, 4, 15>::Zero();

        // 速度部分对 δv 的偏导：∂(R^T * v_)/∂δv = R^T
        H.block<3, 3>(0, 3) = R_T;

        // 速度部分对 δθ 的偏导：∂(R^T * v_)/∂δθ = [v_body]×
        Eigen::Matrix3d v_anti = skew_symmetric(v_body);
        H.block<3, 3>(0, 6) = v_anti;

        // 角速度部分对 δθ 的偏导：
        // h_w = gyro_z - (b_g_z + δb_g_z)
        // 当旋转有误差 δθ 时，R^T 会变化，但角速度观测是在车体系测量的，
        // δθ 对车体系角速度观测的影响可以忽略（二阶小量）
        // H(3, 6) ≈ 0

        // 角速度部分对 δb_g 的偏导：
        // h_w = [R_imu_to_body_ * (gyro_filtered_ - b_g_)]_z
        //      = R_imu_to_body_.row(2) * (gyro_filtered_ - b_g_)
        // ∂h_w/∂δb_g = -R_imu_to_body_.row(2)（精确值，含 x/y 交叉项）
        H.block<1, 3>(3, 12) = -R_imu_to_body_.row(2);

        // 卡尔曼增益 Kk (15×4)
        auto S = H * P_ * H.transpose() + R_obs;
        Eigen::Matrix<double, 15, 4> Kk =
            P_ * H.transpose() * S.inverse();

        // 更新误差状态
        delta_x_ += Kk * (y_obs - h_x);

        // Joseph 形式更新协方差
        Eigen::Matrix<double, 15, 15> I15 =
            Eigen::Matrix<double, 15, 15>::Identity();
        P_ = (I15 - Kk * H) * P_ *
             (I15 - Kk * H).transpose() +
             Kk * R_obs * Kk.transpose();
    }

    void NavDataHandle::observeZeroTilt() {

        // 地面机器人约束：pitch ≈ 0, roll ≈ 0
        // 从当前四元数提取 pitch 和 roll 作为 "观测值"，
        // 目标值是 0，通过 ESKF 观测更新将角度拉回水平面。
        //
        // R = R_z(yaw) * R_y(-pitch) * R_x(roll)
        // 提取方法：
        //   pitch = -asin(R(2,0))   （R 的第 3 行第 1 列）
        //   roll  = atan2(R(2,1), R(2,2))

        Eigen::Matrix3d R = q_.toRotationMatrix();
        double pitch = -std::asin(std::clamp(R(2, 0), -1.0, 1.0));
        double roll  =  std::atan2(R(2, 1), R(2, 2));

        // ESKF 观测约定：y = 传感器观测, h_x = 名义状态预测
        // 零倾斜约束："传感器说"倾斜为 0，名义状态的 pitch/roll 是预测值
        Eigen::Vector2d y = Eigen::Vector2d::Zero();   // 约束：地面水平，pitch=0, roll=0
        Eigen::Vector2d h_x(pitch, roll);               // 名义状态当前的 pitch/roll

        // 雅可比矩阵 H (2×15)
        // body frame 右乘扰动：q_true = q_nominal * δq, R_true = R * (I + [δθ]×)
        // 对 level 机器人（pitch≈0, roll≈0）：
        //   R_true(2,0) = R(2,0) - R(2,2)*δθ_y → pitch ≈ δθ_y
        //   R_true(2,1) = R(2,1) + R(2,2)*δθ_x → roll ≈ δθ_x
        // 关键：body frame 扰动下 Jacobian 不依赖 yaw
        Eigen::Matrix<double, 2, 15> H =
            Eigen::Matrix<double, 2, 15>::Zero();
        H(0, 6) =  0.0;  // ∂pitch/∂δθ_x
        H(0, 7) =  1.0;  // ∂pitch/∂δθ_y （R(2,2)/cos(pitch) ≈ +1）
        H(0, 8) =  0.0;  // ∂pitch/∂δθ_z
        H(1, 6) =  1.0;  // ∂roll/∂δθ_x
        H(1, 7) =  0.0;  // ∂roll/∂δθ_y
        H(1, 8) =  0.0;  // ∂roll/∂δθ_z

        // 卡尔曼增益 Kk (15×2)
        auto S = H * P_ * H.transpose() + R_tilt_;
        Eigen::Matrix<double, 15, 2> Kk =
            P_ * H.transpose() * S.inverse();

        // 更新误差状态
        delta_x_ += Kk * (y - h_x);

        // 更新协方差矩阵（Joseph 形式）
        Eigen::Matrix<double, 15, 15> I15 =
            Eigen::Matrix<double, 15, 15>::Identity();
        P_ = (I15 - Kk * H) * P_ *
             (I15 - Kk * H).transpose() +
             Kk * R_tilt_ * Kk.transpose();
    }

    void NavDataHandle::constrainYawRate(double dt) {

        // 直线行驶时软约束：yaw rate ≈ 0
        // 当陀螺仪和轮速一致认为 |wz| 很小时，将当前帧的航向变化约束为零，
        // 抑制电机振动导致的 wz 震荡积分漂移

        // 计算两种 wz 源
        Eigen::Vector3d w_body = R_imu_to_body_ * (gyro_filtered_ - b_g_);
        double wz_gyro = w_body.z();

        constexpr double kWz = 0.0815 / (4.0 * 0.2125);
        double wz_wheel = kWz * (-wheel_filtered_[0] + wheel_filtered_[1]
                                 - wheel_filtered_[2] + wheel_filtered_[3]);

        // 激活条件：两传感器一致确认 |wz| < 阈值，且车辆有前进速度
        constexpr double kWzThresh = 0.15;  // rad/s，低于此值视为「直线」
        if (std::abs(wz_gyro) > kWzThresh || std::abs(wz_wheel) > kWzThresh) return;
        if (v_.norm() < 0.1) return;  // 静止时不约束，避免干扰零速状态

        // 观测：y = 0（零航向变化），h_x = wz_gyro * dt（本帧预测航向变化）
        double y = 0.0;
        double h_x = wz_gyro * dt;

        // 雅可比 H (1×15)
        // ∂(wz_gyro*dt)/∂δθ = 0（角速度测量不依赖姿态误差）
        // ∂(wz_gyro*dt)/∂δb_g = -R_imu_to_body_.row(2) * dt
        Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
        H.block<1, 3>(0, 12) = -R_imu_to_body_.row(2) * dt;

        // 大噪声 → 软约束（只作偏置慢修正，不完全压制真实小幅度转动）
        constexpr double R_yaw = 0.2;
        double S_val = (H * P_ * H.transpose())(0, 0) + R_yaw;
        Eigen::Matrix<double, 15, 1> Kk =
            P_ * H.transpose() * (1.0 / S_val);

        delta_x_ += Kk * (y - h_x);

        // Joseph 协方差更新
        Eigen::Matrix<double, 15, 15> I15 =
            Eigen::Matrix<double, 15, 15>::Identity();
        P_ = (I15 - Kk * H) * P_ *
             (I15 - Kk * H).transpose() +
             Kk * R_yaw * Kk.transpose();
    }

    void NavDataHandle::injectAndReset() {

        // 保存 ESKF 融合角速度（IMU predict + 所有观测的最优估计）
        // fused_wz = w_imu + δθ_z / dt，即 predict 基础 + 卡尔曼修正
        Eigen::Vector3d w_imu  = R_imu_to_body_ * (gyro_filtered_ - b_g_);
        Eigen::Vector3d dtheta = delta_x_.segment<3>(6);
        fused_wz_ = w_imu.z() + dtheta.z() / last_dt_;

        // 获取真实状态
        p_ += delta_x_.segment<3>(0);
        v_ += delta_x_.segment<3>(3);
        b_a_ += delta_x_.segment<3>(9);
        b_g_ += delta_x_.segment<3>(12);
        if (dtheta.norm() > 1e-10) {

            Eigen::Quaterniond dq(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
            q_ = (q_ * dq).normalized();
        }

        // 重置
        delta_x_.setZero();

        // 实时发布零偏数据
        geometry_msgs::msg::Vector3 ba_msg, bg_msg;
        ba_msg.x = b_a_.x(); ba_msg.y = b_a_.y(); ba_msg.z = b_a_.z();
        bg_msg.x = b_g_.x(); bg_msg.y = b_g_.y(); bg_msg.z = b_g_.z();
        bias_acc_pub_->publish(ba_msg);
        bias_gyro_pub_->publish(bg_msg);
    }

    // IESKF 迭代观测更新：跨迭代累积 δx，末次一次性注入
    // 每轮：重置 P/δx → 观测在当前状态上重线性化 → 累积 δx → 手动更新状态供下轮重线性化
    // 循环结束后：一次性 injectAndReset + P 旋转
    void NavDataHandle::iteratedObserve(double dt) {

        // 保存 predict 后的完整名义状态
        p_prop_   = p_;
        v_prop_   = v_;
        q_prop_   = q_;
        b_a_prop_ = b_a_;
        b_g_prop_ = b_g_;
        P_prop_   = P_;

        // 消费 lidarCallback 异步计算的 ICP 结果
        Eigen::Vector4d y_lidar = Eigen::Vector4d::Zero();
        Eigen::Matrix<double, 4, 4> R_lidar_scaled = R_lidar_;
        bool use_lidar = false;
        {
            std::lock_guard<std::mutex> lock(icp_result_mtx_);
            if (icp_result_ready_) {
                y_lidar = icp_y_lidar_;
                R_lidar_scaled = icp_R_lidar_;
                use_lidar = true;
                icp_result_ready_ = false;
            }
        }

        Eigen::Matrix<double, 15, 1> dx_accum =
            Eigen::Matrix<double, 15, 1>::Zero();

        int actual_iters = 0;
        for (int iter = 0; iter < iter_max_; iter ++) {

            actual_iters ++;

            // 重置 P 和 δx
            P_ = P_prop_;
            delta_x_.setZero();

            // 观测函数用当前 q_/v_（x_iter）计算 H、h(x)，结果累加到 δx
            observeWheel();
            if (use_lidar) {

                observeVelocity(y_lidar, R_lidar_scaled);
            }
            observeZeroTilt();

            // 累积本轮 δx
            dx_accum += delta_x_;

            // 收敛检查
            if (delta_x_.norm() < eps_dx_) break;

            // 手动更新状态供下一轮重线性化（不调 injectAndReset，避免重复注入和清零）
            p_   = p_prop_   + dx_accum.segment<3>(0);
            v_   = v_prop_   + dx_accum.segment<3>(3);
            b_a_ = b_a_prop_ + dx_accum.segment<3>(9);
            b_g_ = b_g_prop_ + dx_accum.segment<3>(12);
            Eigen::Vector3d dr = dx_accum.segment<3>(6);
            double nr = dr.norm();
            if (nr > 1e-10) {
                
                q_ = (q_prop_ * Eigen::Quaterniond(
                    Eigen::AngleAxisd(nr, dr.normalized()))).normalized();
            }
        }

        // fused_wz（状态已在循环末次迭代注入，无需重复）
        Eigen::Vector3d w_imu = R_imu_to_body_ * (gyro_filtered_ - b_g_);
        fused_wz_ = w_imu.z() + dx_accum(8) / last_dt_;

        // 发布零偏
        delta_x_.setZero();
        geometry_msgs::msg::Vector3 ba_msg, bg_msg;
        ba_msg.x = b_a_.x(); ba_msg.y = b_a_.y(); ba_msg.z = b_a_.z();
        bg_msg.x = b_g_.x(); bg_msg.y = b_g_.y(); bg_msg.z = b_g_.z();
        bias_acc_pub_->publish(ba_msg);
        bias_gyro_pub_->publish(bg_msg);

        // 旋转 P 到新切空间
        Eigen::Quaterniond qe = q_prop_.conjugate() * q_;
        if (qe.w() < 0.0) qe.coeffs() = -qe.coeffs();
        Eigen::Vector3d qev(qe.x(), qe.y(), qe.z());
        double ne = qev.norm();
        Eigen::Vector3d dte;
        if (ne < 1e-10) dte = 2.0 * qev;
        else dte = 2.0 * std::atan2(ne, qe.w()) / ne * qev;
        if (std::isfinite(dte.norm())) {
            
            Eigen::Matrix3d Ae = A_matrix(dte);
            Eigen::Matrix<double, 15, 15> Af =
                Eigen::Matrix<double, 15, 15>::Identity();
            Af.block<3, 3>(6, 6) = Ae;
            P_ = Af.transpose() * P_ * Af;
        }
    }

    void NavDataHandle::publishOdometry(const rclcpp::Time &stamp) {

        geometry_msgs::msg::TransformStamped tfs;
        tfs.header.stamp = stamp;
        tfs.header.frame_id = "odom";
        tfs.child_frame_id = "base_footprint";

        tfs.transform.translation.x = p_.x();
        tfs.transform.translation.y = p_.y();
        tfs.transform.translation.z = p_.z();

        tfs.transform.rotation.x = q_.x();
        tfs.transform.rotation.y = q_.y();
        tfs.transform.rotation.z = q_.z();
        tfs.transform.rotation.w = q_.w();

        tf_broadcaster_->sendTransform(tfs);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint"; // 必须与 TF child frame 和 nav2_params robot_base_frame 一致

        odom.pose.pose.position.x = p_.x();
        odom.pose.pose.position.y = p_.y();
        odom.pose.pose.position.z = p_.z();
        odom.pose.pose.orientation.x = q_.x();
        odom.pose.pose.orientation.y = q_.y();
        odom.pose.pose.orientation.z = q_.z();
        odom.pose.pose.orientation.w = q_.w();
        
        odom_pub_->publish(odom);

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.pose = odom.pose.pose;
        
        path_.header.frame_id = "odom"; // frame_id 必须设置，否则 RViz 不显示且 Nav2 报警
        path_.header.stamp = stamp;
        //if (path_.poses.size() > 5000) path_.poses.clear();
        path_.poses.push_back(ps);

        path_pub_->publish(path_);
    }

    void NavDataHandle::publishRawOdometry(const rclcpp::Time &stamp) {

        // 使用原始未滤波数据做简单积分（仅补偿零偏，无ESKF校正）
        // 零偏在标定完成后才有意义，标定期间不发布
        if (calib_state_ != CalibState::RUNNING) return;

        double dt = static_cast<double>(last_t_ms_ - last_t_ms_raw_) / 1000.0;
        if (last_t_ms_raw_ == 0 || dt <= 0.0) {
            last_t_ms_raw_ = last_t_ms_;
            return;
        }

        // 原始数据（未低通滤波）：使用 gimbalCallBack 中的 msg 原始值
        // 此处直接使用上一次回调中保存的原始 IMU 测量
        // 注意：acc_raw_ / gyro_raw_ / wheel_raw_ 在 gimbalCallBack 中赋值
        Eigen::Vector3d acc_imu_raw = acc_raw_ - b_a_;
        Eigen::Vector3d w_imu_raw   = gyro_raw_ - b_g_;

        // IMU系 → 车体系
        Eigen::Vector3d acc_raw = R_imu_to_body_ * acc_imu_raw;
        Eigen::Vector3d w_raw   = R_imu_to_body_ * w_imu_raw;

        // 简单前向欧拉积分
        p_raw_ = p_raw_ + v_raw_ * dt + 0.5 * (q_raw_ * acc_raw + G_VEC_) * dt * dt;
        v_raw_ = v_raw_ + (q_raw_ * acc_raw + G_VEC_) * dt;
        Eigen::Vector3d dtheta_raw = w_raw * dt;
        if (dtheta_raw.norm() > 1e-10) {
            q_raw_ = q_raw_ * Eigen::Quaterniond(
                Eigen::AngleAxisd(dtheta_raw.norm(), dtheta_raw.normalized()));
        }

        // 发布原始里程计
        nav_msgs::msg::Odometry odom_raw;
        odom_raw.header.stamp = stamp;
        odom_raw.header.frame_id = "odom";
        odom_raw.child_frame_id = "base_footprint";

        odom_raw.pose.pose.position.x = p_.x();
        odom_raw.pose.pose.position.y = p_.y();
        odom_raw.pose.pose.position.z = p_.z();
        odom_raw.pose.pose.orientation.x = q_raw_.x();
        odom_raw.pose.pose.orientation.y = q_raw_.y();
        odom_raw.pose.pose.orientation.z = q_raw_.z();
        odom_raw.pose.pose.orientation.w = q_raw_.w();

        odom_raw_pub_->publish(odom_raw);
        last_t_ms_raw_ = last_t_ms_;
    }

    void NavDataHandle::odomRawCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg
    ) {
        
        // 订阅 /odom_raw，实时同步发布小车运动轨迹到 /path_raw
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg->header;
        ps.pose = msg->pose.pose;

        path_raw_.header.frame_id = msg->header.frame_id;
        path_raw_.header.stamp = msg->header.stamp;
        path_raw_.poses.push_back(ps);

        path_raw_pub_->publish(path_raw_);
    }

    Eigen::Matrix4d NavDataHandle::estimate_motion_with_gicp(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
        double &alignment_score,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned_cloud
    ) {

        // 体素降采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.setInputCloud(target_cloud);
        voxel_filter.filter(*filtered_target);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.setInputCloud(source_cloud);
        voxel_filter.filter(*filtered_source);

        // 移除无效点
        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_target(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_source(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : filtered_target->points) {
            if (pcl::isFinite(p)) clean_target->points.push_back(p);
        }
        for (const auto& p : filtered_source->points) {
            if (pcl::isFinite(p)) clean_source->points.push_back(p);
        }

        // 点数不足时返回单位阵
        if (clean_target->size() < 10 || clean_source->size() < 10) {
            alignment_score = 1e9;
            return Eigen::Matrix4d::Identity();
        }

        // GICP 配准
        small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;
        reg.setNumThreads(4);
        reg.setCorrespondenceRandomness(20);
        reg.setMaxCorrespondenceDistance(1.0);
        reg.setRegistrationType("GICP");

        reg.setInputTarget(clean_target);
        reg.setInputSource(clean_source);

        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        Eigen::Matrix4f init_guess = gicp_init_guess_;
        reg.align(*aligned, init_guess);

        alignment_score = reg.getFitnessScore();
        aligned_cloud = aligned;

        // 返回 source → target 的变换矩阵（4×4）
        Eigen::Matrix4d T = reg.getFinalTransformation().cast<double>();
        return T;
    }

    void NavDataHandle::lidarCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg
    ) {

        // 防积压：上次 GICP 还没完成，丢弃当前帧
        if (gicp_running_.exchange(true)) {
            return;
        }

        // 将 ROS 点云转为 PCL 格式，移除 NaN/Inf
        auto raw = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *raw);
        auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*raw, *cloud, indices);

        static int cb_cnt = 0;
        if (++cb_cnt <= 5 || cb_cnt % 100 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "LIDAR CB #%d: raw=%zu, after_NaN=%zu",
                cb_cnt, raw->size(), cloud->size());
        }

        // 使用点云消息时间戳计算帧间隔，避免 GICP 执行耗时/回放调度抖动污染速度观测
        int64_t lidar_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (lidar_stamp_ns <= 0) {
            lidar_stamp_ns = this->now().nanoseconds();
        }
        int64_t wall_stamp_ns = this->now().nanoseconds();
        double wall_callback_dt = 0.0;
        if (last_lidar_wall_ns_ > 0) {
            wall_callback_dt = static_cast<double>(wall_stamp_ns - last_lidar_wall_ns_) * 1e-9;
        }

        auto cloud_for_gicp = cloud;
        if (max_points_before_gicp_ > 0 &&
            cloud->size() > static_cast<size_t>(max_points_before_gicp_)) {

            auto limited_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            limited_cloud->reserve(max_points_before_gicp_);
            double stride = static_cast<double>(cloud->size()) /
                            static_cast<double>(max_points_before_gicp_);
            for (int i = 0; i < max_points_before_gicp_; i ++) {
                size_t idx = static_cast<size_t>(i * stride);
                if (idx >= cloud->size()) idx = cloud->size() - 1;
                limited_cloud->points.push_back(cloud->points[idx]);
            }
            limited_cloud->width = static_cast<uint32_t>(limited_cloud->points.size());
            limited_cloud->height = 1;
            limited_cloud->is_dense = cloud->is_dense;
            cloud_for_gicp = limited_cloud;
        }

        // 高角速度时重置 GICP 初始猜测为单位阵，避免上一帧错误配准传播
        if (gyro_filtered_.norm() > 1.5) {
            //gicp_init_guess_ = Eigen::Matrix4f::Identity();
        }

        // GICP 在此线程执行（不阻塞 IMU 200Hz 回调）
        if (prev_cloud_ && prev_cloud_->size() > 0 && cloud_for_gicp->size() > 0) {

            double score = 0.0;
            // 观测gicp匹配用时
            int64_t gicp_start_ns = this->now().nanoseconds();
            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;
            Eigen::Matrix4d T = estimate_motion_with_gicp(
                prev_cloud_, cloud_for_gicp, score, aligned_cloud);
            double gicp_cost_ms =
                static_cast<double>(this->now().nanoseconds() - gicp_start_ns) * 1e-6;
            gicp_init_guess_ = T.cast<float>();

            // 发布 GICP 对齐后的点云
            if (publish_aligned_cloud_ && aligned_cloud && aligned_cloud->size() > 0) {
                sensor_msgs::msg::PointCloud2 aligned_msg;
                pcl::toROSMsg(*aligned_cloud, aligned_msg);
                aligned_msg.header = msg->header;
                aligned_cloud_pub_->publish(aligned_msg);
            }

            // RCLCPP_WARN(
            //     this->get_logger(), "GICP算法耗时 dt = %.2f", this->now() - t1
            // );

            double lidar_dt =
                static_cast<double>(lidar_stamp_ns - last_lidar_stamp_ns_) * 1e-9;
            int skipped_scan_count = 0;
            if (lidar_dt > 0.2) {
                skipped_scan_count = std::max(0, static_cast<int>(std::round(lidar_dt / 0.1)) - 1);
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "ICP RESULT: score=%.4f thresh=%.4f dt=%.3f wall_dt=%.3f "
                "gicp=%.1fms skipped=%d pts=%zu/%zu",
                score, icp_fitness_threshold_, lidar_dt, wall_callback_dt,
                gicp_cost_ms, skipped_scan_count, cloud_for_gicp->size(), cloud->size());

            if (skipped_scan_count > 0) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "GICP DROPPED SCANS: header_dt=%.3f wall_dt=%.3f "
                    "gicp=%.1fms skipped=%d",
                    lidar_dt, wall_callback_dt, gicp_cost_ms, skipped_scan_count);
            }

            if (score < icp_fitness_threshold_ &&
                lidar_dt > 0.01 && lidar_dt < 1.0) {

                Eigen::Vector3d t = T.block<3, 1>(0, 3);
                Eigen::Matrix3d R_icp = T.block<3, 3>(0, 0);
                Eigen::Vector3d v_icp = t / lidar_dt;

                Eigen::AngleAxisd aa(R_icp);
                Eigen::Vector3d w_icp = aa.angle() * aa.axis() / lidar_dt;

                // 雷达系 → 车体系
                v_icp = R_lidar_to_body_ * v_icp;
                w_icp = R_lidar_to_body_ * w_icp;

                // 合理性检查
                double v_xy = v_icp.head<2>().norm();
                if (v_xy > 2.0 || std::abs(w_icp.z()) > 2.0) {

                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 1000,
                        "ICP REJECTED: |v_xy|=%.3f wz=%.3f", v_xy, w_icp.z());
                } else {

                    Eigen::Vector4d y;
                    y(0) = v_icp.x();
                    y(1) = v_icp.y();
                    y(2) = 0.0;
                    y(3) = w_icp.z();

                    // 自适应 R_lidar：快速旋转 + 差配准 → 增大 R 降低 ICP 信任
                    double gyro_norm = gyro_filtered_.norm();
                    double scale_gyro = 1.0 + 50.0 * gyro_norm * gyro_norm;
                    double scale_fitness = 1.0 + 5.0 * (score / icp_fitness_threshold_);
                    double scale_total = scale_gyro * scale_fitness;

                    std::lock_guard<std::mutex> lock(icp_result_mtx_);
                    icp_y_lidar_ = y;
                    icp_R_lidar_ = R_lidar_;// * scale_total;
                    icp_result_ready_ = true;
                }
            }
        }

        prev_cloud_ = cloud_for_gicp;
        last_lidar_stamp_ns_ = lidar_stamp_ns;
        last_lidar_wall_ns_ = wall_stamp_ns;

        gicp_running_ = false;  // GICP 处理完成，允许接收下一帧
    }

} // nav_data_handle

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_data_handle::NavDataHandle>();
    // 多线程执行器：lidarCallback 与 gimbalCallBack 并行，GICP 不阻塞 IMU
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
