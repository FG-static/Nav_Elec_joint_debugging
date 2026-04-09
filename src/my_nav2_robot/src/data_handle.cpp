#include "my_nav2_robot/data_handle.hpp"

namespace nav_data_handle {

    NavDataHandle::NavDataHandle() : rclcpp::Node("DataHandleNode"){

        gimbal_sub_ = this->create_subscription<rm_interfaces::msg::Gimbal>(
            "/tracker/gimbal", rclcpp::SensorDataQoS(),
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
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(* this);

        // 初始化ESKF变量
        p_.setZero();
        v_.setZero();
        b_a_.setZero();
        b_g_.setZero();
        q_.setIdentity();
        delta_x_.setZero();
        P_ = Eigen::Matrix<double, 15, 15>::Identity() * 0.01;
        Q_ = Eigen::Matrix<double, 15, 15>::Identity() * 0.005;
        R_ = Eigen::Matrix4d::Identity() * 0.005;
        R_tilt_ = Eigen::Matrix2d::Identity() * 0.005; // pitch/roll 零倾斜约束，适度信任

        RCLCPP_INFO(this->get_logger(), "ESKF Data Handle Node Initialized");
    }

    void NavDataHandle::gimbalCallBack(
        const rm_interfaces::msg::Gimbal::SharedPtr msg
    ) {

        // 初始化：记录第一帧 t_ms，等下一帧才能算 dt
        if (!initialized) {
            last_t_ms_ = msg->t_ms;
            initialized = true;
            return;
        }

        // 用 MCU 采样时间戳差分计算 dt（单位 s）
        // uint32_t 减法自然处理溢出回绕（约 49 天才绕一圈）
        double dt = static_cast<double>(msg->t_ms - last_t_ms_) / 1000.0;
        if (dt <= 0.0 || dt > 0.1) {
            // 跳帧或 t_ms 异常，更新记录但跳过本轮 ESKF
            last_t_ms_ = msg->t_ms;
            return;
        }

        // ESKF
        predict(msg, dt);
        observeWheel(msg);
        observeZeroTilt();
        injectAndReset();

        // visualizer（用 ROS 系统时间戳，保持 TF/Nav2 兼容性）
        publishOdometry(msg->header.stamp);

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

    void NavDataHandle::predict(const rm_interfaces::msg::Gimbal::SharedPtr msg, double dt) {

        // 补偿零偏
        Eigen::Vector3d acc_raw(
            msg->linear_acceleration.x, 
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );
        Eigen::Vector3d w_raw(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );
        Eigen::Vector3d acc = acc_raw - b_a_;
        Eigen::Vector3d w = w_raw - b_g_;

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

        // 协方差矩阵更新
        Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
        Eigen::Matrix3d R = q_.toRotationMatrix();
        
        Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        Fx.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * dt;
        Fx.block<3, 3>(3, 9) = -R * dt; 
        Fx.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew_symmetric(w * dt);
        Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        P_ = Fx * P_ * Fx.transpose() + Q_;
    }  

    void NavDataHandle::observeWheel(
        const rm_interfaces::msg::Gimbal::SharedPtr msg
    ) {

        // 解算底盘速度
        Eigen::Vector<double, 4> wheel_v(
            msg->wheel_velocity.x,
            msg->wheel_velocity.y,
            msg->wheel_velocity.z,
            msg->wheel_velocity.w
        );
        // 麦轮运动学（来自 imu_coordinate.md）：
        //   vx = r/4 * ( w_fl + w_fr + w_rl + w_rr)
        //   vy = r/4 * (-w_fl + w_fr + w_rl - w_rr)
        //   wz = r/(4*(lx+ly)) * (-w_fl + w_fr - w_rl + w_rr)
        // wheel_velocity 字段映射：x=fl, y=fr, z=rl, w=rr
        //
        // 实车机械参数：
        //   轮半径 r = 81.5mm = 0.0815m
        //   轮对角线距离 = 425mm → 正方形布局假设 lx=ly=425/(2√2)≈150.3mm
        //   lx + ly ≈ 300.5mm = 0.3005m（需与电控确认布局是否为正方形）
        constexpr double kWheel = 0.0815 / 4.0;            // r/4
        constexpr double kWz    = 0.0815 / (4.0 * 0.3005); // r/(4*(lx+ly))

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
        Eigen::Matrix3d R = q_.toRotationMatrix();
        Eigen::Matrix3d R_T = R.transpose();
        Eigen::Vector3d v_body = R_T * v_;  // 世界系速度 → 车体系速度

        // 观测预测值 h(x)
        Eigen::Vector4d h_x;
        h_x.head<3>() = v_body;
        // 角速度观测：陀螺仪 z 轴测量值（已补偿零偏）在 predict 中已被积分进 q_，
        // 这里用 ESKF 名义状态反推的车体 z 角速度作为 h 的第 4 维
        // h_w = [0, 0, 1]^T · (R^T * (q_ * w_body)) 简化为 w_body_z
        h_x(3) = msg->angular_velocity.z - b_g_.z();

        // 雅可比矩阵 H (4×15)
        Eigen::Matrix<double, 4, 15> H =
            Eigen::Matrix<double, 4, 15>::Zero();

        // 速度部分对 δv 的偏导：∂(R^T * v_)/∂δv = R^T
        H.block<3, 3>(0, 3) = R_T;

        // 速度部分对 δθ 的偏导：∂(R^T * v_)/∂δθ = [R^T * v_]× = [v_body]×
        Eigen::Matrix3d v_anti;
        v_anti <<        0, -v_body.z(),  v_body.y(),
                   v_body.z(),        0, -v_body.x(),
                  -v_body.y(),  v_body.x(),        0;
        H.block<3, 3>(0, 6) = v_anti;

        // 角速度部分对 δθ 的偏导：
        // h_w = gyro_z - (b_g_z + δb_g_z)
        // 当旋转有误差 δθ 时，R^T 会变化，但角速度观测是在车体系测量的，
        // δθ 对车体系角速度观测的影响可以忽略（二阶小量）
        // H(3, 6) ≈ 0

        // 角速度部分对 δb_g 的偏导：∂(gyro_z - b_g_z - δb_g_z)/∂δb_g = [0, 0, -1]
        H(3, 14) = -1.0;  // δb_g 的 z 分量在状态向量中的索引是 12+2=14

        // 卡尔曼增益 Kk (15×4)
        auto S = H * P_ * H.transpose() + R_;
        Eigen::Matrix<double, 15, 4> Kk =
            P_ * H.transpose() * S.inverse();

        // 更新误差状态 delta_x
        delta_x_ += Kk * (y - h_x);

        // 更新状态误差协方差矩阵（Joseph 形式）
        Eigen::Matrix<double, 15, 15> I15 =
            Eigen::Matrix<double, 15, 15>::Identity();
        P_ = (I15 - Kk * H) * P_ *
             (I15 - Kk * H).transpose() +
             Kk * R_ * Kk.transpose();
    }

    void NavDataHandle::observeZeroTilt()
    {
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

    void NavDataHandle::injectAndReset() {

        // 获取真实状态
        p_ += delta_x_.segment<3>(0);
        v_ += delta_x_.segment<3>(3);
        b_a_ += delta_x_.segment<3>(9);
        b_g_ += delta_x_.segment<3>(12);
        Eigen::Vector3d dtheta = delta_x_.segment<3>(6);
        if (dtheta.norm() > 1e-10) {

            Eigen::Quaterniond dq(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
            q_ = (q_ * dq).normalized();
        }

        // 重置
        delta_x_.setZero();
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
        if (path_.poses.size() > 5000) path_.poses.clear();
        path_.poses.push_back(ps);

        path_pub_->publish(path_);
    }
} // nav_data_handle

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_data_handle::NavDataHandle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}