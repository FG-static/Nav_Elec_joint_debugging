#ifndef MY_NAV2_ROBOT__DATA_HANDLE
#define MY_NAV2_ROBOT__DATA_HANDLE

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/gimbal.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <fstream>

namespace nav_data_handle {

    class NavDataHandle : public rclcpp::Node{

    public:

        NavDataHandle();
    private:

        // 从 ROS2 参数服务加载 ESKF 噪声参数
        void loadESKFParams();

        int test; // 通信测试

        Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d vec);

        // 回调函数
        void gimbalCallBack(const rm_interfaces::msg::Gimbal::SharedPtr msg);

        // ESKF
        void predict(const rm_interfaces::msg::Gimbal::SharedPtr msg, double dt);
        void observeWheel(const rm_interfaces::msg::Gimbal::SharedPtr msg);
        void observeZeroTilt();
        void injectAndReset();

        // 接收 发布
        void publishOdometry(const rclcpp::Time &stamp);
        rclcpp::Subscription<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_sub_;
        rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
        // 名义状态
        Eigen::Vector3d p_; // pose
        Eigen::Vector3d v_; // velocity
        Eigen::Quaterniond q_; // quaternione : Base->World 也可以表示小车角度
        Eigen::Vector3d b_a_; // 加速计零偏
        Eigen::Vector3d b_g_; // 陀螺仪零偏

        // 误差状态
        Eigen::Matrix<double, 15, 1> delta_x_; // 状态误差

        // 状态误差协方差矩阵
        Eigen::Matrix<double, 15, 15> P_;

        // 噪声矩阵
        Eigen::Matrix<double, 15, 15> Q_; // 过程噪声
        Eigen::Matrix4d R_; // 观测噪声 - observeWheel 观测量 vx, vy, vz, wz
        Eigen::Matrix2d R_tilt_; // 观测噪声 - observeZeroTilt 观测量 pitch, roll

        // help
        bool initialized = false;
        uint32_t last_t_ms_ = 0; // MCU 上一帧采样时间戳（ms），用于计算 dt
        nav_msgs::msg::Path path_;

        const Eigen::Vector3d G_VEC_{0, 0, -9.8}; // 重力加速度
    };
} // nav_data_handle

#endif // MY_NAV2_ROBOT__DATA_HANDLE