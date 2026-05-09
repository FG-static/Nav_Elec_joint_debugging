// imu_adapter.cpp
// 将 sensor_msgs/Imu（/livox/imu）转换为 rm_interfaces/Gimbal（/tracker/gimbal）
// 轮速填零（无轮速传感器），ESKF 仅靠 IMU 积分

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rm_interfaces/msg/gimbal.hpp"

class ImuAdapter : public rclcpp::Node {

public:

    ImuAdapter() : Node("imu_adapter_node") {

        gimbal_pub_ = this->create_publisher<rm_interfaces::msg::Gimbal>(
            "/tracker/gimbal", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {

                // 用 ROS 时钟计算相对毫秒时间戳（避免 header.stamp 溢出 uint32）
                if (start_time_.nanoseconds() == 0) {
                    start_time_ = this->now();
                }
                uint32_t t_ms = static_cast<uint32_t>(
                    (this->now() - start_time_).nanoseconds() / 1000000);

                rm_interfaces::msg::Gimbal gimbal;

                // 头部时间戳
                gimbal.header = msg->header;

                // MCU 时间戳：ROS 相对时间（单调递增，无溢出）
                gimbal.t_ms = t_ms;

                // IMU 角速度 → Gimbal.angular_velocity
                gimbal.angular_velocity.x = msg->angular_velocity.x;
                gimbal.angular_velocity.y = msg->angular_velocity.y;
                gimbal.angular_velocity.z = msg->angular_velocity.z;

                // IMU 线加速度 → Gimbal.linear_acceleration
                // Livox MID360 输出 g 单位，ESKF 用 m/s²，需 ×9.8
                constexpr double G = 9.80665;
                gimbal.linear_acceleration.x = msg->linear_acceleration.x * G;
                gimbal.linear_acceleration.y = msg->linear_acceleration.y * G;
                gimbal.linear_acceleration.z = msg->linear_acceleration.z * G;

                // 轮速填零（无轮速传感器）
                gimbal.wheel_velocity.x = 0.0;  // fl
                gimbal.wheel_velocity.y = 0.0;  // fr
                gimbal.wheel_velocity.z = 0.0;  // rl
                gimbal.wheel_velocity.w = 0.0;  // rr

                gimbal_pub_->publish(gimbal);
            });

        RCLCPP_INFO(this->get_logger(),
            "IMU 适配节点已启动: /livox/imu → /tracker/gimbal（轮速填零）");
    }

private:

    rclcpp::Publisher<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Time start_time_;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuAdapter>());
    rclcpp::shutdown();
    return 0;
}
