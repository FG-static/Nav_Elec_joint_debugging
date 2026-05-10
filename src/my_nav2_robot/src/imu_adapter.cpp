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
            "/tracker/gimbal", rclcpp::QoS(1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {

                rm_interfaces::msg::Gimbal gimbal;

                // 头部时间戳
                gimbal.header = msg->header;

                // MCU 时间戳：使用 IMU 采样时间的相对毫秒，避免 rosbag 回放调度影响 dt
                gimbal.t_ms = make_t_ms(msg);

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

    uint32_t make_t_ms(const sensor_msgs::msg::Imu::SharedPtr &msg)
    {

        const bool has_valid_header_stamp =
            msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;

        rclcpp::Time current_stamp = has_valid_header_stamp ?
            rclcpp::Time(msg->header.stamp) : this->now();

        if (!has_valid_header_stamp) {

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "/livox/imu header.stamp 无效，退回使用节点当前时间生成 Gimbal.t_ms");
        }

        if (!start_stamp_initialized_) {

            start_stamp_ = current_stamp;
            start_stamp_initialized_ = true;
            last_t_ms_ = 0;
            return last_t_ms_;
        }

        // 只使用原始纳秒差，避免 header.stamp 与节点 clock_type 不一致时抛异常
        const int64_t elapsed_ns = current_stamp.nanoseconds() - start_stamp_.nanoseconds();
        if (elapsed_ns < 0) {

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "/livox/imu header.stamp 早于起始时间，保持上一帧 Gimbal.t_ms=%u",
                last_t_ms_);
            return last_t_ms_;
        }

        const uint32_t t_ms = static_cast<uint32_t>(elapsed_ns / 1000000);
        if (t_ms < last_t_ms_) {

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "/livox/imu header.stamp 倒退，保持上一帧 Gimbal.t_ms=%u",
                last_t_ms_);
            return last_t_ms_;
        }

        last_t_ms_ = t_ms;
        return last_t_ms_;
    }

    rclcpp::Publisher<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Time start_stamp_;
    uint32_t last_t_ms_ = 0;
    bool start_stamp_initialized_ = false;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuAdapter>());
    rclcpp::shutdown();
    return 0;
}
