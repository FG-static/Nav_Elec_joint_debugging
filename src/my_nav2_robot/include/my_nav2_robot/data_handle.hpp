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
#include <mutex>
#include <atomic>
#include <deque>
#include <limits>
#include <vector>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <pcl-1.14/pcl/point_cloud.h> // 点云基础类型
#include <pcl-1.14/pcl/point_types.h> // 点类型定义
#include <pcl_conversions/pcl_conversions.h> // ros2和pcl消息互转
#include <pcl-1.14/pcl/filters/voxel_grid.h> // 滤波器
#include <pcl-1.14/pcl/common/transforms.h> // 点云坐标变换

#include <unordered_map>

// FLANN (used by small_gicp) expects a serialization implementation for
// std::unordered_map when saving/loading LSH indices. The system FLANN
// version shipped with Ubuntu does not provide this specialization, so we
// provide it here to avoid compilation errors.
namespace flann {
namespace serialization {

// Forward-declare the primary template so we can provide a specialization
// before the full FLANN serialization headers are included.
template<typename T>
struct Serializer;

template<typename K, typename V>
struct Serializer<std::unordered_map<K, V>>
{
  template<typename InputArchive>
  static inline void load(InputArchive &ar, std::unordered_map<K, V> &map_val)
  {
    size_t size;
    ar & size;
    for (size_t i = 0; i < size; ++i) {
      K key;
      V value;
      ar & key;
      ar & value;
      map_val.emplace(std::move(key), std::move(value));
    }
  }

  template<typename OutputArchive>
  static inline void save(OutputArchive &ar, const std::unordered_map<K, V> &map_val)
  {
    ar & map_val.size();
    for (const auto &kv : map_val) {
      ar & kv.first;
      ar & kv.second;
    }
  }
};

}  // namespace serialization
}  // namespace flann

#include <small_gicp/pcl/pcl_registration.hpp> // pcl结合使用
#include <small_gicp/registration/registration.hpp> // 核心注册算法
#include <small_gicp/factors/gicp_factor.hpp>

namespace nav_data_handle {

    class NavDataHandle : public rclcpp::Node{

    public:

        NavDataHandle();
    private:

        // 从 ROS2 参数服务加载 ESKF 噪声参数
        void loadESKFParams();

        int test; // 通信测试

        Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d vec);
        Eigen::Matrix3d A_matrix(const Eigen::Vector3d &v);  // SO3 左雅可比（IESKF）

        // 回调函数
        void gimbalCallBack(const rm_interfaces::msg::Gimbal::SharedPtr msg);

        // ESKF
        void predict(double dt);
        void observeWheel();
        void observeZeroTilt();
        void constrainYawRate(double dt);  // 直线行驶时软约束 yaw rate ≈ 0，抗振动漂移
        void injectAndReset();
        void iteratedObserve(double dt);   // IESKF 迭代观测更新（合并所有观测）

        // 接收 发布
        void publishOdometry(const rclcpp::Time &stamp);
        void publishRawOdometry(const rclcpp::Time &stamp);
        void publishWzDebug();
        void odomRawCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        rclcpp::Subscription<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_raw_sub_;
        rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wz_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_raw_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr bias_acc_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr bias_gyro_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr acc_compensated_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gyro_compensated_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wheel_vel_raw_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wheel_vel_filtered_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gicp_vel_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gicp_innovation_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gyro_z_debug_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gicp_yaw_debug_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gicp_status_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gicp_map_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
        // 名义状态
        Eigen::Vector3d p_; // pose
        Eigen::Vector3d v_; // velocity
        Eigen::Quaterniond q_; // quaternione : Base->World 也可以表示小车角度
        Eigen::Vector3d b_a_; // 加速计零偏
        Eigen::Vector3d b_g_; // 陀螺仪零偏

        // IMU 外参预处理旋转矩阵（离线标定，IMU 系 → 车体系）
        Eigen::Matrix3d R_imu_to_body_;

        // IESKF 迭代更新：保存 predict 后的名义状态/协方差
        Eigen::Vector3d p_prop_, v_prop_, b_a_prop_, b_g_prop_;  // predict 后的名义状态
        Eigen::Quaterniond q_prop_;                               // predict 后的四元数
        Eigen::Matrix<double, 15, 15> P_prop_;                    // predict 后的协方差
        int iter_max_ = 3;                                        // 最大迭代次数
        double eps_dx_ = 1e-4;                                    // 收敛阈值（弧度）

        // 误差状态
        Eigen::Matrix<double, 15, 1> delta_x_; // 状态误差

        // ESKF 融合角速度（IMU gyro + 轮速 wz + v_anti，卡尔曼最优加权）
        double fused_wz_ = 0.0;
        double last_dt_ = 0.005;

        // 状态误差协方差矩阵
        Eigen::Matrix<double, 15, 15> P_;

        // 噪声矩阵
        Eigen::Matrix<double, 15, 15> Q_; // 过程噪声
        Eigen::Matrix4d R_; // 观测噪声 - observeWheel 观测量 vx, vy, vz, wz
        Eigen::Matrix2d R_tilt_; // 观测噪声 - observeZeroTilt 观测量 pitch, roll

        // 点云 GICP 观测
        void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void observeVelocity(
            const Eigen::Vector4d &y_obs, const Eigen::Matrix<double, 4, 4> &R_obs);
        void observeVelocity(
            const Eigen::Vector4d &y_obs, const Eigen::Matrix<double, 4, 4> &R_obs,
            const Eigen::Vector3d &v_body_at_lidar);
        void observeYaw(
            double delta_yaw_icp, const Eigen::Quaterniond &q_lidar_ref, double R_yaw);
        void observeYawResidual(double yaw_innovation, double R_yaw);
        struct LidarFrame {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
            std::vector<int64_t> point_stamps;
            int64_t stamp_ns = 0;
            int64_t min_point_stamp_ns = 0;
            int64_t max_point_stamp_ns = 0;
            bool has_point_time = false; // 是否退化
            bool deskewed = false;
            size_t raw_points = 0;
        };

        struct StateSnapshot {
            int64_t stamp_ns = 0;
            Eigen::Vector3d p = Eigen::Vector3d::Zero();
            Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
            Eigen::Vector3d v = Eigen::Vector3d::Zero();
            Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
            Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
        };
        struct LocalSubmapFrame {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            int64_t stamp_ns = 0;
            Eigen::Matrix4d frame_to_submap = Eigen::Matrix4d::Identity();
        };
        struct ImuSample {
            int64_t stamp_ns = 0;
            Eigen::Vector3d acc_body = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyro_body = Eigen::Vector3d::Zero();
        };
        struct ScanPoseSample {
            int64_t stamp_ns = 0;
            Eigen::Vector3d p = Eigen::Vector3d::Zero();
            Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
            Eigen::Vector3d v = Eigen::Vector3d::Zero();
        };

        bool parseLidarFrame(
            const sensor_msgs::msg::PointCloud2::SharedPtr msg,
            LidarFrame &frame);

        void pushStateHistory(int64_t stamp_ns);
        bool filterCloudForGicp(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &output) const;
        bool interpolateState(
            int64_t stamp_ns, Eigen::Vector3d &p, Eigen::Quaterniond &q) const;
        bool findAnchorState(
            const int64_t stamp_ns,
            StateSnapshot &anchor) const;
        bool buildImuTrajectory(
            const StateSnapshot &anchor, 
            const int64_t t_start,
            const int64_t t_end,
            std::vector<ScanPoseSample> &traj) const;
        bool lookupScanPose(
            const std::vector<ScanPoseSample> &traj,
            const int64_t stamp_ns,
            Eigen::Vector3d &p,
            Eigen::Quaterniond &q) const;

        bool applyGicpTranslationDeskew(
            const LidarFrame &frame,
            const Eigen::Matrix4d &prev_to_current,
            int64_t current_stamp_ns,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &corrected_cloud);
        bool estimateLidarMotion(
            int64_t source_stamp_ns, int64_t target_stamp_ns,
            Eigen::Matrix4d &source_to_target) const;
        Eigen::Matrix4d bodyToLidarTransform(const Eigen::Matrix4d &body_tf) const;
        void resetGicpMap(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
            const rclcpp::Time &stamp);
        void resetGicpMapTransform();
        void publishGicpMapTransform(const rclcpp::Time &stamp);
        void updateGicpMap(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &current_cloud,
            const Eigen::Matrix4d &prev_to_current,
            const rclcpp::Time &stamp);
        void publishGicpMap(const rclcpp::Time &stamp);
        bool buildGicpLocalSubmap(
            pcl::PointCloud<pcl::PointXYZ>::Ptr &local_submap);
        void addGicpLocalSubmapFrame(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
            int64_t stamp_ns,
            const Eigen::Matrix4d &frame_to_submap);
        void clearGicpLocalSubmap();
        Eigen::Matrix4d estimate_motion_with_gicp(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
            double &alignment_score,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned_cloud
        );
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
        LidarFrame prev_lidar_frame_;
        std::atomic<bool> gicp_running_{false};  // 防积压：GICP 还在跑时跳过新帧

        // ICP 结果缓冲（lidarCallback 计算，iteratedObserve 消费）
        std::mutex icp_result_mtx_;
        bool icp_result_ready_ = false;
        bool icp_velocity_ready_ = false;
        Eigen::Vector4d icp_y_lidar_ = Eigen::Vector4d::Zero();
        double icp_delta_yaw_ = 0.0;
        double icp_yaw_innovation_ = 0.0;
        Eigen::Quaterniond icp_yaw_ref_q_ = Eigen::Quaterniond::Identity();
        bool icp_yaw_ready_ = false;

        int64_t last_lidar_stamp_ns_ = 0;
        int64_t last_lidar_wall_ns_ = 0;
        Eigen::Quaterniond prev_lidar_q_ = Eigen::Quaterniond::Identity();
        bool prev_lidar_q_ready_ = false;
        double voxel_leaf_size_ = 0.05;
        double icp_fitness_threshold_ = 0.5;
        bool publish_aligned_cloud_ = false;
        int max_points_before_gicp_ = 0;
        double r_lidar_yaw_delta_ = 0.05;
        double lidar_max_range_ = 80.0;
        double lidar_min_voxel_leaf_size_ = 0.05;
        double lidar_max_gicp_velocity_ = 2.5;
        double lidar_max_gicp_yaw_rate_ = 1.5;
        double lidar_max_gicp_yaw_delta_ = 0.12;
        double lidar_max_gicp_yaw_innovation_ = 0.025;
        double lidar_max_gicp_velocity_innovation_ = 0.75;
        bool enable_gicp_local_submap_ = true;
        int gicp_local_submap_max_frames_ = 10;
        int gicp_local_submap_min_frames_ = 2;
        double gicp_local_submap_leaf_size_ = 0.20;
        int gicp_local_submap_max_points_ = 80000;
        std::deque<LocalSubmapFrame> gicp_local_submap_frames_;
        struct GicpDeskewAnchor {
            bool ready = false;
            int64_t stamp_ns = 0;
            Eigen::Vector3d p = Eigen::Vector3d::Zero();
            Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        };
        GicpDeskewAnchor gicp_deskew_anchor_;
        bool publish_gicp_map_ = true;
        double gicp_map_leaf_size_ = 0.10;
        int gicp_map_max_points_ = 300000;
        std::string gicp_map_frame_ = "gicp_map";
        std::string gicp_map_parent_frame_ = "odom";
        geometry_msgs::msg::TransformStamped gicp_map_tf_;
        bool gicp_map_tf_ready_ = false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_map_cloud_;
        Eigen::Matrix4d gicp_current_to_map_ = Eigen::Matrix4d::Identity();
        bool gicp_map_initialized_ = false;
        Eigen::Matrix4f gicp_init_guess_ = Eigen::Matrix4f::Identity();
        bool enable_lidar_deskew_ = true;
        bool deskew_translation_ = false;
        double state_history_duration_ = 3.0;
        Eigen::Matrix<double, 4, 4> R_lidar_;
        Eigen::Matrix<double, 4, 4> icp_R_lidar_;  // 自适应缩放后的 R_lidar
        Eigen::Matrix3d R_lidar_to_body_;           // 雷达系 → 车体系旋转
        std::deque<StateSnapshot> state_history_;
        mutable std::mutex state_history_mtx_;

        // imu历史队列
        std::deque<ImuSample> imu_history_;
        mutable std::mutex imu_history_mtx_;

        // 保护名义状态 p_/v_/q_ 在 IMU 线程和 lidar 线程之间的读写
        mutable std::mutex state_mtx_;

        // lidarCallback 时刻的 ESKF 状态快照（用于 GICP 速度/yaw 残差观测）
        struct LidarStateSnapshot {
            int64_t stamp_ns = 0;
            Eigen::Vector3d v_body = Eigen::Vector3d::Zero();
            double yaw = 0.0;
        };
        LidarStateSnapshot lidar_state_snap_;
        mutable std::mutex lidar_snap_mtx_;

        // 零偏标定状态机
        enum class CalibState { CALIBRATING, RUNNING };
        CalibState calib_state_ = CalibState::CALIBRATING;
        Eigen::Vector3d calib_acc_sum_  = Eigen::Vector3d::Zero();
        Eigen::Vector3d calib_gyro_sum_ = Eigen::Vector3d::Zero();
        int calib_count_ = 0;
        uint32_t calib_start_t_ms_ = 0;
        double calibration_duration_ = 1.5; // 标定时长（秒），从参数加载

        // 低通滤波系数
        // alpha ∈ (0,1)：越小滤波越重（时间常数越长），越大响应越快
        // 加速度计：alpha 小（平滑强），陀螺仪：alpha 大（响应快）
        double alpha_lowpass_      = 0.05;  // 加速度计 + 轮速（重滤波）
        double alpha_lowpass_gyro_ = 0.3;   // 陀螺仪（轻滤波）

        // 低通滤波器状态（标定结束时初始化，运行阶段每帧更新）
        Eigen::Vector3d gyro_filtered_   = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc_filtered_    = Eigen::Vector3d::Zero();
        Eigen::Vector4d wheel_filtered_  = Eigen::Vector4d::Zero();

        // 原始未滤波数据缓存（每帧 gimbalCallBack 中赋值，供 publishRawOdometry 使用）
        Eigen::Vector3d acc_raw_   = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_raw_  = Eigen::Vector3d::Zero();
        Eigen::Vector4d wheel_raw_ = Eigen::Vector4d::Zero();

        // 原始未滤波位姿状态（简单积分，无ESKF校正）
        Eigen::Vector3d p_raw_;
        Eigen::Vector3d v_raw_;
        Eigen::Quaterniond q_raw_;
        uint32_t last_t_ms_raw_ = 0;
        nav_msgs::msg::Path path_raw_;

        // help
        uint32_t last_t_ms_ = 0; // MCU 上一帧采样时间戳（ms），用于计算 dt
        nav_msgs::msg::Path path_;

        const Eigen::Vector3d G_VEC_{0, 0, -9.80665}; // 重力加速度（与 imu_adapter 的 G 常数一致）
    };
} // nav_data_handle

#endif // MY_NAV2_ROBOT__DATA_HANDLE
