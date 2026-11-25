#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>

class ImuFusionNode : public rclcpp::Node
{
public:
  ImuFusionNode() : Node("imu_fusion")
  {
    using std::placeholders::_1;

    mag_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/mag_orientation", 10,
      std::bind(&ImuFusionNode::magCallback, this, _1));

    raw_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", 10,
      std::bind(&ImuFusionNode::rawCallback, this, _1));

    fused_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/imu/combined", 10);

    wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mecanum_drive_controller/odometry", 10,
      std::bind(&ImuFusionNode::wheelOdomCallback, this, _1));

    // The ICM20948 gyro has a z bias of 0.01-0.02 rad/s that wanders as it warms up, and the
    // driver reports zero covariance, so the EKF trusted it absolutely and the stationary robot
    // spun ~1 deg/s in odom. The bias is averaged over the first samples (robot must be still at
    // launch), then kept tracking whenever the wheel encoders report the robot fully stopped, and
    // a real covariance is filled in where the driver sends zeros.
    calibration_samples_ = this->declare_parameter<int>("gyro_calibration_samples", 200);
    bias_alpha_ = this->declare_parameter<double>("gyro_bias_alpha", 0.005);  // ~4 s at 50 Hz
    gyro_variance_ = this->declare_parameter<double>("gyro_variance", 1e-3);

    RCLCPP_INFO(
      this->get_logger(), "IMU fusion node started, estimating gyro bias over %d samples (keep the robot still)",
      calibration_samples_);
  }

private:
  void magCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Only stored: publishing here too would re-send the same gyro sample with an old stamp
    last_mag_ = *msg;
    have_mag_ = true;
  }

  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & v = msg->twist.twist;
    const bool still = std::fabs(v.linear.x) < 1e-4 && std::fabs(v.linear.y) < 1e-4 &&
      std::fabs(v.angular.z) < 1e-4;
    const rclcpp::Time now = this->now();
    if (!still) {
      last_wheel_motion_ = now;
    }
    // Wait out the tail of a stop, when the wheels read zero but the body may still be settling
    wheels_still_ = still && (now - last_wheel_motion_).seconds() > 0.5;
    last_wheel_odom_ = now;
  }

  void rawCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (bias_count_ < calibration_samples_) {
      bias_x_ += msg->angular_velocity.x;
      bias_y_ += msg->angular_velocity.y;
      bias_z_ += msg->angular_velocity.z;
      if (++bias_count_ == calibration_samples_) {
        bias_x_ /= bias_count_;
        bias_y_ /= bias_count_;
        bias_z_ /= bias_count_;
        RCLCPP_INFO(
          this->get_logger(), "Gyro bias [%.4f, %.4f, %.4f] rad/s, publishing /imu/combined",
          bias_x_, bias_y_, bias_z_);
      }
      return;
    }
    // Zero-velocity update: only while wheel odometry is fresh and says the robot is stopped
    if (wheels_still_ && (this->now() - last_wheel_odom_).seconds() < 0.2) {
      bias_x_ += bias_alpha_ * (msg->angular_velocity.x - bias_x_);
      bias_y_ += bias_alpha_ * (msg->angular_velocity.y - bias_y_);
      bias_z_ += bias_alpha_ * (msg->angular_velocity.z - bias_z_);
    }
    last_raw_ = *msg;
    last_raw_.angular_velocity.x -= bias_x_;
    last_raw_.angular_velocity.y -= bias_y_;
    last_raw_.angular_velocity.z -= bias_z_;
    for (size_t i = 0; i < 9; i += 4) {
      if (last_raw_.angular_velocity_covariance[i] <= 0.0) {
        last_raw_.angular_velocity_covariance[i] = gyro_variance_;
      }
    }
    have_raw_ = true;
    publishIfReadyFromRaw();
  }

  void publishIfReadyFromRaw()
  {
    if (!have_mag_) {
      return;
    }
    publishFused(/*use_raw_header=*/true);
  }

  void publishFused(bool use_raw_header)
  {
    sensor_msgs::msg::Imu fused;

    // Header: prefer raw IMU timing, but keep mag frame if different
    if (use_raw_header) {
      fused.header = last_raw_.header;
      // If mag frame_id is set and different, keep mag frame_id for orientation
      if (!last_mag_.header.frame_id.empty()) {
        fused.header.frame_id = last_mag_.header.frame_id;
      }
    } else {
      fused.header = last_mag_.header;
    }

    // Orientation from magnetometer-derived IMU message
    fused.orientation = last_mag_.orientation;
    for (size_t i = 0; i < fused.orientation_covariance.size(); ++i) {
      fused.orientation_covariance[i] = last_mag_.orientation_covariance[i];
    }

    // Angular velocity and linear acceleration from raw IMU
    fused.angular_velocity = last_raw_.angular_velocity;
    for (size_t i = 0; i < fused.angular_velocity_covariance.size(); ++i) {
      fused.angular_velocity_covariance[i] = last_raw_.angular_velocity_covariance[i];
    }

    fused.linear_acceleration = last_raw_.linear_acceleration;
    for (size_t i = 0; i < fused.linear_acceleration_covariance.size(); ++i) {
      fused.linear_acceleration_covariance[i] = last_raw_.linear_acceleration_covariance[i];
    }

    fused_pub_->publish(fused);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr fused_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;

  sensor_msgs::msg::Imu last_mag_;
  sensor_msgs::msg::Imu last_raw_;
  bool have_mag_ {false};
  bool have_raw_ {false};

  int calibration_samples_ {200};
  int bias_count_ {0};
  double bias_x_ {0.0}, bias_y_ {0.0}, bias_z_ {0.0};
  double bias_alpha_ {0.005};
  double gyro_variance_ {1e-3};
  bool wheels_still_ {false};
  rclcpp::Time last_wheel_motion_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wheel_odom_ {0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
