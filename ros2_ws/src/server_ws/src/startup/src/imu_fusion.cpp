#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

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

    RCLCPP_INFO(this->get_logger(), "IMU fusion node started");
  }

private:
  void magCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_mag_ = *msg;
    have_mag_ = true;
    publishIfReadyFromMag();
  }

  void rawCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_raw_ = *msg;
    have_raw_ = true;
    publishIfReadyFromRaw();
  }

  void publishIfReadyFromMag()
  {
    if (!have_raw_) {
      return;
    }
    publishFused(/*use_raw_header=*/true);
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

  sensor_msgs::msg::Imu last_mag_;
  sensor_msgs::msg::Imu last_raw_;
  bool have_mag_ {false};
  bool have_raw_ {false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
