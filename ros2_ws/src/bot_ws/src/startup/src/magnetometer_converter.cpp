#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class MagnetometerConverter : public rclcpp::Node
{
public:
    MagnetometerConverter() : Node("magnetometer_converter")
    {
        // Subscribe to magnetometer data
        mag_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "/imu/mag_raw",
            10,
            std::bind(&MagnetometerConverter::magnetometer_callback, this, std::placeholders::_1)
        );
        
        // Publisher for IMU message with magnetometer-derived orientation
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/mag_orientation",
            10
        );
        
        RCLCPP_INFO(this->get_logger(), "Magnetometer converter node started");
    }

private:
    void magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
    {
        // Create IMU message with orientation from magnetometer
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header = msg->header;
        
        // Calculate yaw from magnetometer readings
        // Note: This assumes the magnetometer is calibrated and the robot is on level ground
        double mag_x = msg->magnetic_field.x;
        double mag_y = msg->magnetic_field.y;
        
        // Calculate heading (yaw) from magnetometer
        // atan2(y, x) gives the angle in radians
        double yaw = std::atan2(mag_y, mag_x);
        
        // Convert yaw to quaternion (roll=0, pitch=0, yaw=calculated)
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);
        
        imu_msg.orientation.x = quaternion.x();
        imu_msg.orientation.y = quaternion.y();
        imu_msg.orientation.z = quaternion.z();
        imu_msg.orientation.w = quaternion.w();
        
        // Set orientation covariance
        // Only yaw (index 8) has valid data, others are unknown
        std::fill(imu_msg.orientation_covariance.begin(), imu_msg.orientation_covariance.end(), 0.0);
        imu_msg.orientation_covariance[0] = -1.0;  // roll (unknown)
        imu_msg.orientation_covariance[4] = -1.0;  // pitch (unknown)
        imu_msg.orientation_covariance[8] = 0.01;  // yaw (from magnetometer, low covariance)
        
        // Set angular velocity and linear acceleration as invalid
        imu_msg.angular_velocity_covariance[0] = -1.0;
        imu_msg.linear_acceleration_covariance[0] = -1.0;
        
        // Publish the converted message
        imu_publisher_->publish(imu_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MagnetometerConverter>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
