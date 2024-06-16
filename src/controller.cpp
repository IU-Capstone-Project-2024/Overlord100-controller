#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "overlord100_msgs/msg/wheels_data.hpp" 

# define M_PI       3.14159265358979323846  /* pi */

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController() : Node("diff_drive_controller")
  {
    // Parameters for the differential drive robot
    this->declare_parameter<double>("robot_base", 0.5); // Distance between the wheels
    this->declare_parameter<double>("wheel_radius", 0.1); // Radius of the wheels

    this->get_parameter("robot_base", robot_base_);
    this->get_parameter("wheel_radius", wheel_radius_);

    // Subscriber to the cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1));

    // Publisher for the wheel velocities 
    wheels_pub_ = this->create_publisher<overlord100_msgs::msg::WheelsData>("wheels_control", 10);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Extract linear and angular velocities from the message
    double angular_velocity = msg->angular.z;
    double lin_vx = msg->linear.x;
    double lin_vy = msg->linear.y;
    double linear_velocity = std::sqrt(lin_vx*lin_vx  + lin_vy*lin_vy);
    
    // Compute the wheel velocities
    double angular_right = (2 * linear_velocity + half_robot_base_ * angular_velocity) / (2 * wheel_radius_);
    double angular_left =  (2 * linear_velocity - half_robot_base_ * angular_velocity) / (2 * wheel_radius_);
    double angular_right_rpm = angular_right * 30 / M_PI;
    double angular_left_rpm = angular_left * 30 / M_PI;

    // Create and publish the custom message
    auto wheels_msg = overlord100_msgs::msg::WheelsData();
    wheels_msg.left = static_cast<float>(angular_left_rpm);
    wheels_msg.right = static_cast<float>(angular_right_rpm);
    RCLCPP_INFO(this->get_logger(), "Publishing sth: '%s'", std::to_string(wheels_msg.left).c_str());

    wheels_pub_->publish(wheels_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<overlord100_msgs::msg::WheelsData>::SharedPtr wheels_pub_;

  double wheel_radius_;
  double robot_base_;
  double half_robot_base_ = robot_base_ / 2;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}