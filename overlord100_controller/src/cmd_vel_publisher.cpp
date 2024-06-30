#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node
{
public:
  CmdVelPublisher()
      : Node("cmd_vel_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&CmdVelPublisher::publish_velocity, this));
  }

private:
  void publish_velocity()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 10.0;
    message.linear.y = 10.0;
    message.linear.z = 0.0;
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = 100.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: '%f' angular.z: '%f'", message.linear.x, message.angular.z);
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelPublisher>());
  rclcpp::shutdown();
  return 0;
}
