
// #include <cstdio>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <stdexcept>
// #include <string>

// #include "geometry_msgs/msg/twist.hpp"
// #include "overlord100_msgs/msg/wheels_data.hpp"
// #include "overlord100_msgs/srv/change_mode.hpp"

// /*
//  Node implementing mode switcher to toggle between manual and auto modes

//  Service: ChangeMode via the /change_mode topic

//  Manual mode:
//  - Checks whether the input velocity is validw
//  - Publishes: Twist messages over the /cmd_vel topic

//  Auto mode:
//  - Launches the autonomous navigation stack
//  - TEMPORARILY publishes: Twist messages over the /cmd_vel topic
// */
// class NavigationController : public rclcpp::Node {
//  public:
//   NavigationController() : Node("navigation_controller"), current_mode_(0) {
//     mode_service_ = this->create_service<overlord100_msgs::srv::ChangeMode>(
//         "change_mode",
//         std::bind(&NavigationController::changeModeCallback, this,
//                   std::placeholders::_1, std::placeholders::_2));

//     manual_control_sub_ =
//     this->create_subscription<geometry_msgs::msg::Twist>(
//         "man_cmd_vel", 10,
//         std::bind(&NavigationController::manualControlCallback, this,
//                   std::placeholders::_1));

//     autonomous_control_sub_ =
//         this->create_subscription<geometry_msgs::msg::Twist>(
//             "auto_cmd_vel", 10,
//             std::bind(&NavigationController::autonomousControlCallback, this,
//                       std::placeholders::_1));

//     cmd_vel_pub_ =
//         this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
//   }

//  private:
//   void changeModeCallback(
//       const std::shared_ptr<overlord100_msgs::srv::ChangeMode::Request>
//       request, std::shared_ptr<overlord100_msgs::srv::ChangeMode::Response>
//       response) {
//     current_mode_ = request->mode;
//     if (current_mode_ == 1) {
//       RCLCPP_INFO(this->get_logger(), "Switching to autonomous mode");
//       // Launch path planner using system call
//       std::string command = "ros2 launch your_package path_planner.launch.py
//       &"; std::system(command.c_str());
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Switching to manual mode");
//     }
//     response->success = true;
//   }

//   // bool isVelocityValid(const geometry_msgs::msg::Twist::SharedPtr msg) {
//   //   return (msg->angular.x == 0 && msg->angular.y == 0 &&
//   //           msg->angular.z <= 100 && msg->linear.z == 0 &&
//   //           msg->linear.x <= 100 && msg->linear.y <= 100);
//   // }

//   void velocityHandler(const geometry_msgs::msg::Twist::SharedPtr msg) {
//     if (msg->angular.z > 100) {
//       msg->angular.z = 100;
//     }
//     if (msg->linear.x > 100) {
//       msg->linear.x = 100;
//     }
//     if (msg->linear.y > 100) {
//       msg->linear.y = 100;
//     }
//     if (msg->angular.z < -100) {
//       msg->angular.z = -100;
//     }
//     if (msg->linear.x < -100) {
//       msg->linear.x = -100;
//     }
//     if (msg->linear.y < -100) {
//       msg->linear.y = -100;
//     }
//     if (msg->angular.x != 0 || msg->angular.y != 0 || msg->linear.z != 0) {
//       msg->angular.x = 0;
//       msg->angular.y = 0;
//       msg->linear.z = 0;
//     }
//     RCLCPP_INFO(this->get_logger(), "Your velocity is not valid. Switched to
//     a default value '%s'",
//               std::to_string(msg->angular.x).c_str());

//   }

//   void manualControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
//   {
//     velocityHandler(msg);
//     if (current_mode_ == 0) {
//       cmd_vel_pub_->publish(*msg);
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Error");
//     }
//   }

//   void autonomousControlCallback(
//       const geometry_msgs::msg::Twist::SharedPtr msg) {
//     if (current_mode_ == 1) {
//       cmd_vel_pub_->publish(*msg);
//     }
//   }

//   int32_t current_mode_;
//   rclcpp::Service<overlord100_msgs::srv::ChangeMode>::SharedPtr
//   mode_service_; rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
//       manual_control_sub_;
//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
//       autonomous_control_sub_;
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<NavigationController>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <algorithm>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "overlord100_msgs/msg/wheels_data.hpp"
#include "overlord100_msgs/srv/change_mode.hpp"

class NavigationController : public rclcpp::Node {
 public:
  NavigationController() : Node("navigation_controller"), current_mode_(0) {
    mode_service_ = this->create_service<overlord100_msgs::srv::ChangeMode>(
        "change_mode",
        std::bind(&NavigationController::changeModeCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    manual_control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "man_cmd_vel", 10,
        std::bind(&NavigationController::manualControlCallback, this,
                  std::placeholders::_1));

    autonomous_control_sub_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "auto_cmd_vel", 10,
            std::bind(&NavigationController::autonomousControlCallback, this,
                      std::placeholders::_1));

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

 private:
  static constexpr double UPPER_VELOCITY_LIMIT = 100.0;
  static constexpr double LOWER_VELOCITY_LIMIT = -100.0;

  void changeModeCallback(
      const std::shared_ptr<overlord100_msgs::srv::ChangeMode::Request> request,
      std::shared_ptr<overlord100_msgs::srv::ChangeMode::Response> response) {
    current_mode_ = request->mode;
    if (current_mode_ == 1) {
      RCLCPP_INFO(this->get_logger(), "Switching to autonomous mode");
      // Launch path planner using system call
      std::string command = "ros2 launch your_package path_planner.launch.py &";
      std::system(command.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Switching to manual mode");
    }
    response->success = true;
  }

  void velocityHandler(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // bool adjustedExistingComponents = false;
    // bool adjustedNonExistingComponents = false;

    auto clampAndCheckValidValues = [&](double& value,
                                        const std::string& name) {
      if (value > UPPER_VELOCITY_LIMIT) {
        value = UPPER_VELOCITY_LIMIT;
        // adjustedExistingComponents = true;
        RCLCPP_WARN(
            this->get_logger(),
            "%s exceeded upper limit. Adjusted to the default value: %f",
            name.c_str(), UPPER_VELOCITY_LIMIT);
      } else if (value < LOWER_VELOCITY_LIMIT) {
        value = LOWER_VELOCITY_LIMIT;
        // adjustedExistingComponents = true;
        RCLCPP_WARN(this->get_logger(),
                    "%s exceeded lower limit. Adjusted to the default value %f",
                    name.c_str(), LOWER_VELOCITY_LIMIT);
      }
    };

    clampAndCheckValidValues(msg->angular.z, "angular.z");
    clampAndCheckValidValues(msg->linear.x, "linear.x");
    clampAndCheckValidValues(msg->linear.y, "linear.y");

    auto clampAndCheckNonExistingComponents = [&](double& value,
                                                  const std::string& name) {
      if (value != 0) {
        // adjustedNonExistingComponents = true;
        value = 0;
        RCLCPP_WARN(this->get_logger(),
                    "%s is not a valid velocity component. Adjusted to zero.",
                    name.c_str());
      }
    };

    clampAndCheckNonExistingComponents(msg->angular.x, "angular.x");
    clampAndCheckNonExistingComponents(msg->angular.y, "angular.y");
    clampAndCheckNonExistingComponents(msg->linear.z, "linear.z");

    // Rewrite in lambda function
    // if (msg->angular.x != 0 || msg->angular.y != 0 || msg->linear.z != 0) {
    //   msg->angular.x = 0;
    //   msg->angular.y = 0;
    //   msg->linear.z = 0;
    //   adjusted = true;
    //   RCLCPP_WARN(this->get_logger(),
    //               "Invalid angular.x, angular.y, or linear.z set to 0");
    // }
  }

  void manualControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocityHandler(msg);
    if (current_mode_ == 0) {
      cmd_vel_pub_->publish(*msg);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot publish in manual mode while in autonomous mode");
    }
  }

  void autonomousControlCallback(
      const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (current_mode_ == 1) {
      cmd_vel_pub_->publish(*msg);
    }
  }

  int32_t current_mode_;
  rclcpp::Service<overlord100_msgs::srv::ChangeMode>::SharedPtr mode_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      manual_control_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      autonomous_control_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationController>());
  rclcpp::shutdown();
  return 0;
}
