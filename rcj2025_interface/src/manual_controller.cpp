#include "rcj2025_interface/manual_controller.hpp"

namespace rcj2025_interface
{

ManualController::ManualController(const rclcpp::NodeOptions & options)
: Node("manual_controller_node", options)
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&ManualController::joy_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ManualController::publish_cmd_vel, this));
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.linear.y = 0.0;
  cmd_vel_.linear.z = 0.0;
  cmd_vel_.angular.x = 0.0;
  cmd_vel_.angular.y = 0.0;
  cmd_vel_.angular.z = 0.0;
}

ManualController::~ManualController()
{
}

void ManualController::publish_cmd_vel()
{
  cmd_vel_pub_->publish(cmd_vel_);
}

void ManualController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Handle joystick input
  cmd_vel_.linear.x = msg->axes[2];
  cmd_vel_.angular.z = msg->axes[3] * 2;
}

}  // namespace rcj2025_interface
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rcj2025_interface::ManualController)
