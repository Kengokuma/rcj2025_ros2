#include "rcj2025_interface/manual_controller.hpp"

namespace rcj2025_interface
{

ManualController::ManualController(const rclcpp::NodeOptions & options)
: Node("manual_controller_node", options)
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&ManualController::joy_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ManualController::publish_cmd_vel, this));
  cmd_vel_.header.frame_id = "base_link";
  cmd_vel_.twist.linear.x = 0.0;
  cmd_vel_.twist.linear.y = 0.0;
  cmd_vel_.twist.linear.z = 0.0;
  cmd_vel_.twist.angular.x = 0.0;
  cmd_vel_.twist.angular.y = 0.0;
  cmd_vel_.twist.angular.z = 0.0;

}

ManualController::~ManualController()
{
}

void ManualController::publish_cmd_vel()
{
  // Publish the cmd_vel message
  cmd_vel_.header.stamp = this->now();
  cmd_vel_pub_->publish(cmd_vel_);
}

void ManualController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Handle joystick input
  cmd_vel_.twist.linear.x = msg->axes[2];
  cmd_vel_.twist.angular.z = msg->axes[3] * 10;
  RCLCPP_INFO(this->get_logger(), "Joy message received: [%f, %f]", cmd_vel_.twist.linear.x, cmd_vel_.twist.angular.z);
}

}  // namespace rcj2025_interface
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rcj2025_interface::ManualController)
