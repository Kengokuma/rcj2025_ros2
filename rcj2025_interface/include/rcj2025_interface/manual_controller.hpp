#if !defined(RCJ2025_INTERFACE_MANUAL_CONTROLLER_HPP)
#define RCJ2025_INTERFACE_MANUAL_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace rcj2025_interface
{

class ManualController : public rclcpp::Node
{
public:
  explicit ManualController(const rclcpp::NodeOptions & options);
  ~ManualController();

private:
  void publish_cmd_vel();
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TwistStamped cmd_vel_;
};

}  // namespace rcj2025_interface

#endif  // RCJ2025_INTERFACE_MANUAL_CONTROLLER_HPP