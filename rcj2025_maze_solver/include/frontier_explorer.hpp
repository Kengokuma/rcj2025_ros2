#ifndef RCJ2025_MAZE_SOLVER__FRONTIER_EXPLORER_HPP_
#define RCJ2025_MAZE_SOLVER__FRONTIER_EXPLORER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <vector>

namespace rcj2025_maze_solver
{

class FrontierExplorer : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit FrontierExplorer(const rclcpp::NodeOptions & options);
  ~FrontierExplorer();

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void findAndSendFrontierGoal();

  std::vector<std::pair<double, double>> findFrontiers();
  bool isAdjacentToFreeSpace(int x, int y, int width, int height, const std::vector<int8_t> & data);
  std::pair<double, double> findFarthestFrontier(
    const std::vector<std::pair<double, double>> & frontiers,
    const geometry_msgs::msg::Point & robot_pos);
  
  // 新しいAction Clientのメンバー変数とコールバック関数
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void feedbackCallback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
  
  // メンバー変数
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
};

}  // namespace rcj2025_maze_solver

#endif  // RCJ2025_MAZE_SOLVER__FRONTIER_EXPLORER_HPP_