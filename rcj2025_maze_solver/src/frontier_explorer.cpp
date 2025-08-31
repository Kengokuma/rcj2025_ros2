#include "frontier_explorer.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace rcj2025_maze_solver
{

FrontierExplorer::FrontierExplorer(const rclcpp::NodeOptions & options)
: Node("frontier_explorer_node", options)
{
  // ROS 2 サブスクライバー
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 1, std::bind(&FrontierExplorer::mapCallback, this, std::placeholders::_1));

  // TF2 リスナー
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Action Clientの作成
  this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  RCLCPP_INFO(this->get_logger(), "Frontier Explorer Node has been started.");
}

FrontierExplorer::~FrontierExplorer()
{
}

std::pair<double, double> FrontierExplorer::findFarthestFrontier(
  const std::vector<std::pair<double, double>> & frontiers,
  const geometry_msgs::msg::Point & robot_pos)
{
  double max_dist_sq = -1.0;  // 最小値ではなく、負の値で初期化
  std::pair<double, double> farthest_point;

  for (const auto & p : frontiers) {
    double dist_sq = std::pow(p.first - robot_pos.x, 2) + std::pow(p.second - robot_pos.y, 2);
    if (dist_sq > max_dist_sq) {  // < ではなく > に変更
      max_dist_sq = dist_sq;
      farthest_point = p;
    }
  }
  return farthest_point;
}

void FrontierExplorer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  // ゴールをパブリッシュする代わりに送信
  findAndSendFrontierGoal();
}

bool FrontierExplorer::isAdjacentToFreeSpace(
  int x, int y, int width, int height, const std::vector<int8_t> & data)
{
  // この関数内のロジック全体を確認
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;

      int nx = x + dx;
      int ny = y + dy;

      if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
        int n_index = ny * width + nx;
        if (data[n_index] == 0) {
          return true;
        }
      }
    }
  }
  return false;
}

std::vector<std::pair<double, double>> FrontierExplorer::findFrontiers()
{
  int width = current_map_->info.width;
  int height = current_map_->info.height;
  float resolution = current_map_->info.resolution;
  double origin_x = current_map_->info.origin.position.x;
  double origin_y = current_map_->info.origin.position.y;
  const auto & data = current_map_->data;

  std::vector<std::pair<double, double>> frontiers;
  // 15cmをピクセル単位に変換
  int safety_dist_pixels = static_cast<int>(0.15 / resolution);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      if (data[index] == -1) {  // 未知のセルを探す
        // Step 1: 自由空間と隣接しているかチェック
        if (isAdjacentToFreeSpace(x, y, width, height, data)) {
          // Step 2: 安全距離内に障害物がないかチェック
          bool is_safe = true;
          for (int dy_s = -safety_dist_pixels; dy_s <= safety_dist_pixels; ++dy_s) {
            for (int dx_s = -safety_dist_pixels; dx_s <= safety_dist_pixels; ++dx_s) {
              int sx = x + dx_s;
              int sy = y + dy_s;
              if (sx >= 0 && sx < width && sy >= 0 && sy < height) {
                int s_index = sy * width + sx;
                if (data[s_index] == 100) {  // 障害物 (100)
                  is_safe = false;
                  break;
                }
              }
            }
            if (!is_safe) break;
          }

          if (is_safe) {
            double map_x = x * resolution + origin_x + resolution / 2.0;
            double map_y = y * resolution + origin_y + resolution / 2.0;
            frontiers.push_back({map_x, map_y});
          }
        }
      }
    }
  }
  return frontiers;
}

void FrontierExplorer::findAndSendFrontierGoal()
{
  // アクションサーバーが利用可能になるまで待機
  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Action server not available. Trying again.");
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped = tf_buffer_->lookupTransform(
    "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(1.0));

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header = transform_stamped.header;
  robot_pose.pose.position.x = transform_stamped.transform.translation.x;
  robot_pose.pose.position.y = transform_stamped.transform.translation.y;
  robot_pose.pose.position.z = transform_stamped.transform.translation.z;
  robot_pose.pose.orientation = transform_stamped.transform.rotation;

  // フロンティアを探索
  std::vector<std::pair<double, double>> frontiers = findFrontiers();
  if (frontiers.empty()) {
    RCLCPP_INFO(this->get_logger(), "No frontiers found. The map might be fully explored.");
    return;
  }

  // 最も遠いフロンティアを選択
  std::pair<double, double> farthest_frontier =
    findFarthestFrontier(frontiers, robot_pose.pose.position);

  // Action Goalの作成
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = farthest_frontier.first;
  goal_msg.pose.pose.position.y = farthest_frontier.second;
  goal_msg.pose.pose.orientation.w = 1.0;

  // ゴール送信オプションの設定
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&FrontierExplorer::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &FrontierExplorer::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&FrontierExplorer::resultCallback, this, std::placeholders::_1);

  // Goalを送信
  RCLCPP_INFO(
    this->get_logger(), "Sending new goal: (%.2f, %.2f)", farthest_frontier.first,
    farthest_frontier.second);
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

// === Action Clientのコールバック関数 ===

void FrontierExplorer::goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result");
  }
}

void FrontierExplorer::feedbackCallback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
}

void FrontierExplorer::resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  // ゴールが完了したときに新しいゴールを探索
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      // 次のゴール探索をトリガー
      findAndSendFrontierGoal();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted. Searching for a new goal...");
      findAndSendFrontierGoal();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled. Searching for a new goal...");
      findAndSendFrontierGoal();
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code. Searching for a new goal...");
      findAndSendFrontierGoal();
      break;
  }
}

}  // namespace rcj2025_maze_solver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rcj2025_maze_solver::FrontierExplorer)