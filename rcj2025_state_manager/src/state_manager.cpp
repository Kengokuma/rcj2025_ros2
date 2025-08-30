#include "rcj2025_msgs/srv/goal_points.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>

using namespace std::chrono_literals;

class StateManager : public rclcpp::Node
{
  public:
    StateManager()
    : Node("state_manager")
    {
      //クライアントの作成
      client_ = this->create_client<rcj2025_msgs::srv::GoalPoints>("GoalPoints");
    }
  
    void run()
    {
      // サーバが存在しているか確認する
      while (!client_->wait_for_service(1s)) {    // サーバの存在を1s待つ
        // 実行が止められた場合、エラーメッセージを表示して終了する
        if (!rclcpp::ok()) {
          return;
        }
        // サーバがない場合、ターミナルにその旨を表示する
        RCLCPP_INFO(this->get_logger(), "Service is not available. waiting...");
      }

      // リクエストを作成し、値を代入する
      auto request = std::make_shared<rcj2025_msgs::srv::GoalPoints::Request>();
      request->request_string = "request";

      auto result = client_->async_send_request(request);
      
      RCLCPP_INFO(this->get_logger(), "Send request.");

      // 送信したリクエストに対する結果が返ってくるまで待つ
      auto return_code = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), result);

      // 正しく結果が返ってきたかどうかを判定し、メッセージを表示
      auto received_pose = result.get()->pose;

      if (return_code == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Received pose: x=%f, y=%f, z=%f",
        received_pose.pose.position.x,
        received_pose.pose.position.y,
        received_pose.pose.position.z);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
      }  
    }

  private:
    rclcpp::Client<rcj2025_msgs::srv::GoalPoints>::SharedPtr client_; // これを追加する
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 引数の個数を確認
  if (argc != 2) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "USAGE: GoalPoints client <string> <geometry_msgs>");
    return 0;
  }

  // AddTwoIntsClientクラスのインスタンスを生成
  auto client = std::make_shared<StateManager>();

  // 実行
  client->run();

  rclcpp::shutdown();

  return 0;
}