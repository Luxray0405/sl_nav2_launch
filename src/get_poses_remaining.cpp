#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

// Humble でもOKなエイリアス（FeedbackMessage は Impl 内に定義）
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using FeedbackMsg = NavigateThroughPoses::Impl::FeedbackMessage;

class RemainingSub : public rclcpp::Node {
public:
  RemainingSub() : rclcpp::Node("navthrough_remaining_sub") {
    // Nav2のNavigateThroughPosesのFeedbackは "navigate_through_poses/_action/feedback"
    sub_ = this->create_subscription<FeedbackMsg>(
      "navigate_through_poses/_action/feedback",
      rclcpp::QoS(10),
      [this](const FeedbackMsg::SharedPtr msg) {
        // フィードバックに含まれる残ウェイポイント数を表示
        const int rem = msg->feedback.number_of_poses_remaining;
        RCLCPP_INFO(this->get_logger(), "remaining points: %d", rem);
      }
    );
  }
private:
  rclcpp::Subscription<FeedbackMsg>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemainingSub>());
  rclcpp::shutdown();
  return 0;
}
