#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <termios.h> // POSIX ターミナル制御
#include <unistd.h>  // STDIN_FILENO

// waypoint_saverクラスの定義
class WaypointSaver : public rclcpp::Node
{
public:
  WaypointSaver() : Node("waypoint_saver")
  {
    // パラメータの宣言と取得
    this->declare_parameter<std::string>("save_path", "~/waypoints.yaml");
    this->get_parameter("save_path", save_path_);

    // Subscriberの作成
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&WaypointSaver::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waypoint Saver Node is running.");
    RCLCPP_INFO(this->get_logger(), "Waypoints will be saved to: %s", save_path_.c_str());
    print_instructions();
  }

  // ウェイポイントを保存する関数
  void save_waypoints_to_yaml()
  {
    std::lock_guard<std::mutex> lock(waypoints_mutex_); // スレッドセーフのためのロック
    if (waypoints_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No waypoints to save.");
      return;
    }

    YAML::Node root;
    for (const auto& pose_stamped : waypoints_) {
      YAML::Node pose_node;
      pose_node["header"]["frame_id"] = pose_stamped.header.frame_id;
      pose_node["header"]["stamp"]["sec"] = pose_stamped.header.stamp.sec;
      pose_node["header"]["stamp"]["nanosec"] = pose_stamped.header.stamp.nanosec;
      pose_node["pose"]["position"]["x"] = pose_stamped.pose.position.x;
      pose_node["pose"]["position"]["y"] = pose_stamped.pose.position.y;
      pose_node["pose"]["position"]["z"] = pose_stamped.pose.position.z;
      pose_node["pose"]["orientation"]["x"] = pose_stamped.pose.orientation.x;
      pose_node["pose"]["orientation"]["y"] = pose_stamped.pose.orientation.y;
      pose_node["pose"]["orientation"]["z"] = pose_stamped.pose.orientation.z;
      pose_node["pose"]["orientation"]["w"] = pose_stamped.pose.orientation.w;
      root["poses"].push_back(pose_node);
    }

    try {
      std::ofstream fout(save_path_);
      fout << root;
      RCLCPP_INFO(this->get_logger(), "Successfully saved %zu waypoints to %s", waypoints_.size(), save_path_.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save waypoints: %s", e.what());
    }
  }

  // ウェイポイントをクリアする関数
  void clear_waypoints()
  {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    waypoints_.clear();
    RCLCPP_INFO(this->get_logger(), "All waypoints have been cleared.");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    waypoints_.push_back(*msg);
    RCLCPP_INFO(this->get_logger(), "Added waypoint #%zu. Total waypoints: %zu", waypoints_.size(), waypoints_.size());
  }

  void print_instructions()
  {
    RCLCPP_INFO(this->get_logger(), "-----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Usage:");
    RCLCPP_INFO(this->get_logger(), "1. Use the \"2D Goal Pose\" tool in RViz to add waypoints.");
    RCLCPP_INFO(this->get_logger(), "2. When finished, press 's' in this terminal to save.");
    RCLCPP_INFO(this->get_logger(), "3. Press 'c' to clear all waypoints.");
    RCLCPP_INFO(this->get_logger(), "4. Press 'q' to quit the node.");
    RCLCPP_INFO(this->get_logger(), "-----------------------------------------");
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::string save_path_;
  std::mutex waypoints_mutex_;
};

// ターミナルからのキー入力を1文字ずつ受け取るための関数
int getch()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

// キーボード入力を監視する関数
void keyboard_listener(std::shared_ptr<WaypointSaver> node)
{
  while (rclcpp::ok()) {
    int key = getch();
    if (key == 's') {
      node->save_waypoints_to_yaml();
    } else if (key == 'c') {
      node->clear_waypoints();
    } else if (key == 'q' || key == 3) { // q or Ctrl-C
      RCLCPP_INFO(rclcpp::get_logger("keyboard_listener"), "Shutdown requested.");
      rclcpp::shutdown();
      break;
    }
  }
}

// main関数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointSaver>();

  // キーボード監視を別スレッドで実行
  std::thread key_thread(keyboard_listener, node);
  
  // ROSのメインループ
  rclcpp::spin(node);

  key_thread.join();
  return 0;
}