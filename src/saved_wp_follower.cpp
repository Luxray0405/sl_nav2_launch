#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <string>
#include <vector>
#include <memory>

class SavedWpFollower : public rclcpp::Node
{
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    explicit SavedWpFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("saved_wp_follower", options)
    {
        // パラメータを宣言し、ファイルパスを取得
        this->declare_parameter<std::string>("waypoint_file_path", "default_path.yaml");
        std::string waypoint_file_path = this->get_parameter("waypoint_file_path").as_string();

        // アクションクライアント、Publisher、Service Clientを作成
        this->action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "/follow_waypoints");
        event_publisher_ = this->create_publisher<std_msgs::msg::String>("/waypoint_event", 10);
        dummy_service_client_ = this->create_client<std_srvs::srv::Empty>("/trigger_action");
        start_motion_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/start_motion", 10);

        // ウェイポイントをロードしてゴールを送信
        send_goal(waypoint_file_path);
    }

private:
    // ROS 2関連のメンバー変数
    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr dummy_service_client_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_motion_publisher_;
    
    // 状態管理用のメンバー変数
    int last_processed_waypoint_index_ = -1; //最後に処理したウェイポイントのインデックスを記録
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_; // 送信したウェイポイントを保持する

    // YAMLファイルからウェイポイントをロードする関数
    std::vector<geometry_msgs::msg::PoseStamped> load_waypoints_from_yaml(const std::string& file_path)
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;

        // ファイルの存在確認
        std::ifstream file(file_path);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "Waypoint file not found at: %s", file_path.c_str());
            return waypoints;
        }

        RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s", file_path.c_str());
        
        try {
            YAML::Node root = YAML::LoadFile(file_path);
            const YAML::Node& poses_node = root["poses"];

            if (!poses_node.IsSequence()) {
                RCLCPP_ERROR(this->get_logger(), "'poses' key is not a sequence in the YAML file.");
                return waypoints;
            }

            for (const auto& pose_node : poses_node) {
                geometry_msgs::msg::PoseStamped ps;

                ps.header.frame_id = pose_node["header"]["frame_id"].as<std::string>();
                ps.header.stamp.sec = pose_node["header"]["stamp"]["sec"].as<int32_t>();
                ps.header.stamp.nanosec = pose_node["header"]["stamp"]["nanosec"].as<uint32_t>();

                ps.pose.position.x = pose_node["pose"]["position"]["x"].as<double>();
                ps.pose.position.y = pose_node["pose"]["position"]["y"].as<double>();
                ps.pose.position.z = pose_node["pose"]["position"]["z"].as<double>();

                ps.pose.orientation.x = pose_node["pose"]["orientation"]["x"].as<double>();
                ps.pose.orientation.y = pose_node["pose"]["orientation"]["y"].as<double>();
                ps.pose.orientation.z = pose_node["pose"]["orientation"]["z"].as<double>();
                ps.pose.orientation.w = pose_node["pose"]["orientation"]["w"].as<double>();

                waypoints.push_back(ps);
            }
            RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints.", waypoints.size());
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error while parsing YAML file: %s", e.what());
        }

        return waypoints;
    }

    // アクションゴールを送信する関数
    void send_goal(const std::string& file_path)
    {
        // 読み込んだウェイポイントをメンバー変数に保存
        this->waypoints_ = load_waypoints_from_yaml(file_path);
        if (this->waypoints_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No waypoints to send. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = this->waypoints_; // 保存したメンバー変数を使用

        RCLCPP_INFO(this->get_logger(), "Sending goal request...");
        
        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&SavedWpFollower::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&SavedWpFollower::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&SavedWpFollower::result_callback, this, std::placeholders::_1);
        
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // ウェイポイント到着時に実行されるアクションを定義する関数
    void on_waypoint_reached(int reached_waypoint_index)
    {
        RCLCPP_INFO(this->get_logger(), "======== Waypoint %d Reached! ========", reached_waypoint_index);

        // 最後のウェイポイントのインデックスを動的に取得
        int last_waypoint_index = waypoints_.size() - 1;

        // if-else if文で処理を分岐
        if (reached_waypoint_index == last_waypoint_index)
        {
            // 最後のウェイポイントに到着した場合の処理
            RCLCPP_INFO(this->get_logger(), "Final waypoint (index %d) reached! Publishing to /start_motion.", last_waypoint_index);
            auto empty_msg = std_msgs::msg::Empty();
            start_motion_publisher_->publish(empty_msg);
        }
        else if (reached_waypoint_index == 0)
        {
            // // 最初のウェイポイント(インデックス0)に到着
            // RCLCPP_INFO(this->get_logger(), "Action for waypoint 0: Publishing a message.");
            // auto msg = std_msgs::msg::String();
            // msg.data = "Waypoint 0 has been reached.";
            // event_publisher_->publish(msg);
        }
        else if (reached_waypoint_index == 2)
        {
        //     // 3番目のウェイポイント(インデックス2)に到着
        //     RCLCPP_INFO(this->get_logger(), "Action for waypoint 2: Calling a service.");
        //     if (!dummy_service_client_->wait_for_service(std::chrono::seconds(1))) {
        //         RCLCPP_WARN(this->get_logger(), "Service /trigger_action not available.");
        //     } else {
        //         auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        //         dummy_service_client_->async_send_request(request);
        //     }
        }
        else
        {
            // 上記のどの条件にも当てはまらない場合のデフォルト処理
            RCLCPP_INFO(this->get_logger(), "No specific action defined for waypoint %d.", reached_waypoint_index);
        }
        
        RCLCPP_INFO(this->get_logger(), "=========================================");
    }

    // 各種コールバック関数
    void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleFollowWaypoints::SharedPtr,
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
        int current_waypoint = feedback->current_waypoint;
        RCLCPP_INFO(this->get_logger(), "Navigating towards waypoint %u...", current_waypoint);

        // 到着したウェイポイントのインデックスを計算
        int reached_waypoint_index = current_waypoint - 1;

        // 有効なインデックスで、かつまだ処理していない場合のみアクションを実行
        if (reached_waypoint_index >= 0 && reached_waypoint_index != last_processed_waypoint_index_)
        {
            on_waypoint_reached(reached_waypoint_index);
            last_processed_waypoint_index_ = reached_waypoint_index; // 処理済みとして記録
        }
    }

    void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                // 最後のウェイポイントでのアクションを実行
                if (!waypoints_.empty()) {
                    int last_waypoint_index = waypoints_.size() - 1;
                    // feedbackでまだ処理されていなければ処理する
                    if (last_waypoint_index != last_processed_waypoint_index_) {
                        on_waypoint_reached(last_waypoint_index);
                    }
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        
        RCLCPP_INFO(this->get_logger(), "Result: Missed waypoints: %zu", result.result->missed_waypoints.size());
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SavedWpFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}