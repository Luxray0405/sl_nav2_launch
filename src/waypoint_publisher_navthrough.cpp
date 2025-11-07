#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>

class WaypointPublisher : public rclcpp::Node
{
public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    explicit WaypointPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("waypoint_publisher", options)
    {
        // パラメータを宣言し、ファイルパスを取得
        this->declare_parameter<std::string>("waypoint_file_path", "default_path.yaml");
        this->declare_parameter<std::vector<int64_t>>("manual_stop_indices", std::vector<int64_t>{});
        this->declare_parameter<std::vector<int64_t>>("auto_stop_indices", std::vector<int64_t>{});
        std::string waypoint_file_path = this->get_parameter("waypoint_file_path").as_string();
        auto manual_stops_long = this->get_parameter("manual_stop_indices").as_integer_array();
        auto auto_stops_long = this->get_parameter("auto_stop_indices").as_integer_array();      

        // 手動停止リストをセットに保存 (int64_t -> int)
        for (int64_t idx : manual_stops_long) {
            manual_stops_set_.insert(static_cast<int>(idx));
        }

        // 両方のリストを stop_indices_ (std::vector<int>) にマージ
        for (int idx : manual_stops_set_) {
            stop_indices_.push_back(idx);
        }
        for (int64_t idx : auto_stops_long) {
            // 重複を避けるため、manual_stops_set_ にないものだけ追加
            if (manual_stops_set_.find(static_cast<int>(idx)) == manual_stops_set_.end()) {
                stop_indices_.push_back(static_cast<int>(idx));
            }
        }
         // 順番通りに処理するためソート
        std::sort(stop_indices_.begin(), stop_indices_.end());

        // アクションクライアント、Publisher、Service Clientを作成
        this->action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "/navigate_through_poses");
        event_publisher_ = this->create_publisher<std_msgs::msg::String>("/waypoint_event", 10);
        dummy_service_client_ = this->create_client<std_srvs::srv::Empty>("/trigger_action");
        start_motion_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/start_motion", 10);

        // --- 全ウェイポイントを読み込んでメンバー変数に保存 ---
        RCLCPP_INFO(this->get_logger(), "Loading all waypoints...");
        this->all_waypoints_ = load_waypoints(waypoint_file_path);

        if (this->all_waypoints_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No waypoints loaded. Shutting down.");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting. Shutting down.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Action server found. Ready to send segments.");
    }

    ~WaypointPublisher()
    {
        // ノードが破棄される（シャットダウンする）ときに呼ばれる
        
        // goal_active_ フラグがtrueの場合、まだゴールが実行中のはず
        if (goal_active_.load()) 
        {
            RCLCPP_INFO(this->get_logger(), "Node shutting down. Attempting to cancel active goal...");
            
            // 安全のためミューテックスで保護
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            
            if (active_goal_handle_) 
            {
                // アクティブなゴールハンドルが存在すれば、非同期でキャンセルを送信
                action_client_->async_cancel_goal(active_goal_handle_);
                RCLCPP_INFO(this->get_logger(), "Cancel request sent.");
            } 
            else 
            {
                RCLCPP_WARN(this->get_logger(), "Goal was active, but no valid handle was found to cancel.");
            }
        }
    }

    /**
     * @brief [PUBLIC] ノードがユーザーの入力待ち状態かを確認する
     */
    bool is_waiting_for_input() const
    {
        return waiting_for_input_.load();
    }

    /**
     * @brief [PUBLIC] 次のウェイポイントセグメント（区間）を送信する
     */
    void send_next_segment()
    {
        if (goal_active_.load() || all_segments_sent_.load()) {
            RCLCPP_WARN(this->get_logger(), "Goal is already active or all segments sent. Ignoring request.");
            return;
        }
        
        waiting_for_input_ = false; // 入力待ち状態を解除

        // 1. 送信するウェイポイントの範囲を決定
        int start_idx = current_segment_start_index_;
        int end_idx;
        int max_valid_index = static_cast<int>(all_waypoints_.size()) - 1; // [追加] 最大インデックス

        if (stop_index_tracker_ < stop_indices_.size()) {
            // 次の停止位置が指定されている場合
            // stop_indicesのstop_index_tracker番目の値をend_idxに代入
            end_idx = stop_indices_[stop_index_tracker_];
            stop_index_tracker_++;
        } else {
            // これが最後のセグメント（指定された停止位置の残りがない）
            end_idx = max_valid_index; 
            all_segments_sent_ = true; // フラグを立てる
        }

        // 2. 範囲のバリデーション
        if (end_idx > max_valid_index) {
            RCLCPP_WARN(this->get_logger(), 
                "Stop index %d is out of bounds (max is %d). Truncating segment to max index.",
                end_idx, max_valid_index);
            end_idx = max_valid_index; // 超えていたら丸める
            
            // 本来のstop_indicesではこれが最後でなくても、
            // 最大インデックスに達したので「全セグメント送信済み」とする
            all_segments_sent_ = true; 
            
            // stop_indices_リストの残りを無視する
            stop_index_tracker_ = stop_indices_.size(); 
        }

        if (start_idx > end_idx || start_idx > max_valid_index){
            RCLCPP_INFO(this->get_logger(), "All waypoint segments have been processed.");
            rclcpp::shutdown();
            return;
        }

        // 3. ウェイポイントのセグメントを作成
        std::vector<geometry_msgs::msg::PoseStamped> segment_waypoints;
        for (int i = start_idx; i <= end_idx; ++i) {
            segment_waypoints.push_back(all_waypoints_[i]);
        }
        // 実行中セグメントの開始index
        active_segment_start_index_ = start_idx;
        // 次のセグメントのために開始インデックスを更新
        current_segment_start_index_ = end_idx + 1;
        // 現在のセグメントの終了インデックスを（コールバック用に）記録
        current_segment_end_index_ = end_idx; 

        RCLCPP_INFO(this->get_logger(), "Sending segment from index %d to %d (%zu waypoints).",
                    start_idx, end_idx, segment_waypoints.size());

        // 4. ゴールを送信
        auto goal_msg = NavigateThroughPoses::Goal();
        goal_msg.poses = segment_waypoints;

        auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WaypointPublisher::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&WaypointPublisher::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&WaypointPublisher::result_callback, this, std::placeholders::_1);
        
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_active_ = true; // 実行中フラグを立てる
    }

private:
    // ROS 2関連のメンバー変数
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr dummy_service_client_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_motion_publisher_;
    
    // --- ウェイポイントと状態管理 ---
    std::vector<geometry_msgs::msg::PoseStamped> all_waypoints_; // 全ウェイポイント
    std::vector<int> stop_indices_;          // 停止するインデックスのリスト
    size_t stop_index_tracker_ = 0;          // stop_indices_の何番目まで処理したか
    int current_segment_start_index_ = 0;    // 次に送信するセグメントの開始インデックス
    int current_segment_end_index_ = -1;   // 送信したセグメントの終了インデックス（グローバル）
    int active_segment_start_index_ = 0;   // 実行中のセグメントの開始インデックス
    std::atomic<bool> waiting_for_input_ = {false}; // ターミナルの入力待ちか
    std::atomic<bool> goal_active_ = {false};       // アクションが実行中か
    std::atomic<bool> all_segments_sent_ = {false}; // 最後のセグメントを送信済みか
    std::set<int> manual_stops_set_;           // 手動停止（入力待ち）するインデックスを保持するセット

    int last_processed_waypoint_index_ = -1; // on_waypoint_reached処理済みのインデックス
    
    std::mutex goal_handle_mutex_;
    GoalHandleNavigateThroughPoses::SharedPtr active_goal_handle_ = nullptr;

    // ファイル拡張子に基づいて適切なローダーを呼び出す関数
    std::vector<geometry_msgs::msg::PoseStamped> load_waypoints(const std::string& file_path)
    {
        std::string extension = file_path.substr(file_path.find_last_of(".") + 1);
        if (extension == "yaml" || extension == "yml") {
            return load_waypoints_from_yaml(file_path);
        } else if (extension == "csv") {
            return load_waypoints_from_csv(file_path);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported file format: %s", extension.c_str());
            return {};
        }
    }
    
    // CSVファイルからウェイポイントをロードする関数 (新規追加)
    std::vector<geometry_msgs::msg::PoseStamped> load_waypoints_from_csv(const std::string& file_path)
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        std::ifstream file(file_path);

        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Waypoint file not found at: %s", file_path.c_str());
            return waypoints;
        }

        RCLCPP_INFO(this->get_logger(), "Loading waypoints from CSV file: %s", file_path.c_str());
        std::string line;

        // ヘッダー行をスキップ
        if (!std::getline(file, line)) {
            RCLCPP_ERROR(this->get_logger(), "Cannot read header line from CSV file.");
            return waypoints;
        }

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;

            while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
            }
            
            // 必要な列数があるか確認 (id, pose_x~rot_w)
            if (row.size() < 8) continue; 

            geometry_msgs::msg::PoseStamped ps;

            // ヘッダー情報を設定
            ps.header.frame_id = "map"; // CSVにはないので固定値
            ps.header.stamp = this->get_clock()->now();

            // CSVから読み込んだ値を設定
            ps.pose.position.x = std::stod(row[1]);
            ps.pose.position.y = std::stod(row[2]);
            ps.pose.position.z = std::stod(row[3]);
            ps.pose.orientation.x = std::stod(row[4]);
            ps.pose.orientation.y = std::stod(row[5]);
            ps.pose.orientation.z = std::stod(row[6]);
            ps.pose.orientation.w = std::stod(row[7]);

            waypoints.push_back(ps);
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints from CSV.", waypoints.size());
        return waypoints;
    }

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

        RCLCPP_INFO(this->get_logger(), "Loading waypoints from YAML: %s", file_path.c_str());
        
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
            RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints from YAML.", waypoints.size());
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error while parsing YAML file: %s", e.what());
        }

        return waypoints;
    }

    // // アクションゴールを送信する関数
    // void send_goal(const std::string& file_path)
    // {
    //     // 読み込んだウェイポイントをメンバー変数に保存 (load_waypointsを呼び出すように変更)
    //     this->waypoints_ = load_waypoints(file_path);
    //     if (this->waypoints_.empty()) {
    //         RCLCPP_ERROR(this->get_logger(), "No waypoints to send. Shutting down.");
    //         rclcpp::shutdown();
    //         return;
    //     }

    //     if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    //         RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting. Shutting down.");
    //         rclcpp::shutdown();
    //         return;
    //     }

    //     auto goal_msg = NavigateThroughPoses::Goal();
    //     goal_msg.poses = this->waypoints_; // 保存したメンバー変数を使用

    //     RCLCPP_INFO(this->get_logger(), "Sending goal request...");
        
    //     auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    //     send_goal_options.goal_response_callback =
    //         std::bind(&WaypointPublisher::goal_response_callback, this, std::placeholders::_1);
    //     send_goal_options.feedback_callback =
    //         std::bind(&WaypointPublisher::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    //     send_goal_options.result_callback =
    //         std::bind(&WaypointPublisher::result_callback, this, std::placeholders::_1);
        
    //     this->action_client_->async_send_goal(goal_msg, send_goal_options);
    // }

    // ウェイポイント到着時に実行されるアクションを定義する関数
    void on_waypoint_reached(int reached_waypoint_index)
    {
        RCLCPP_INFO(this->get_logger(), "======== Waypoint %d Reached! ========", reached_waypoint_index);

        // 最後のウェイポイントのインデックスを動的に取得
        int last_waypoint_index = all_waypoints_.size() - 1;

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
    void goal_response_callback(const GoalHandleNavigateThroughPoses::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Segment Goal was rejected by server");
            goal_active_ = false; // 失敗したのでフラグを下ろす
            waiting_for_input_ = true; // 再試行のために待機状態にする (あるいはシャットダウン)
            active_goal_handle_ = nullptr;
        } else {
            RCLCPP_INFO(this->get_logger(), "Segment Goal accepted by server.");
            active_goal_handle_ = goal_handle;
        }
    }

    void feedback_callback(
        GoalHandleNavigateThroughPoses::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
        "Feedback: Distance remaining to segment goal: %.2f m, Nav time: %.1f s", 
        feedback->distance_remaining, 
        rclcpp::Duration(feedback->navigation_time).seconds());
    }

    void result_callback(const GoalHandleNavigateThroughPoses::WrappedResult & result)
    {
        { // [追加] ロックのためのスコープ
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            active_goal_handle_ = nullptr; // [追加] ゴールが終了したのでクリア
        }
        goal_active_ = false; // アクションが完了したのでフラグを下ろす

        // どのウェイポイントに到着したか（セグメントの最後のインデックス）
        int reached_index = current_segment_end_index_; 

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Segment (up to index %d) succeeded!", reached_index);

                // 到着時アクションを実行
                // if (reached_index != last_processed_waypoint_index_) {
                //     on_waypoint_reached(reached_index);
                //     last_processed_waypoint_index_ = reached_index;
                // }

                if (all_segments_sent_.load()) {
                    // これが最後のセグメントだった場合
                    RCLCPP_INFO(this->get_logger(), "All segments completed successfully!");
                    rclcpp::shutdown(); // ノードを終了
                } else {
                    // まだ次のセグメントがある場合
                    // 停止したインデックスが、手動停止リストに含まれているか確認
                    if (manual_stops_set_.count(reached_index)) 
                    {
                        // 手動停止リストに含まれていた場合 -> 入力待ち
                        RCLCPP_INFO(this->get_logger(), "Robot stopped at MANUAL index %d. Waiting for input...", reached_index);
                        waiting_for_input_ = true; // mainループに入力待ちを通知
                    } 
                    else 
                    {
                        // 自動停止リストだった場合 -> 即座に次を送信
                        RCLCPP_INFO(this->get_logger(), "Robot at AUTO index %d. Sending next segment...", reached_index);
                        send_next_segment(); // 次のセグメントを自動で送信
                    }
                }
                break;
                
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Segment Goal was aborted");
                rclcpp::shutdown(); // 失敗したら終了
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Segment Goal was canceled");
                rclcpp::shutdown(); // 失敗したら終了
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                rclcpp::shutdown(); // 失敗したら終了
                break;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // ノードを初期化
    auto node = std::make_shared<WaypointPublisher>();

    // Executor（コールバック処理機）を作成
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // 最初のセグメントを送信
    // （アクションサーバへの接続待機はコンストラクタで行っている）
    RCLCPP_INFO(node->get_logger(), "Sending first segment...");
    node->send_next_segment();

    // カスタム実行ループ
    while (rclcpp::ok())
    {
        if (node->is_waiting_for_input())
        {
            // --- 入力待ち状態 ---
            RCLCPP_INFO(node->get_logger(), " "); // 改行
            RCLCPP_INFO(node->get_logger(), "=== PAUSED ===");
            RCLCPP_INFO(node->get_logger(), "Press [Enter] in this terminal to continue to the next segment...");
            
            // ターミナルからの入力をブロッキングで待機
            std::string line;
            std::getline(std::cin, line);

            if (!rclcpp::ok()) break; // Ctrl+Cなどで終了した場合

            RCLCPP_INFO(node->get_logger(), "=== RESUMING ===");
            
            // 入力が確認されたので、次のセグメントを送信
            node->send_next_segment();
        }
        
        // 入力待ちでない間、ROSのコールバック（フィードバック、リザルト）を処理
        executor.spin_some(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}