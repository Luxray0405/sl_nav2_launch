from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # パッケージの共有ディレクトリへのパスを取得
    pkg_share = get_package_share_directory('sl_nav2_launch')

    # デフォルトのYAMLファイルのパスを設定
    default_yaml_path = os.path.join(pkg_share, 'paths', 'uec_west.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=default_yaml_path,
            description='Path to the waypoint YAML file.'
        ),

        # waypoint_publisherノードを起動
        Node(
            package='sl_nav2_launch',
            executable='waypoint_publisher_navthrough_single', 
            name='waypoint_publisher_navthrough_single',
            output='screen',
            parameters=[{
                'waypoint_file_path': LaunchConfiguration('waypoint_file'),
                'start_waypoint_index': 0,              # ナビゲーションを開始するwaypoint
                'manual_stop_indices': [84,87,90,104,105,106,108], # 入力待ちするwaypointのindex
                'stop_area': ["101-102"] # mid70で障害物停止する区間  
            }]
        ),
    ])


# denchare_long 
# 'manual_stop_indices': [5, 6, 17, 18],    # 入力待ちするwaypointのindex
# 'stop_area': ["4-6", "16-18"]             # mid70で障害物停止する区間

# tsukuba_kakunin
# 'manual_stop_indices': [82],  # 入力待ちするwaypointのindex
# 'stop_area': ["83-85"]           # mid70で障害物停止する区間

# tsukuba
# 'manual_stop_indices': [58,61,62,75,76,77,79,127,171,183,184,185,187,202,203], # 入力待ちするwaypointのindex
# 'stop_area': ["74-76","182-184"] # mid70で障害物停止する区間

# tsukuba_merged
# 'manual_stop_indices': [84,87,89,102,103,104,106], # 入力待ちするwaypointのindex
# 'stop_area': ["101-102"] # mid70で障害物停止する区間 
# # 最初の横断歩道前：80くらい