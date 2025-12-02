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
                'start_waypoint_index': 0, # ナビゲーションを開始するwaypoint
                'manual_stop_indices': [500, 1700], # 入力待ちするwaypointのindex
                'stop_area': ["100-300"] # mid70で障害物停止する区間
            }]
        ),
    ])