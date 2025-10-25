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
            executable='waypoint_publisher', 
            name='waypoint_publisher',
            output='screen',
            # emulate_tty=True, # Terminalの標準入力を受け付ける
            prefix='gnome-terminal --', # 新しい端末を立ち上げる（Enterをうけとるため）
            parameters=[{
                'waypoint_file_path': LaunchConfiguration('waypoint_file'),
                'stop_indices': [1,3]
            }]
        ),
    ])