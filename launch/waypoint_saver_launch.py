import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():

    # パッケージの共有ディレクトリへのパスを取得
    pkg_dir = get_package_share_directory('sl_nav2_launch')

    # 地図ファイルパスを受け取るための引数を宣言
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_dir, 'maps', 'uec_250719.yaml'),
        description='Full path to the map YAML file to load'
    )

    # ウェイポイント保存先パスを受け取るための引数を宣言
    declare_waypoints_save_file_arg = DeclareLaunchArgument(
        'waypoints_save_file',
        default_value=os.path.expanduser('~/my_route.yaml'),
        description='Full path to save the waypoints YAML file'
    )

    map_yaml_path = LaunchConfiguration('map_file')
    waypoint_save_path = LaunchConfiguration('waypoints_save_file')

    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'waypoint_saver.rviz')

    # 1. Map Server ノードの起動設定
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_path}],
        # output='screen'
    )

    # Map Serverをアクティブにするためのライフサイクルマネージャ
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        # output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # 2. RViz2 ノードの起動設定
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        # output='screen'
    )

    # 3. Waypoint Saver ノードの起動設定
    waypoint_saver_node = Node(
        package='sl_nav2_launch',
        executable='waypoint_saver',
        name='waypoint_saver',
        output='screen',
        prefix='gnome-terminal --',
        parameters=[{'save_path': waypoint_save_path}]
    )

    # LaunchDescriptionに各ノードを追加して返す
    return LaunchDescription([
        declare_map_file_arg,
        declare_waypoints_save_file_arg,
        
        map_server_node,
        lifecycle_manager_node,
        rviz2_node,
        waypoint_saver_node
    ])