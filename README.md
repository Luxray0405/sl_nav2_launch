# sl_nav2_launch

salamander用nav2起動パッケージ  
バイナリ/ソースインストールされたnav2を起動するためのもの。


## 起動

rvizの起動+launch。自己位置推定も含めた全体の手順は下に書いてあります。
```
rviz2 -d ~/ros2_ws/src/sl_nav2_launch/rviz/nav2_default_view.rviz
ros2 launch sl_nav2_launch sl_nav2_launch.py
```

## パラメータ調整
params/nav2_params.yamlを編集

### mapファイルの指定方法
```
map_server:
  ros__parameters:
    yaml_filename: "/home/ueno/map_data/uec_250819/uec_250819.yaml"
```
のyaml_filenameでファイルパスを指定。


## ディレクトリ構成

### launch
起動するlaunchファイル。  
sl_nav2_launchではnav2のナビゲーション機能一連を起動。  
waypoint_saver_launchはrviz上でwaypointを指定・保存するノードを起動。  
waypoint_publisher_launchは保存したwaypointファイルからnav2にゴール情報を送るノードを起動。


### maps
2Dmapファイル。必ずpgmとyamlをセットで同じ階層に入れる。

### params
パラメータファイル。設定は主にここのファイルをいじる。

### paths
作成したwaypoint列を保存したファイル。

### rviz
rvizファイル。

### src
waypoint_saver.cpp：waypoint作成・保存ノード。  
waypoint_publisher.cpp：waypoint送信ノード。


## 自律移動の全体の手順

### 自己位置推定
```
rviz2 -d ~/ros2_ws/src/sl_nav2_launch/rviz/nav2_default_view.rviz
ros2 launch lidar_localization_ros2 lidar_localization.launch.py
```
LiDAR・IMUを起動・接続するか、rosbagを再生してLiDARとIMUのトピックをpublishすると自己位置推定が始まり、tfが発行される。  
rosbag起動の際は、
```
ros2 bag play file_name --clock
```
のように--clockをつけること。

### ナビゲーション
```
ros2 launch sl_nav2_launch sl_nav2_launch.py
```
でnav2の機能一式が起動。rosbagを再生している場合は、
```
ros2 launch sl_nav2_launch sl_nav2_launch.py use_sim_time:=true
```
のようにする。

## 保存済みwaypointの使用方法

[waypoint editor](https://github.com/kzm784/waypoint_editor)などを用いて作成したwaypointファイルを指定する方法。  
ナビゲーションの準備が終わった段階で起動すると、そのままwaypointへの追従を開始する。  
csvファイルを指定。ファイルパスは自分の環境に書き換えてください。  
(上記のwaypoint editorを使用した場合、waypointがcsvファイルとして作成される)
```
ros2 launch sl_nav2_launch waypoint_publisher_launch.py waypoint_file:=/home/ueno/ros2_ws/src/sl_nav2_launch/paths/uec_250819_path.csv 
```
nav_throughモードを用いる場合
```
ros2 launch sl_nav2_launch waypoint_publisher_navthrough_launch.py waypoint_file:=/home/ueno/ros2_ws/src/sl_nav2_launch/paths/uec_250819_path.yaml 
```

nav_throughモードで一点ずつ送信する場合
```
ros2 launch sl_nav2_launch waypoint_publisher_navthrough_single_launch.py waypoint_file:=/home/ueno/ros2_ws/src/sl_nav2_launch/paths/uec_250819_path.yaml 
```

入力待ちになったら、別ターミナルで以下を実行。  
ゴール送信が再開される。
```
ros2 topic pub /resume_waypoint std_msgs/msg/Empty {} --once
```