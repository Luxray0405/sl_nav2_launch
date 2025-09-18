# sl_nav2_launch

salamander用nav2起動パッケージ
バイナリ/ソースインストールされたnav2を起動するためのもの。


## 起動

rvizの起動+launch
```
rviz2 -d ~/ros2_ws/src/sl_nav2_launch/rviz/nav2_default_view.rviz
ros2 launch sl_nav2_launch sl_nav2_launch.py use_sim_time:=true
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
saved_wp_follower_launchは保存したwaypointファイルからnav2にゴール情報を送るノードを起動。


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
saved_wp_follower.cpp：waypoint送信ノード。


