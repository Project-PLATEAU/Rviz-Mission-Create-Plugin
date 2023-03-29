# Rviz-Mission-Create-Plugin起動方法

以下コマンドによりRvizを立ち上げます。このコマンド中ではpluginを実行するために必要なconfigファイルを読み込む操作が自動的に行われます。
```
roslaunch waypoint_navigation_plugin create_way_points.launch
```
Rvizが正常に起動すると以下のような画面が現れます。

<img src="../resources/Usage/default.jpg" width="80%">

また、明示的にconfigファイルを読み込ませる方法でRvizを起動する方法は、以下のコマンドを別ターミナルで順番に実行します。
```
roscore
```
```
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_point map 20
```
```
rviz -d autonomous.rviz
```


