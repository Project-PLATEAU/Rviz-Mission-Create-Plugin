# 環境構築・インストール


必要動作環境: Ubuntu18

### 依存ライブラリインストール方法
```
sudo apt install curl git
```

ROS： [ROS インストールチュートリアル](http://wiki.ros.org/melodic/Installation/Ubuntu)に従って、インストールしてください。 

```
sudo apt install -y ros-melodic-pcl-ros  ros-melodic-octomap ros-melodic-octomap-server ros-melodic-octomap-rviz-plugins ros-melodic-jsk-visualization
```

### ROS ワークスペース作成
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone git@github.com:sensyn-robotics/PLATEAU-UC22-024-Rviz-Mission-Create-Plugin.git

# 環境変数の設定
echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### ビルド
```
cd ~/catkin_ws/
catkin_make
```

