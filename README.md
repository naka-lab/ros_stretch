# ROSのインストール:ROS melodic (Ubuntu18.04)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup? op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt upgrade
sudo apt install -y ros-melodic-desktop-full
sudo apt install python-rosdep
sudo -E rosdep init
rosdep update
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
sudo apt-get install ros-melodic-dwa-local-planner
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
```

# Stretchパッケージのインストール
- 公式リポジトリをClone
```
cd ~/catkin_ws/src
git clone https://github.com/hello-robot/stretch_ros
git clone https://github.com/pal-robotics/realsense_gazebo_plugin
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

- このリポジトリをClone
```
cd ~/catkin_ws/src
git clone https://github.com/naka-lab/ros_stretch.git
```

- URDFの作成
```
cd ~/catkin_ws/src/stretch_ros/stretch_description/urdf/
./xacro_to_urdf.sh
```

# 実行
## 台車の速度制御とアーム制御（シミュレーション）

```
roslaunch stretch_gazebo gazebo.launch
roslaunch stretch_moveit_config demo_gazebo.launch
```
- アームの制御：[stretch_arm.py](scripts/stretch_arm.py)
- 物体把持動作：[stretch_grasp_object.py](scripts/stretch_grasp_object.py)

## SLAM（シミュレーション）
1. Gazenboが起動したら障害物を配置する
```
roslaunch stretch_gazebo gazebo.launch
```

2. キーボード操作でMAP作成
```
roslaunch ros_stretch sim_mapping.launch
roslaunch stretch_gazebo teleop_keyboard.launch
```

3. MAP保存
```
rosrun map_server map_saver -f ~/stretchmap
```

# Navigation（シミュレーション）

- SALMの1で起動したlaunch以外は落として以下を実行
```
roslaunch ros_stretch sim_navigation.launch map_yaml:=$HOME/stretchmap.yaml
```

**navigationが上手くいかない場合は[navigation_config](navigation_config/)内にある3つのファイルを**<br>
**catkin_ws/src/stretch_ros/stretch_navigation/config内にあるファイルと置き換える**<br>
