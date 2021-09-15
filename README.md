# Stretch



## ROSのインストール: ROS melodic (Ubuntu18.04)
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

## 実機

### [ハードウェアの使い方等](hardware.md)


### セットアップ
config内の以下のファイルを置き換える（すでに置き換えていたら不要）  
[base_local_planner_params.yaml](config/base_local_planner_params.yaml)  
[common_costmap_params.yaml](config/common_costmap_params.yaml)  
[local_costmap_params.yaml](config/local_costmap_params.yaml)  

### 実行前の準備
- 遠隔操作プログラムを終了する
  ```
  pkill -f stretch_xbox*
  ```

- Stretchドライバーを起動する
  ```
  roslaunch stretch_core stretch_driver.launch
  ```

- ナビゲーションモード（走行可能）にする
  ```
  rosservice call /stretch_to_navigation_mode “{}”
  ```

### SLAM

- マップ生成を開始する
  ```
  roslaunch stretch_navigation mapping.launch
  ```
  rviz上のマップが更新されない場合は、Fixed Frame を base_linkかlaserにする

- 別のターミナルを開く
- 作成したマップを保存する
  ```
  rosrun map_server map_saver –f test1
  ```

### ナビゲーション
- 自律走行プログラムを実行する
  ```
  roslaunch stretch_navigation navigation.launch map_yaml:=/home/hello-robot/test1.yaml
  ```

## シミュレーション
### Stretchパッケージのインストール
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

- move itのインストール(パッケージが入ってなかった場合)
  ```
  sudo apt install ros-melodic-moveit
  ```

### 台車の速度制御とアーム制御（シミュレーション）

```
roslaunch stretch_gazebo gazebo.launch
roslaunch stretch_moveit_config demo_gazebo.launch
```
- アームの制御：[stretch_arm.py](scripts/stretch_arm.py)
- 物体把持動作：[stretch_grasp_object.py](scripts/stretch_grasp_object.py)

### SLAM（シミュレーション）
- Gazenboを起動
  ```
  export GAZEBO_MODEL_PATH=~/catkin_ws/src/ros_stretch/models:${GAZEBO_MODEL_PATH}
  roslaunch ros_stretch gazebo_stretchworld.launch
  ```
  （1行目はモデルファイルの場所を指定している．bashrcに書いておけば省略可能．）

- キーボード操作でMAP作成
  ```
  roslaunch ros_stretch sim_mapping.launch
  roslaunch stretch_gazebo teleop_keyboard.launch
  ```
  **teleop_keyboard.launchが立ち上がらない場合は以下のコマンドを実行**
  ```
  roslaunch ros_stretch teleop_keyboard.launch
  ```

- MAP保存
  ```
  rosrun map_server map_saver -f ~/stretchmap
  ```

### Navigation（シミュレーション）

- SALMの1で起動したlaunch以外は落として以下を実行
  ```
  roslaunch ros_stretch sim_navigation.launch map_yaml:=$HOME/stretchmap.yaml
  ```
  すでに作成済みのマップ`map_yaml:=$HOME/catkin_ws/src/ros_stretch/map/stretchmap.yaml`も使用可能．
  （sim_navigation.launchでは，「base_local_planner_params.yaml, common_costmap_params.yaml, local_costmap_params.yaml」のデフォルトファイルだとうまく動作しなかったため，独自に書き換えた[config](config)内のファイルを利用している．）

- この状態でrvizから移動させることが可能．プログラムから移動させたい場合は，[サンプル](scripts/stretch_navigation.py)を参照
