# ROSのインストール
## ROS melodic (Ubuntu18.04)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```
sudo apt install -y curl
```

```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup? op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

```
sudo apt update
```

```
sudo apt upgrade
```

```
sudo apt install -y ros-melodic-desktop-full
```

```
sudo apt install python-rosdep
```
