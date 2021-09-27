pkill -f stretch_xbox*
python ~/catkin_ws/src/ros_stretch/scripts/swicth_to_navigation_mode.py &
roslaunch stretch_core stretch_driver.launch
