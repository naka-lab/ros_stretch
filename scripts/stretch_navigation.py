#!/usr/bin/env python

import rospy
import actionlib
import math
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# 場所の名前とその座標を設定
locations = {
    "kitchen" : (1.0, 0, -0.000000),
    "living" : (1.0, 1.0, 3.14/2),
    "desk" : (0.0, 1.0, 3.14),
}

def send_navi_goal(x, y, theta ):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0

    xyzw = tf.transformations.quaternion_from_euler(0, 0, theta )
    goal.target_pose.pose.orientation.x = xyzw[0]
    goal.target_pose.pose.orientation.y = xyzw[1]
    goal.target_pose.pose.orientation.z = xyzw[2]
    goal.target_pose.pose.orientation.w = xyzw[3]

    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_state()

    return result


if __name__ == '__main__':
    rospy.init_node('navigation')
    result = send_navi_goal( *locations["kitchen"] )
    print(result)

    result = send_navi_goal( *locations["living"] )
    print(result)

    result = send_navi_goal( *locations["desk"] )
    print(result)