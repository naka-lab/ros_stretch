#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import math
import time
import tf

def set_pose(x, y, z, rz, ry, rx):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    x,y,z,w = tf.transformations.quaternion_from_euler(rz, ry, rx)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w

    arm.set_start_state_to_current_state()
    arm.set_pose_target(pose)

    print(arm.get_current_pose())

    ret = arm.go()

    return ret

def main():
    # armの長さで指定
    # [高さ(m)、水平方向1(m), 水平方向2(m), 水平方向3(m), 水平方向4(m), 手首の角度(rad) ]
    print("Arm joints -------")
    print(arm.get_current_joint_values()) 
    arm.set_joint_value_target( [0, 0, 0, 0, 0, 0] )
    arm.go()
    arm.set_joint_value_target( [0.5, 0.1, 0.1, 0.1, 0.1, 0.5] )
    arm.go()

    # griper: [左指の角度(rad)、右指の角度(rad)]

    print("Gripper ---------")
    print(gripper.get_current_joint_values())
    gripper.set_joint_value_target([0.5, 0.5]) # 開く
    gripper.go()
    gripper.set_joint_value_target([0, 0])  # 閉じる
    gripper.go()

    # head: [pan, tilt]
    print("Head ---------")
    print(head.get_current_joint_values())
    head.set_joint_value_target([0, 0]) # 開く
    head.go()
    
    head.set_joint_value_target([0.5, 0]) # 開く
    head.go()

    head.set_joint_value_target([0.5, 0.5]) # 開く
    head.go()

    # pose指定だと動かせない？
    print("Arm pose -------")
    print(arm.get_current_pose()) 
    set_pose(-0.02392, -0.3, 0.68, -3.14, 0, 0)


if __name__ == '__main__':
    rospy.init_node("stretch_arm")

    arm = moveit_commander.MoveGroupCommander("stretch_arm")
    gripper = moveit_commander.MoveGroupCommander("stretch_gripper")
    head = moveit_commander.MoveGroupCommander("stretch_head")

    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(1.0)

    head.set_max_velocity_scaling_factor(1.0)
    head.set_max_acceleration_scaling_factor(1.0)

    main()

