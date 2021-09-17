#! /usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def move_arm( lift=None, length=None, wrist=None ):
    joint_names = []
    positions = []

    # 指定された関節だけを動かす
    if lift:
        joint_names.append( "joint_lift" )
        positions.append( lift )

    if length:
        joint_names += ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"]
        positions += [ length/4 for _ in range(4) ]

    if wrist:
        joint_names.append( "joint_wrist_yaw" )
        positions.append(wrist)

    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(1.0)
    point.positions = positions
    print(point)

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = joint_names

    trajectory_goal.trajectory.points = [point]
    trajectory_goal.trajectory.header.stamp = rospy.Time.now()
    trajectory_client.send_goal(trajectory_goal)
    trajectory_client.wait_for_result()

if __name__=="__main__":
    rospy.init_node("test")
    trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    trajectory_client.wait_for_server(timeout=rospy.Duration(60.0))

    # 高さだけ変える
    move_arm( 0.6 )

    # 高さ，長さ，ハンド角度を変える
    move_arm( 0.6, 0.1, 0.0 )


