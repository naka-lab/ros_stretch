#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PointStamped, Twist
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
from nav_msgs.msg import Odometry   
import math
import time
import tf
import yaml
import numpy as np


def move_robot( lift=None, length=None, wrist=None, pan=None, tilt=None, gripper=None ):
    joint_names = []
    positions = []

    # 指定された関節だけを動かす
    if lift != None:
        joint_names.append( "joint_lift" )
        positions.append( lift )

    if length != None:
        joint_names += ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"]
        positions += [ length/4 for _ in range(4) ]

    if wrist != None:
        joint_names.append( "joint_wrist_yaw" )
        positions.append(wrist)

    if pan != None:
        joint_names.append( "joint_head_pan" )
        positions.append(pan)

    if tilt != None:
        joint_names.append( "joint_head_tilt" )
        positions.append(tilt)
        
    if gripper != None:
        joint_names.append('joint_gripper_finger_left')
        positions.append(gripper)

    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(1.0)
    point.positions = positions

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = joint_names

    trajectory_goal.trajectory.points = [point]
    trajectory_goal.trajectory.header.stamp = rospy.Time.now()
    trajectory_client.send_goal(trajectory_goal)

    # wait_for_resultが機能してない・・・？
    time.sleep(4)
    trajectory_client.wait_for_result()


def transform( camx, camy, camz ):
    tf_listener.waitForTransform( "base_link" , "camera_depth_optical_frame", rospy.Time(0), rospy.Duration(5)  )

    pos_from_cam = PointStamped()
    pos_from_cam.header.frame_id = "camera_depth_optical_frame"
    pos_from_cam.point.x = camx
    pos_from_cam.point.y = camy
    pos_from_cam.point.z = camz

    pos_trans = tf_listener.transformPoint( "base_link", pos_from_cam )

    return pos_trans.point.x, pos_trans.point.y, pos_trans.point.z 

def move_vel( linear, rot ):
    t = Twist()
    t.linear.x = linear
    t.angular.z = rot
    pub_cmdvel.publish( t )

def get_current_rot():
    old_odom = rospy.wait_for_message( "odom", Odometry )
    q = old_odom.pose.pose.orientation
    rot = tf.transformations.euler_from_quaternion( [q.x, q.y, q.z, q.w] )[2]

    return rot

# 角度を-pi〜+piの範囲に収める
def normalize_angle( rot ):
    while 1:
        if rot>math.pi:
            rot -= 2 * math.pi
        elif rot<-math.pi:
            rot += 2 * math.pi
        else:
            return rot

def rotate( theta ):
    # 回転方向決定
    if theta<-0.01:
        rot = -0.1
    elif theta>0.01:
        rot = 0.1
    else:
        return 

    # 初期角度
    init_rot = get_current_rot()

    while 1:
        cur_rot = get_current_rot()
        diff = normalize_angle(cur_rot-init_rot)

        move_vel( 0, rot )

        print(init_rot, cur_rot, diff, theta)
        if theta>0 and diff>theta:
            break

        if theta<0 and diff<theta:
            break

    move_vel( 0, 0 )


def main():
    # 初期位置へ移動
    move_robot( length=0, wrist=2.5, pan=-1.5, tilt=-0.5 )
    move_robot( lift=0.2 )

    # ターゲットとなる物体がカメラに映るのを待つ
    target_cam = None
    while not rospy.is_shutdown():
        data = rospy.wait_for_message( "/ar_marker_rec/object_info", String )
        obj_info = yaml.safe_load(data.data)

        if len(obj_info)==0:
            print("物体なし")
            continue
        elif obj_info:
            for o in obj_info:
                if o["label"]==0:
                    print("物体発見")
                    target_cam = o["position"]
                    break

        if target_cam!=None:
            break
    print(target_cam)

    target_rob = transform( target_cam[0], target_cam[1], target_cam[2] )


    print(target_rob)

    # 回転角度
    theta = math.atan2( target_rob[1], target_rob[0] )+math.pi/2

    print( theta )
    rotate( theta )

    # 腕を上げる
    print("腕を上げる")
    move_robot( lift=target_rob[2], wrist=0.0 )

    # ハンドを開く
    print("ハンドを開く")
    move_robot( gripper=0.139 )

    # 腕を伸ばす
    print("腕を伸ばす")
    offset_length = 0.33
    move_robot( length=-target_rob[1]-offset_length )

    # ハンドを閉じる
    print("ハンドを閉じる")
    move_robot( gripper=-0.2 )

    # 腕を縮める
    print("腕を縮める")
    move_robot( length=0 )

    time.sleep(5)

if __name__=="__main__":
    rospy.init_node("test")
    pub_cmdvel = rospy.Publisher( "/stretch/cmd_vel", Twist, queue_size=1, latch=True)
    trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    trajectory_client.wait_for_server(timeout=rospy.Duration(60.0))
    tf_listener = tf.TransformListener()

    main()