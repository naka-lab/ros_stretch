#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PointStamped, Twist
from std_msgs.msg import String
import math
import time
import tf
import yaml
import numpy as np

def transform_to_relativepos( wldx, wldy, wldz ):
    tf_listener.waitForTransform( "base_link" , "odom", rospy.Time(0), rospy.Duration(5)  )

    pos_from_cam = PointStamped()
    pos_from_cam.header.frame_id = "odom"
    pos_from_cam.point.x = wldx
    pos_from_cam.point.y = wldy
    pos_from_cam.point.z = wldz

    pos_trans = tf_listener.transformPoint( "base_link", pos_from_cam )

    return pos_trans

def move_vel( linear, rot ):
    t = Twist()
    t.linear.x = linear
    t.angular.z = rot
    pub_cmdvel.publish( t )

def control_vel( target_x, target_y, enable_linesar=True, enable_rot=True, rot_offset=0, rot_thresh=0.02, dist_threash=0.7 ):
    # 使わない条件はしきい値を大きくして無効にする
    if not enable_linesar:
        dist_threash = 99999999999
    if not enable_rot:
        rot_thresh = 999999999999


    while not rospy.is_shutdown():
        # 相対位置を計算
        rel_pos = transform_to_relativepos(target_x, target_y, 0 )

        # 角度と距離を計算
        theta = math.atan2( rel_pos.point.y, rel_pos.point.x ) + rot_offset
        dist = math.sqrt( rel_pos.point.y**2 + rel_pos.point.x**2 )


        if  -rot_thresh<theta<rot_thresh and dist<dist_threash:
            print("回転・直進停止")
            move_vel( 0, 0)
            break

        # 距離と角度に応じて比例制御する
        linear = 0.3 * min( max( dist , 0) , 1.0) + 0.1 if enable_linesar else 0
        rot = 0.3 *  min( max( theta , -1.0) , 1.0) + 0.1*np.sign(theta) if enable_rot else 0

        move_vel( linear, rot )


def approach_to_graspable_position( target_x, target_y ):
    # 目標位置まで比例制御（本来はマップを使ったプランニング）
    # 回転だけで方向を合わせる
    control_vel( target_x, target_y, enable_linesar=False )

    # 直進+回転で近づく
    control_vel( target_x, target_y )

    # アームをtargetに向ける
    control_vel( target_x, target_y, enable_linesar=False, rot_offset=math.pi/2 )
    return


def grasp_object( target_x, target_y, target_z ):

    # griperを開く    
    gripper.set_joint_value_target([0.5, 0.5])
    gripper.go()

    # 相対位置を計算
    rel_pos = transform_to_relativepos(target_x, target_y, 0 )
    joint_len = (math.sqrt( rel_pos.point.y**2 + rel_pos.point.x**2 )-0.25)/4

    # アームを上げる
    arm.set_joint_value_target( [target_z-0.15, 0, 0, 0, 0, 0] )
    arm.go()

    # アームを伸ばす
    arm.set_joint_value_target( [target_z-0.15, joint_len, joint_len, joint_len, joint_len, 0] )
    arm.go()

    # gripperを閉じる
    gripper.set_joint_value_target([0, 0])
    gripper.go()

    # アームを引く
    arm.set_joint_value_target( [target_z-0.15, 0, 0, 0, 0, 0] )
    arm.go()

    # アームを下ろす
    arm.set_joint_value_target( [0, 0, 0, 0, 0, 0] )
    arm.go()

def main():
    arm.set_joint_value_target( [0, 0, 0, 0, 0, 0] )
    arm.go()

    # ターゲットとなる物体がカメラに映るのを待つ
    target = None
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
                    target = o["position"]
                    break

        if target!=None:
            break

    print( transform_to_relativepos(target[0], target[1], target[2]) )

    # 物体位置まで移動
    approach_to_graspable_position( target[0], target[1] )

    # 掴む
    grasp_object( target[0], target[1], target[2] )
    return 



if __name__ == '__main__':
    rospy.init_node("stretch_arm")
    pub_cmdvel = rospy.Publisher( "/stretch_diff_drive_controller/cmd_vel", Twist, queue_size=1, latch=True)

    arm = moveit_commander.MoveGroupCommander("stretch_arm")
    gripper = moveit_commander.MoveGroupCommander("stretch_gripper")
    head = moveit_commander.MoveGroupCommander("stretch_head")

    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(1.0)
    head.set_max_velocity_scaling_factor(1.0)
    head.set_max_acceleration_scaling_factor(1.0)
    tf_listener = tf.TransformListener()

    main()

