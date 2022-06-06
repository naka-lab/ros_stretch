#! /usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, unicode_literals
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk
import time

def move_robot( lift=None, length=None, wrist=None, pan=None, tilt=None, gripper=None, timeout=20 ):
    joint_names = []
    positions = []
    velocities = []
    accelerations = []


    # 指定された関節だけを動かす
    if lift != None:
        joint_names.append( "joint_lift" )
        positions.append( lift )
        velocities.append( 0.3 )
        accelerations.append( 0.3 )

    if length != None:
        joint_names += ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"]
        positions += [ length/4 for _ in range(4) ]
        velocities.append( 0.5 )
        accelerations.append( 0.5 )

    if wrist != None:
        joint_names.append( "joint_wrist_yaw" )
        positions.append(wrist)
        velocities.append( 5 )
        accelerations.append( 5 )

    if pan != None:
        joint_names.append( "joint_head_pan" )
        positions.append(pan)
        velocities.append( 5 )
        accelerations.append( 5 )

    if tilt != None:
        joint_names.append( "joint_head_tilt" )
        positions.append(tilt)
        velocities.append( 5 )
        accelerations.append( 5 )

    if gripper != None:
        joint_names.append('joint_gripper_finger_left')
        positions.append(gripper)
        velocities.append( 5 )
        accelerations.append( 5 )

    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(1.0)
    point.positions = positions
    point.velocities = velocities
    point.accelerations = accelerations

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = joint_names
    trajectory_goal.trajectory.points = [point]
    trajectory_goal.trajectory.header.stamp = rospy.Time.now()
    trajectory_client.send_goal(trajectory_goal)
    #trajectory_client.wait_for_result()

    """
    # ゴールに付いてるか確認
    threshold = {"joint_lift": 0.01, 
    "joint_arm_l0": 0.01, 
    "joint_arm_l1": 0.01, 
    "joint_arm_l2": 0.01, 
    "joint_arm_l3": 0.01, 
    "joint_wrist_yaw": 0.1, 
    "joint_head_pan": 0.1, 
    "joint_head_tilt": 0.1, 
    "joint_gripper_finger_left": 0.1  }

    start = time.time()
    while (time.time()-start)<timeout:
        cur_js = rospy.wait_for_message( "/joint_states", JointState )
        cur_pos = dict(zip(cur_js.name, cur_js.position ))

        for name, goal_pos in zip(joint_names, positions):
            diff = abs( goal_pos - cur_pos[name] )
            #print(name, "goal:%lf, cur:%lf" % (goal_pos, cur_pos[name]) , "ｰ>" ,diff<threshold[name])

            if diff>threshold[name]:
                #print("retry")
                time.sleep(0.1)
                trajectory_client.send_goal(trajectory_goal)
                #trajectory_client.wait_for_result()
                break
        else:
            break
    """

def make_gui( name, from_, to, resolution, command ):
    frame = tk.Frame(root)
    tk.Label(frame, text=name, width=15  ).pack(side="left")
    sb = tk.Scale(frame, orient=tk.HORIZONTAL, from_=from_, to=to, resolution=resolution, length=200 )
    sb.pack(side="left")
    tk.Button( frame, text='go', command=command ).pack(side="left")
    frame.pack()

    return sb

def get_current_pose( scrolls ):
    js = rospy.wait_for_message( "/joint_states", JointState )
    print(js)

    scrolls[0].set(  js.position[10] )
    scrolls[1].set(  js.position[11] )
    scrolls[2].set(  js.position[4] )
    scrolls[3].set(  js.position[5]+js.position[6]+js.position[7]+js.position[8] )
    scrolls[4].set(  js.position[9] )
    scrolls[5].set(  js.position[0] )

def send_vel( linear, rot ):
    t = Twist()
    t.linear.x = linear
    t.angular.z = rot
    pub_cmdvel.publish( t )

lin_vel = 0.0
rot_vel = 0.0
def set_vel( lin, rot ):
    global lin_vel, rot_vel
    lin_vel = lin
    rot_vel = rot
    if lin_vel==0 and rot_vel==0:
        send_vel(0, 0)

def send_navi_goal_loop():
    if lin_vel!=0 or rot_vel!=0:
        send_vel( lin_vel, rot_vel )

    root.after(100, send_navi_goal_loop)

def main():
    sb_pan = make_gui( "NECK PAN", -3.14, 1.7, 0.1, lambda : move_robot( pan=sb_pan.get() ) )
    sb_tilt = make_gui( "NECK TILT", -1.6, 0.4, 0.1, lambda : move_robot( tilt=sb_tilt.get() ) )
    sb_lift = make_gui( "ARM LIFT", 0.2, 0.9, 0.01, lambda : move_robot( lift=sb_lift.get() ) )
    sb_len = make_gui( "ARM LEN", 0, 0.51, 0.01, lambda : move_robot( length=sb_len.get() ) )
    sb_wrist = make_gui( "ARM WRIST", -1.3, 4.5, 0.1, lambda : move_robot( wrist=sb_wrist.get() ) )
    sb_gripper = make_gui( "GRIPPER", -0.37, 0.18, 0.01, lambda : move_robot( gripper=sb_gripper.get() ) )

    frame = tk.Frame(root)
    sbs = [ sb_pan, sb_tilt, sb_lift, sb_len, sb_wrist, sb_gripper ]
    tk.Button( frame, text='GET POSE', command=lambda : get_current_pose(sbs) ).pack(side="left")
    frame.pack()

    frame = tk.Frame(root)
    tk.Label(frame, text="MOVE BASE", width=15  ).pack(side="left")
    tk.Button( frame, text='<', command=lambda : set_vel(0.0, 0.1)  ).pack(side="left")
    tk.Button( frame, text='^', command=lambda : set_vel(0.05, 0.0)  ).pack(side="left")
    tk.Button( frame, text='v', command=lambda : set_vel(-0.05, 0.0)   ).pack(side="left")
    tk.Button( frame, text='>', command=lambda : set_vel(0.0, -0.1)   ).pack(side="left")
    tk.Button( frame, text='stop', command=lambda : set_vel(0.0, 0.0) ).pack(side="left")
    frame.pack()

    send_navi_goal_loop()
    root.mainloop()

if __name__=="__main__":
    rospy.init_node("test")
    trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    pub_cmdvel = rospy.Publisher( "/stretch/cmd_vel", Twist, queue_size=1, latch=True)
    
    root = tk.Tk()
    root.title("stretch controller")


    main()
