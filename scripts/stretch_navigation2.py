#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import math
import tf
import time
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PointStamped, Twist
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# 場所の名前とその座標を設定
# mapはtest.yamlを使用
locations = {
    "crane_table" : (2.65, 1.60, 1.53),
    # "middle_point" : (2.65, 0.30, 3.1),
    "middle_point" : (2.65, 0.20, 3.1),
    "human" : (-0.650, 0.250, -1.57),
    # "table" : (1.43, 0.10, -2.99),
    "table" : (1.30, 0.20, 3.1),

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




def move_robot( lift=None, length=None, wrist=None, pan=None, tilt=None, gripper=None, timeout=20 ):
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
    print(point)

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = joint_names

    trajectory_goal.trajectory.points = [point]
    trajectory_goal.trajectory.header.stamp = rospy.Time.now()
    trajectory_client.send_goal(trajectory_goal)
    #trajectory_client.wait_for_result()

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
        diff = normalize_angle(cur_rot-init_rot) # 現在の移動量

        # 現在の移動量と目標との差で比例制御
        if theta>=0:       
            rot = min( max( 0.1, (theta-diff)*0.3 ), 0.2 )
        else:
            rot = min( max( -0.2, (theta-diff)*0.3 ), -0.1 )

        move_vel( 0, rot )

        print(init_rot, cur_rot, diff, theta, diff-theta ,rot)
        if theta>0 and diff>theta:
            break

        if theta<0 and diff<theta:
            break

    move_vel( 0, 0 )

def ar_marker_rec_to_grasp():
    # 初期位置へ移動
    move_robot( length=0, wrist=2.5, pan=-1.5, tilt=-0.8 )
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
    move_robot( lift=target_rob[2]+0.06 )
    # move_robot( lift=target_rob[2]+0.05 )
    # move_robot( wrist=0.0  )
    move_robot( wrist=0.37  )
    

    # ハンドを開く
    print("ハンドを開く")
    move_robot( gripper=0.160 )

    # 腕を伸ばす
    print("腕を伸ばす")
    offset_length = 0.33
    length_= min( -target_rob[1]-offset_length+0.05, 0.51 )
    move_robot( length=length_ )

    # ハンドを閉じる
    print("ハンドを閉じる")
    move_robot( gripper=-0.2 )

    # 腕を縮める
    print("腕を縮める")
    move_robot( length=0 )

    # 手首をひねる
    print("手首をひねる")
    move_robot( wrist=2.5)

    time.sleep(5)

def ar_marker_rec_to_place():
    # 初期位置へ移動
    # move_robot( length=0, wrist=2.5, pan=-1.5, tilt=-0.8 )
    # move_robot( lift=0.2 )

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
    # 正確な高さがいまいちわからない状況
    print("腕を上げる")
    move_robot( lift=max(target_rob[2]+0.05, 0.78 ) )
    move_robot( wrist=0.0  )
    # move_robot( wrist=0.25  )
    
    # 腕を伸ばす
    print("腕を伸ばす")
    offset_length = 0.33
    length_= min( -target_rob[1]-offset_length+0.1, 0.51 )
    move_robot( length=length_ )

    # ハンドを開く
    print("ハンドを開く")
    move_robot( gripper=0.160 )

    # 腕を縮める
    print("腕を縮める")
    move_robot( length=0 )

    # 手首をひねる
    print("手首をひねる")
    move_robot( wrist=2.5)

    time.sleep(5)

def tabletop_rec_to_grasp():
    # 初期位置へ移動
    move_robot( length=0, wrist=2.5, pan=-1.5, tilt=-0.8 )
    move_robot( lift=0.2 )

    # ターゲットとなる物体がカメラに映るのを待つ
    target_cam = None
    while not rospy.is_shutdown():
        data = rospy.wait_for_message( "/object_rec/object_info", String )
        obj_info = yaml.safe_load(data.data)

        if len(obj_info)==0:
            print("物体なし")
            continue
        elif obj_info:
            for o in obj_info:
                if o["label"]==3:
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

    move_robot( lift=max(target_rob[2]-0.03, 0.8 ) )

    # move_robot( wrist=0.0  )
    move_robot( wrist=0.20  )
    

    # ハンドを開く
    print("ハンドを開く")
    move_robot( gripper=0.160 )

    # 腕を伸ばす
    print("腕を伸ばす")
    offset_length = 0.33
    length_= min( -target_rob[1]-offset_length-0.01, 0.51 )
    move_robot( length=length_ )

    # ハンドを閉じる
    print("ハンドを閉じる")
    move_robot( gripper=-0.2 )

    # 腕を縮める
    print("腕を縮める")
    move_robot( length=0 )

    # 手首をひねる
    print("手首をひねる")
    move_robot( wrist=2.5)

    time.sleep(5)



def main():
    
    # 移動指令
    result = send_navi_goal( *locations["middle_point"] )
    print(result)

    result = send_navi_goal( *locations["table"] )
    print(result)

    
    
    print('テーブル上のモノを掴みます')
    tabletop_rec_to_grasp()

    result = send_navi_goal( *locations["middle_point"] )
    print(result)

    
    result = send_navi_goal( *locations["crane_table"] )
    print(result)

    print('ARマーカーを認識してカップ麺を置きます')
    ar_marker_rec_to_place()
    

    print('finish')

    '''
    print('ARマーカーを認識して掴みます')
    ar_marker_rec_to_grasp()

    print('all finish')
    '''
    

if __name__ == '__main__':
    rospy.init_node('navigation')
    pub_cmdvel = rospy.Publisher( "/stretch/cmd_vel", Twist, queue_size=1, latch=True)
    trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    trajectory_client.wait_for_server(timeout=rospy.Duration(60.0))
    tf_listener = tf.TransformListener()



    

    '''
    result = send_navi_goal( *locations["middle_point"] )
    print(result)

    if result == 4:
        result = send_navi_goal( *locations["middle_point"] )
        print(result)

    result = send_navi_goal( *locations["human"] )
    print(result)
    '''

    main()
    # result = send_navi_goal( *locations["middle_point"] )
    # print(result)
    # result = send_navi_goal( *locations["living"] )
    # print(result)

    # result = send_navi_goal( *locations["desk"] )
    # print(result)

