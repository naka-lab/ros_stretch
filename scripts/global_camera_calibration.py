#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time
import yaml
from math import pi
import sys
import torch
import numpy as np
import matplotlib.pyplot as plt
import os
import tf
from geometry_msgs.msg import PoseStamped, PointStamped


def recieve_ar_makers():
    while not rospy.is_shutdown():
        data = rospy.wait_for_message( "/ar_marker_rec/object_info", String )
        #print( data.data )

        data = yaml.load(data.data)

        for d in data:
            if d["label"]==0:
                return d["position"]

    return None

def transform(x, y, z, target_tf, from_tf ):
    tf_listener = tf.TransformListener()        
    tf_listener.waitForTransform( target_tf, from_tf, rospy.Time(0), rospy.Duration(5)  )

    pos_from_cam = PointStamped()
    pos_from_cam.header.frame_id = from_tf
    pos_from_cam.point.x = x
    pos_from_cam.point.y = y
    pos_from_cam.point.z = z

    pos_trans = tf_listener.transformPoint( target_tf, pos_from_cam )
    return pos_trans.point.x, pos_trans.point.y, pos_trans.point.z

def estimate_transform( pos_from_cam, pos_from_arm ):
    pos_from_cam = torch.tensor( pos_from_cam, dtype=torch.float ).reshape( -1, 3, 1 )
    pos_from_arm = torch.tensor( pos_from_arm, dtype=torch.float ).reshape( -1, 3, 1 )
    N = pos_from_arm.shape[0]

    rx = torch.tensor([ -1.57 ], requires_grad=True)
    ry = torch.tensor([ 3.14 ], requires_grad=True)
    rz = torch.tensor([ -1.66 ], requires_grad=True)

    x = torch.tensor([ 0.0 ], requires_grad=True)
    y = torch.tensor([ 0.0 ], requires_grad=True)
    z = torch.tensor([ 0.0 ], requires_grad=True)

    optimizer = torch.optim.Adam( [rx, ry, rz, x, y, z] )

    loss_list = []
    for i in range(8000):
        Rx = torch.eye( 3, 3 )
        Rx[1,1] = torch.cos(-rx)
        Rx[1,2] = -torch.sin(-rx)
        Rx[2,1] = torch.sin(-rx) 
        Rx[2,2] = torch.cos(-rx)

        Ry = torch.eye( 3, 3 )
        Ry[0,0] = torch.cos(-ry)
        Ry[0,2] = torch.sin(-ry)
        Ry[2,0] = -torch.sin(-ry) 
        Ry[2,2] = torch.cos(-ry)

        Rz = torch.eye( 3, 3 )
        Rz[0,0] = torch.cos(-rz)
        Rz[0,1] = -torch.sin(-rz)
        Rz[1,0] = torch.sin(-rz) 
        Rz[1,1] = torch.cos(-rz)

        T = torch.zeros(3,1)
        T[0,0] = x
        T[1,0] = y
        T[2,0] = z

        pos_pred = Rx.matmul(Ry.matmul(Rz.matmul(pos_from_arm-T)))
        loss = torch.sqrt( 1/N * torch.sum( (pos_from_cam-pos_pred)**2 ) )

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        loss_list.append(loss)

        if i%200==0:
            print(loss)


    return [ float(v) for v in (x, y, z, rx, ry, rz)]


def main():
    rospy.init_node("global_camera_calibration")

    # キャリブレーション結果の保存先
    path = os.path.abspath(__file__) + ".param"

    pos_from_cam = []
    pos_global = []

    if len(sys.argv)==3 and sys.argv[1]=="calib":
        N = int(sys.argv[2])
        print(f"{N}個のデータ点を取得します．link_aruco_inner_wrist上にマーカーを貼り，カメラで認識させてください．")
        for i in range(N):
            print( f"---- {i+1}/{N} ----" )
            input("Enterを押すと，計測・保存します．")
            pc = recieve_ar_makers()
            pg = transform( 0, 0, 0, "map", "link_aruco_inner_wrist" )
            pos_from_cam.append( pc )
            pos_global.append( pg )
            print( pc, pg )
        x,y,z,rx,ry,rz = estimate_transform( pos_from_cam, pos_global )

         # キャリブレーション結果を保存
        np.savetxt( path, [x,y,z,rx,ry,rz] )
    elif len(sys.argv)==2 and sys.argv[1]=="pub":
        x,y,z,rx,ry,rz = np.loadtxt( path )
    else:
        print( "*** キャリブレーションの実行 ***" )
        print( "n個のマーカーを計測して，キャリブレーションを実行" )
        print( "python global_camera_calibration.py calib n" )
        print()
        print( "*** 前回のキャリブレーション結果を再利用 ***" )
        print( "python global_camera_calibration.py pub" )
        return 


    print("****** 以下のコマンドを実行します ******")
    print("rosrun tf static_transform_publisher %.4f %.4f %.4f %.4f %.4f %.4f /map /camera_depth_optical_frame 100"%(x,y,z,rz,ry,rx) )
    print("**********************************************")

    os.system( "rosrun tf static_transform_publisher %.4f %.4f %.4f %.4f %.4f %.4f /map /camera_depth_optical_frame 100"%(x,y,z,rz,ry,rx) ) 
        
if __name__ == '__main__':
    main()
