# -*- coding: utf-8 -*-

from __future__ import print_function, unicode_literals
import rospy
from std_srvs.srv import Trigger
import time

# stretch driverが起動するまで待ってから実行
time.sleep(10)

rospy.init_node('switch_to_nav' )
rospy.wait_for_service('switch_to_navigation_mode')
switch = rospy.ServiceProxy('switch_to_navigation_mode', Trigger)
print( switch() )
    
