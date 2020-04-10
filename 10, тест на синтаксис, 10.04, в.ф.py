#coding: utf8

import rospy
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
import math
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aruco_pose.msg import MarkerArray
import numpy as np
import cv2



arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)



rospy.init_node('flight')

telemetry = get_telemetry(frame_id='aruco_map')




navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='body', auto_arm=True)
rospy.sleep(10)


k=0
for i in range(15):
    for j in range(5):
        navigate(x=j / 3.33, y=i / 50, z=0.55, speed=1.5, frame_id='body', auto_arm=True)
        rospy.sleep(1)
navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='body', auto_arm=True)
rospy.sleep(2)
navigate(x=0, y=0, z=0, speed=0.25, frame_id='body', auto_arm=True)
rospy.sleep(120)

x=open('x.txt', 'r')
y=open('y.txt', 'r')
navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='body', auto_arm=True)
rospy.sleep(1)
for i in range(k):

    navigate(x=x.read(i), y=y.read(i), z=0.55, speed=1.5, frame_id='body', auto_arm=True)

    rospy.sleep(2)
x.close()
y.close()
navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='body', auto_arm=True)
rospy.sleep(2)
navigate(x=0, y=0, z=0, speed=0.25, frame_id='body', auto_arm=True)

land()
arming(False)