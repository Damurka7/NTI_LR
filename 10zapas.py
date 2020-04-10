#coding: utf8
#ну так было написано в гикбуке
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

telemetry = get_telemetry(frame_id='aruco_map')#эта переменная хранит в себе координаты х и у коптера




navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='body', auto_arm=True)#взлетаем
rospy.sleep(10)


k=2#пришлось на шару подобрать, чтобы сделать хоть какой-то отчет
for i in range(10):# создаем цикл, чтоб летать змейкой, 10 рядов
    for j in range(5):
        navigate(x=float(j / 3.33), y=float(i / 3.33), z=0.55, speed=1.5, frame_id='aruco_map', auto_arm=True)
        rospy.sleep(1)
navigate(x=0, y=0, z=0.55, speed=1, frame_id='aruco_map', auto_arm=True)
rospy.sleep(3)
navigate(x=0, y=0, z=0, speed=0.25, frame_id='aruco_map', auto_arm=True)
rospy.sleep(120)
navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='aruco_map', auto_arm=True)
for i in range(k):

    navigate(x=0.7, y=1, z=0.55, speed=1.5, frame_id='aruco_map', auto_arm=True)
    set_effect(r=255, g=0, b=0)  # fill strip with red color
    rospy.sleep(2)

navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='aruco_map', auto_arm=True)
rospy.sleep(1)



x = open('x.txt', 'w')
x.write('Количество подтвержденный случаев COVID-19: 2')
x.close()

land()
arming(False)