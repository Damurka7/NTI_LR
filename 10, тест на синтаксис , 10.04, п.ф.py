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

#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

telemetry = get_telemetry(frame_id='aruco_map')
io=0

cap = cv2.VideoCapture(0)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
        # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(frame,dictionary)

    img = frame
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        ## Gen lower mask (0-5) and upper mask (175-180) of RED
    mask_red = cv2.inRange(img_hsv, (0, 50, 20), (5, 255, 255))
    mask2 = cv2.inRange(img_hsv, (175, 50, 20), (180, 255, 255))

        ## Merge the mask and crop the red regions
    mask = cv2.bitwise_or(mask_red, mask2)
    croped = cv2.bitwise_and(img, img, mask=mask)

    hsv_min = np.array((0, 54, 5), np.uint8)
    hsv_max = np.array((187, 255, 253), np.uint8)
    h = 50
    w = 50
    x, y = 320, 220
    img=croped
    crop_img = img[y:y + h, x:x + w]
    img=crop_img
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # меняем цветовую модель с BGR на HSV
    thresh = cv2.inRange(hsv, hsv_min, hsv_max)  # применяем цветовой фильтр
    contours0, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours0) == 1:
        x = open('x.txt', 'w')
        y = open('y.txt', 'w')
        x.write()
        y.write()

        # перебираем все найденные контуры в цикле
    for cnt in contours0:
        rect = cv2.minAreaRect(cnt)  # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect)  # поиск четырех вершин прямоугольника
        box = np.int0(box)  # округление координат
        area = int(rect[1][0] * rect[1][1])  # вычисление площади
        if area > 500:
            cv2.drawContours(img, [box], 0, (255, 0, 0), 2)

        ## Read and merge



    if len(res[0]) > 0:
        cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])


        ## Display

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    x.write(io)
    y.write(io)
    io+=1
    x.close()
    y.close()
cap.release()
cv2.destroyAllWindows()
