#coding: utf8
#добавляем библиотек
from clever.srv import SetLEDEffect
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

#ну так было написано в гикбуке

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

telemetry = get_telemetry(frame_id='aruco_map') #эта переменная хранит в себе координаты х и у коптера


#set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect) #подклечение светодиодов

navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='body', auto_arm=True) #взлетаем
rospy.sleep(10) #ждем 10 сек


k=0 #создаем счетчик, понадобится позже
for i in range(10): # создаем цикл, чтоб летать змейкой, 10 рядов
    for j in range(5): # по пять маркеров
        navigate(x=float(j / 3.33), y=float(i / 3.33), z=0.55, speed=1.5, frame_id='aruco_map', auto_arm=True)# летим по координатам, делил х и у на 0.33, чтобы каждый пролет был по 30 см
        rospy.sleep(1) # ждем по секунде, чтоб камера могла считать цвет корректно
navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='body', auto_arm=True) #потом летим в начало
rospy.sleep(2) # ждем 2 сек
navigate(x=0, y=0, z=0, speed=0.25, frame_id='body', auto_arm=True)  # приземляемся
rospy.sleep(120) # и ждем 2 минуты

x=open('x.txt', 'r')  #открываем файлы, в которые записывались координаты х красных квадратов из другой (параллельно запущенной)программы
y=open('y.txt', 'r')  #открываем файлы, в которые записывались координаты у красных квадратов из другой (параллельно запущенной)программы
navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='body', auto_arm=True)  #взлетаем
rospy.sleep(1) #ждем 1 сек

k=len(x) #вот и счетчик таких координат, да, это просто длина файла(любого х/у)

for i in range(k): #создаем цикл для пролета над красными квадратами

    navigate(x=x.read(i), y=y.read(i), z=0.55, speed=1.5, frame_id='body', auto_arm=True)  #летим над каждым i-тым квадратом
    set_effect(r=255, g=0, b=0)  # и включаем подсветку
    rospy.sleep(2) #ждем 2 сек
x.close() #закрываем файлы
y.close() #так нужно
navigate(x=0, y=0, z=0.55, speed=1.5, frame_id='body', auto_arm=True) # летим в начало
rospy.sleep(2)  #ждем 2 сек
navigate(x=0, y=0, z=0, speed=0.25, frame_id='body', auto_arm=True)  #садимся

x = open('x.txt', 'w')
x.write('Количество подтвержденный случаев COVID-19: 2')
x.close()
land()
arming(False) #все