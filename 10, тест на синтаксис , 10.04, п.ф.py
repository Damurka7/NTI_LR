#coding: utf8
#подключаем все библиотеки
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
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) #нужн для распознавания маркеров
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

telemetry = get_telemetry(frame_id='aruco_map')  #эта переменная хранит в себе координаты х и у коптера


cap = cv2.VideoCapture(-1) #закидывваем в переменную сар видеопоток с любой свободной  камеры на коптере (если так можно делать на клевере, просто на компьютере с вебкамерой все прекрасно работало)
x = open('x.txt', 'w') #создаем или открываем файлы, где будем хранить координаты
y = open('y.txt', 'w') #


while(True): #создаем бесконечный цикл, чтобы смотреть, что там камера наша показывает

    ret, frame = cap.read() #создаем переменные, они позже понадобятся. Видеопоток обрабатывается кдр за кдаром

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #красим картинку в чб
    res = cv2.aruco.detectMarkers(frame,dictionary) #распознаем там маркеры

    img = frame #да, тут можно было обойтись без этого, но пришлось бы менять везде переменную, а это заняло бы время
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #переводим цвета из ргб в хсв


    mask_red = cv2.inRange(img_hsv, (0, 50, 20), (5, 255, 255)) #задаем границы дял поиска карсного цвета
    mask2 = cv2.inRange(img_hsv, (175, 50, 20), (180, 255, 255)) #задаем границы дял поиска карсного цвета


    mask = cv2.bitwise_or(mask_red, mask2) #создаем маску
    croped = cv2.bitwise_and(img, img, mask=mask) #создаем изображение с маской

    hsv_min = np.array((0, 54, 5), np.uint8) #задаем границы дял поиска карсного цвета
    hsv_max = np.array((187, 255, 253), np.uint8) #задаем границы дял поиска карсного цвета
    h = 50 #высота отступа
    w = 50 #ширина отступа
    x, y = 320, 220 #координаты левой нижне точки, от которой пойдет отступ
    img=croped #присваиваем
    crop_img = img[y:y + h, x:x + w] # обрезаем изображение
    img=crop_img #усложнил код, люблю так делать ммммм)))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # меняем цветовую модель с BGR на HSV
    thresh = cv2.inRange(hsv, hsv_min, hsv_max)  # применяем цветовой фильтр
    contours0, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #ищем количество контуров

    if len(contours0) == 1: #если будет один контур
        x = open('x.txt', 'w') #то записываем координаты коптера в файлы
        y = open('y.txt', 'w') #
        x.write(telemetry.x) #записываем
        y.write(telemetry.y) #еще записываем

    for cnt in contours0:# перебираем все найденные контуры в цикле
        rect = cv2.minAreaRect(cnt)  # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect)  # поиск четырех вершин прямоугольника
        box = np.int0(box)  # округление координат
        area = int(rect[1][0] * rect[1][1])  # вычисление площади
        if area > 500: #отбрасываем слишком маленькие конутры, это косяки камеры
            cv2.drawContours(img, [box], 0, (255, 0, 0), 2) #





    if len(res[0]) > 0:  #нужно чтобы все работало
        cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])



    if cv2.waitKey(1) & 0xFF == ord('q'): #тоже нужно
        break

    x.close() #не забываем
    y.close() #закрыть файлы


cap.release() #
cv2.destroyAllWindows() #все
