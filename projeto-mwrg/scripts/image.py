#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
from numpy.lib.twodim_base import _trilu_dispatcher
import rospy
import numpy as np
import math
import cv2
import cv2.aruco as aruco
import time
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

#print(sys.argv)
class Image_converter:

    def __init__(self):
        rospy.init_node('follower')

        self.bridge = CvBridge()
        self.cv_image = None
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback, queue_size=4, buff_size = 2**24)
        self.publica_dif = rospy.Publisher('/dif', String, queue_size=1)
        self.w = -1
        self.h = -1
        self.cx = -1
        self.cy = -1
        self.dif = -1
        self.proxdireita = False

    def image_callback(self, msg):
        
        try:
            #Tratando as imagens
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Definindo campo de visão do robô
            h, w, d = cv_image.shape
            search_top = 3*h//4
            search_bot = 3*h//4 + 20

            # Definindo configurações do Aruco
            aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

            # Definindo para seguir linha amarela
            lower_yellow = np.array([22, 50, 50],dtype=np.uint8)
            upper_yellow = np.array([36, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            if ids is not None and len(ids)>0:
                aresta = abs(corners[0][0][0][0] - corners[0][0][1][0])
                if not self.proxdireita and ids[0] == [200] and aresta > 30:
                    # bloqueia a parte direita da imagem (vira a esquerda)
                    mask[search_top:search_bot, 3*w//5:w] = 0
                elif self.proxdireita and ids[0] == [200] and aresta > 30:
                    # bloqueia a parte esquerda da imagem (vira a direita)
                    mask[search_top:search_bot, 0:3*w//5] = 0
                elif ids[0] == [100]:
                    mask[search_top:search_bot, 3*w//5:w] = 0
                    self.proxdireita = True

            # Achando centro de massa dos pontos amarelos
            M = cv2.moments(mask)

            if M['m00'] > 0:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (self.cx, self.cy), 10, (0,0,255), -1)  
            
            #Atualizando atributos e publicando dif
            self.w = w
            self.h = h   
            self.dif = self.cx - self.w/2
            self.publica_dif.publish(str(self.dif))

            cv2.imshow('mask', mask)
            cv2.imshow("window", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print('ex', e)
    
    def centraliza_creeper(self, cor):
        try:
            if cor == "blue":
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                cor1 = np.array([86, 50, 50],dtype=np.uint8)
                cor2 = np.array([96, 255, 255],dtype=np.uint8)
                mask = cv2.inRange(hsv, cor1, cor2)

            if cor == "green":
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                cor1 = np.array([50, 50, 50],dtype=np.uint8)
                cor2 = np.array([60, 255, 255],dtype=np.uint8)
                mask = cv2.inRange(hsv, cor1, cor2)

            if cor == "orange":
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                cor1 = np.array([170, 50, 50],dtype=np.uint8)
                cor2 = np.array([10, 255, 255],dtype=np.uint8)
                mask = cv2.inRange(hsv, cor1, cor2)            

            M = cv2.moments(mask)

            if M['m00'] > 0:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
        except: 
            pass

def main(args):
  ic = Image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)