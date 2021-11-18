#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
from numpy.lib.twodim_base import _trilu_dispatcher
import rospy
import numpy as np
import math
import cv2
import cv2.aruco as aruco
import rospkg
import time
import argparse
import os
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError
from sklearn.linear_model import LinearRegression

def mascara_creeper(cor, img):
    """
        Recebe: cor desejada do creeper (escolhida a partir do terminal), imagem em hsv
        Devolve: máscara segmentando cor do creeper, centro de massa do creeper
    """
    hsv = img.copy()
    cx_creeper = None
    try:
        if cor == "blue":
            cor1 = np.array([90, 140, 140],dtype=np.uint8)
            cor2 = np.array([100, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, cor1, cor2)

        if cor == "green":
            cor1 = np.array([55, 150, 200],dtype=np.uint8)
            cor2 = np.array([65, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, cor1, cor2)

        if cor == "orange":
            cor1 = np.array([0, 200, 200],dtype=np.uint8)
            cor2 = np.array([4, 255, 255],dtype=np.uint8)
            cor3 = np.array([177, 200, 200],dtype=np.uint8)
            cor4 = np.array([180, 255, 255],dtype=np.uint8)
            mask1 = cv2.inRange(hsv, cor1, cor2)
            mask2 = cv2.inRange(hsv, cor3, cor4)
            mask = cv2.bitwise_or(mask1,mask2)
        
        # Achando centro de massa do creeper
        try:
            M = cv2.moments(mask)

            if M['m00'] > 0:
                cx_creeper = int(M['m10']/M['m00'])
        except:
            pass    

        return mask,cx_creeper        
    except: 
        pass

class Image_converter:
 
    def __init__(self):
        """
            Construtor da classe (publishers + subscribers + atributos)
        """
        rospy.init_node('follower')

        #Câmera do robô
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback, queue_size=4, buff_size = 2**24)
        self.bridge = CvBridge()
        self.cv_image = None
        self.w = -1
        self.h = -1
        self.cx = -1
        self.cy = -1
        self.dif = -1

        #Publishers de atributos para control.py
        self.publica_dif = rospy.Publisher('/dif', String, queue_size=1)
        self.dif = -1

        self.publica_cx_creeper = rospy.Publisher('/cx_creeper', String, queue_size=1)
        self.cx_creeper = -1

        self.publica_angulo = rospy.Publisher('/angulo_linha_amarela', String, queue_size=1)
        self.angulo_linha_amarela = 0

        # self.publica_id = rospy.Publisher('/id_creeper', int, queue_size=1)


        #Atributos lógicos
        self.proxdireita = False

        #Recebimento de instruções pelo terminal
        self.cor_creeper = sys.argv[1]
        # self.id_creeper = sys.argv[2]

    def image_callback(self, msg):
        
        try:
            #Tratando as imagens
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Definindo campo de visão do robô
            h, w, d = cv_image.shape
            search_top = h//2
            search_bot = 3*h//4 + 20

            # Definindo configurações do Aruco
            aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

            # Definindo para seguir linha amarela
            lower_yellow = np.array([22, 200, 200],dtype=np.uint8)
            upper_yellow = np.array([36, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            mask_angulo = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask_angulo[0:search_top, 0:w] = 0
            mask_angulo[search_bot:h, 0:w] = 0

            if ids is not None and len(ids)>0:
                aresta = abs(corners[0][0][0][0] - corners[0][0][1][0])
                if not self.proxdireita and ids[0] == [200] and aresta > 30:
                    # bloqueia a parte direita da imagem (vira a esquerda)
                    mask[search_top:search_bot, 3*w//5:w] = 0
                    mask_angulo[:,3*w//5:w] = 0
                elif self.proxdireita and ids[0] == [200] and aresta > 30:
                    # bloqueia a parte esquerda da imagem (vira a direita)
                    mask[:, 0:3*w//4] = 0
                    mask_angulo[:, 0:3*w//4] = 0
                elif ids[0] == [100]:
                    mask[search_top:search_bot, 3*w//5:w] = 0
                    mask_angulo[:,3*w//5:w] = 0
                    self.proxdireita = True

            # Achando contornos e centros dos contornos
            # Mask_angulo vai ser utilizado para encontrar o angulo entre o robo e a faixa amarela

            contornos_angulo, _ = cv2.findContours(mask_angulo, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            lista_x_contornos = []
            lista_y_contornos = []
            
            for c in contornos_angulo:
                M_angulo = cv2.moments(c)
                if M_angulo['m00'] > 0:
                    lista_x_contornos.append(int(M_angulo['m10'] / M_angulo['m00']))
                    lista_y_contornos.append(int(M_angulo['m01'] / M_angulo['m00']))
            
            if lista_x_contornos:
                array_x_contornos = np.array(lista_x_contornos)
                array_y_contornos = np.array(lista_y_contornos)

                # Preparando dados dos centros dos contornos coletados
                # para utilizar um modelo de regressao linear
                yr = array_y_contornos.reshape(-1,1)
                xr = array_x_contornos.reshape(-1)

                reg = LinearRegression()
                reg.fit(yr,xr)
            
                # Pega o angulo com a vertical e publica ele 
                # para usar no controle proporcional diferencial
                coef_angular = reg.coef_
                self.angulo_linha_amarela = math.atan(coef_angular)
                self.publica_angulo.publish(str(self.angulo_linha_amarela))

            # Achando centro de massa dos pontos amarelos
            M = cv2.moments(mask)

            if M['m00'] > 0:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (self.cx, self.cy), 10, (0,0,255), -1)
            
            # Definindo máscara para creepers a partir dos args
            mask_creeper, cx_creeper = mascara_creeper(self.cor_creeper, hsv)

            #Atualizando atributos e publicando dif

            self.w = w
            self.h = h
            self.cx_creeper = cx_creeper
            self.dif = self.cx - self.w/2
            self.publica_dif.publish(str(self.dif))

            if self.cx_creeper is not None:
                contornos, arvore = cv2.findContours(mask_creeper, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

                maior_area = 0
                for contorno in contornos:
                    area = cv2.contourArea(contorno)
                    if area > maior_area:
                        maior_area = area
                #print(maior_area)
                if maior_area > 150:
                    self.publica_cx_creeper.publish(str(self.cx_creeper))
                

            #cv2.imshow('mask', mask)
            cv2.imshow('mask', mask_angulo)
            #cv2.imshow('cv_image', cv_image)
            cv2.imshow('mask_creeper', mask_creeper)
            cv2.waitKey(1)
            print(self.cx)

        except CvBridgeError as e:
            print('ex', e)

class Mobile_net:
    
    def __init__(self):
        rospy.init_node('follower')

        self.rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.path = self.rospack.get_path("projeto-mwrg")
        path = self.path
        self.scripts = os.path.join(path,  "scripts")
        self.image1 = None
        self.proto = os.path.join(self.scripts,"MobileNetSSD_deploy.prototxt.txt")
        self.model = os.path.join(self.scripts, "MobileNetSSD_deploy.caffemodel")
        self.confianca = 0.5
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.detect, queue_size=4, buff_size = 2**24)
        
        
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
        "sofa", "train", "tvmonitor"]
        
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        self.net = cv2.dnn.readNetFromCaffe(self.proto, self.model)

    def detect(self, frame):

        try:
            image1 = self.bridge.compressed_imgmsg_to_cv2(frame,desired_encoding='bgr8')
            (h, w) = image1.shape[:2]
            blob = cv2.dnn.blobFromImage(cv2.resize(image1, (300, 300)), 0.007843, (300, 300), 127.5)

            # pass the blob through the network and obtain the detections and
            # predictions
            # print("[INFO] computing object detections...")
            self.net.setInput(blob)
            detections = self.net.forward()
            results = []


            # loop over the detections
            for i in np.arange(0, detections.shape[2]):
                # extract the confidence (i.e., probability) associated with the
                # prediction
                confidence = detections[0, 0, i, 2]

                # filter out weak detections by ensuring the `confidence` is
                # greater than the minimum confidence


                if confidence > self.confianca:
                    # extract the index of the class label from the `detections`,
                    # then compute the (x, y)-coordinates of the bounding box for
                    # the object
                    idx = int(detections[0, 0, i, 1])
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    # display the prediction
                    label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
                    #print("[INFO] {}".format(label))
                    cv2.rectangle(image1, (startX, startY), (endX, endY),
                        self.COLORS[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(image1, label, (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)

                    results.append((self.CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))
                    #print(results)
        except:
            pass

def main(args):
  ic = Image_converter()
  mn = Mobile_net()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)