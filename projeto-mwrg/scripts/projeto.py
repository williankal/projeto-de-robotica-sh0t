#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import cv2.aruco as aruco
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from termcolor import colored
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

import modulo_visao

bridge = CvBridge()

tfl = 0
cv_image = None
tf_buffer = tf2_ros.Buffer()

resultados = []
media = []
centro = []

atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos



def recebe_odometria(data):
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    print("Valores da odometria:", x,y) 

def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global resultados

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        # Esta logica do delay so' precisa ser usada com robo real e rede wifi 
        # serve para descartar imagens antigas
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  modulo_visao.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

"""
def processa_imagem(frame):

    img = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    hsv1 = np.array([20, 100, 150])
    hsv2 = np.array([35, 255, 255])

    mask = cv2.inRange(hsv, hsv1, hsv2)

    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5, 5)))

    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    X = []
    Y = []

    for contorno in contornos:
        M = cv2.moments(contorno)
        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            point = (int(cX), int(cY))
            crosshair(img_contornos, point, 15, (0, 0, 255))
            X.append(int(cX))
            Y.append(int(cY))


"""

if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_odom = rospy.Subscriber("/odom", Odometry , recebe_odometria)


    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    try:
        # Inicializando - por default gira no sentido anti-horário
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
