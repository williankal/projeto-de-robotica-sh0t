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
cX = 0
cY = 0
angulo = None

atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

frente = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
direita = Twist(Vector3(0.05,0,0), Vector3(0,0,0.2))
esquerda = Twist(Vector3(0.05,0,0), Vector3(0,0,-0.2))
zero = Twist(Vector3(0,0,0), Vector3(0,0,0))

contornosTeste = None
larguraTela = 640



def recebe_odometria(data):
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    print("Valores da odometria:", x,y) 

def centralizaPista(angulo):

    global direita
    global esquerda
    global frente

    if angulo != None:
        if -10 < angulo < 10:
            velocidade_saida.publish(frente)

        elif angulo <= -10:
            velocidade_saida.publish(esquerda)
        
        elif angulo >= 10:
            velocidade_saida.publish(direita)

    else: 
        default = Twist(Vector3(0,0,0), Vector3(0,0,-0.01))
        velocidade_saida.publish(default)
    
    return None

def centralizaPista2(contornos):

    global direita
    global esquerda
    global frente
    global larguraTela

    foco = None
    areaAmarela = 0
    cX = None
    cY = None 
    
    if contornos is not None: 
        for contorno in contornos:
            area = cv2.contourArea(contorno)
            if area > areaAmarela:
                areaAmalera = area
                foco = contorno

            M = cv2.moments(foco)

            try:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                
            
            except: 
                pass
    
        if cX != None:

            if (larguraTela/2 - 30) < cX < (larguraTela/2 + 30):
                velocidade_saida.publish(frente)

            elif (larguraTela/2 - 30) > cX:
                velocidade_saida.publish(direita)
            
            elif (larguraTela/2 + 30) < cX:
                velocidade_saida.publish(esquerda)

        else: 
            default = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
            velocidade_saida.publish(default)
            print("Não recebeu cX")
    
    return None



        





def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global resultados
    global angulo

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        # delay para robo real, descarta imagens antigas
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # chamada resultados
        centro, saida_net, resultados =  modulo_visao.processa(temp_image) 
        
            # print(r) - print feito para documentar e entender
            # o resultado            
        imagemMain, angulo, contornos = modulo_visao.processa_imagem(temp_image)
        """
        Teste 163,164
        """
        maskTeste = modulo_visao.segmenta_linha_amarela(temp_image)
        contornosTeste = modulo_visao.encontrar_contornos(maskTeste)
        print("OLHA AQUII = ", contornosTeste)

        shape = temp_image.shape
        print(shape)
        larguraTela = shape[1]



        cv2.imshow('imagem', imagemMain)
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
    


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
        
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            centralizaPista2(contornosTeste)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
