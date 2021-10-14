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

contornosTeste = []
contornos = []
larguraTela = 640
X = None
Y = None
maskTeste = None
oi = None
alo = None



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

def centralizaPista2(mask):

    global direita
    global esquerda
    global frente
    global larguraTela

    ret, thresh = cv2.threshold(mask,200, 255, cv2.THRESH_BINARY)
    contours,_ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        try: 
        #encontrando o centro do maior contorno
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])


            if (larguraTela/2 - 30) < cx < (larguraTela/2 + 30):
                velocidade_saida.publish(frente)

            elif (larguraTela/2 - 30) > cx:
                velocidade_saida.publish(direita)
            
            elif (larguraTela/2 + 30) < cx:
                velocidade_saida.publish(esquerda)

        except:
            pass

        else: 
            default = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
            velocidade_saida.publish(default)
            print("Não recebeu cX")


def centralizaPista3(cX3,cY3):

    global direita
    global esquerda
    global frente
    global larguraTela
    
    print("OLHA AQUI O CX= ",cX3)

    if cX3 is not None:
        
        if (larguraTela/2 - 30) < cX3 < (larguraTela/2 + 30):
            velocidade_saida.publish(frente)

        elif (larguraTela/2 - 30) > cX3:
            velocidade_saida.publish(esquerda)
            
        elif (larguraTela/2 + 30) < cX3:
            velocidade_saida.publish(direita)

        else: 
            default = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
            velocidade_saida.publish(default)
            print("Sem centro")

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
        imagemMain, angulo, contornos, cX, cY = modulo_visao.processa_imagem(temp_image)

        """
        Teste 163,164
        """
        maskTeste = modulo_visao.segmenta_linha_amarela(temp_image)
        contornosTeste = modulo_visao.encontrar_contornos(maskTeste)
        oi, alo = modulo_visao.centro_maior_contorno(maskTeste)
        

        shape = temp_image.shape
        print(shape)
        larguraTela = shape[1]



        cv2.imshow('imagem', imagemMain)
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
        cv2.imshow("cv_image", cv_image)
        cv2.imshow("Linhas amarelas", maskTeste)

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
            centralizaPista2(maskTeste)
            #centralizaPista(angulo)
            #centralizaPista3(oi,alo)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
