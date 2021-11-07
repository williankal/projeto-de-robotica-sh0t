#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import math
import cv2
import time
import sys
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Dados de Odometria
x = -1
y = -1
z = -1

def recebeu_leitura(dado):
    """
        Grava nas variáveis x,y,z a posição extraída da odometria
        Atenção: *não coincidem* com o x,y,z locais do drone
    """
    global x
    global y 
    global z 

    x = dado.pose.pose.position.x
    y = dado.pose.pose.position.y
    z = dado.pose.pose.position.z

class MaquinaDeEstados:

    def __init__(self):

        #Publishers e subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)                   
        self.dif_subscriber = rospy.Subscriber('/dif', String, self.atualiza_dif)
        self.cx_creeper_subscriber = rospy.Subscriber('/cx_creeper', String, self.atualiza_cx_creeper)

        #Atributos da ME
        self.twist = Twist()
        self.laser_msg = LaserScan()
        self.dif = -1 #diferença dada por image.py
        self.cx_creeper = None # posição em x do creeper
        self.w = 640 # comprimento lateral da tela (pixels)
        self.lastError = 0
        self.deuVolta = False
        self.estado = "SEGUE RETA"
        self.max_vel_linear = 0.2
        self.max_vel_angular = 2.0
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)

    #Funções de Callback
    def laser_callback(self, msg):
        self.laser_msg = msg
    
    def atualiza_dif(self, msg):
        self.dif = float(msg.data)

    def atualiza_cx_creeper(self,msg):
        data = None
        if msg.data is not None:
            data = float(msg.data)
        self.cx_creeper = data

    def get_laser(self, pos):
        return self.laser_msg.ranges[pos]
    
    #Estados
    def segue_reta(self):

        estado = "SEGUE RETA"        
        #Controle P simples
        self.twist.linear.x = 0.4
        self.twist.angular.z = - self.dif / 100
        
        #publica velocidade
        self.cmd_vel_pub.publish(self.twist)
        
        if x>0.5 and y<1:
            self.deuVolta = True
        
        if self.deuVolta == True and x>-0.15 and x<0.15 and y>-0.05 and y<0.05:
            estado = "PARA"

        if self.cx_creeper is not None and 512<self.cx_creeper<640:
            estado = "FOCA CREEPER"

        return estado
    
    def foca_creeper(self):
        estado = "FOCA CREEPER"
        self.twist.linear.x = 0

        if self.cx_creeper is not None:
            #sabe-se que o tamanho da tela é 640 pixels
            if self.cx_creeper > 320:
                self.twist.angular.z = -0.1
            elif self.cx_creeper <320:
                self.twist.angular.z = 0.1
        
        if 

        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()

        return estado
    
    def stop(self):
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()
        rospy.sleep(5)
        return "PARA"

    #Controle
    def control(self):

        if self.estado=="SEGUE RETA":
            self.estado = self.segue_reta()
        
        if self.estado=="FOCA CREEPER":
            self.estado = self.foca_creeper()

        if self.estado=="PARA":
            self.estado = self.stop()
          
if __name__== "__main__":
    rospy.init_node('Controller')
    controller = MaquinaDeEstados()
    recebe_scan = rospy.Subscriber('/odom', Odometry , recebeu_leitura)

    while not rospy.is_shutdown():
        controller.control()
        

        
