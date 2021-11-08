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
from std_msgs.msg import Float64

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
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)
        self.cx_creeper_subscriber = rospy.Subscriber('/cx_creeper', String, self.atualiza_cx_creeper)

        #Atributos da ME
        self.twist = Twist()
        self.laser_central = 0
        self.dif = -1 #diferença dada por image.py
        self.cx_creeper = None # posição em x do creeper
        self.w = 640 # comprimento lateral da tela (pixels)
        self.lastError = 0
        self.lado_direito = False
        self.estado = "SEGUE RETA"
        self.max_vel_linear = 0.2
        self.max_vel_angular = 2.0
        self.hertz = 250
        self.colidiu = False
        self.rate = rospy.Rate(self.hertz)

    #Funções de Callback
    def laser_callback(self, msg):
        laser_msg = msg.ranges
        self.laser_central = laser_msg[0]
        if laser_msg[0] < 0.15 or laser_msg[1]< 0.15 or laser_msg[2]< 0.15 or laser_msg[3]< 0.15 or laser_msg[359] < 0.15 or laser_msg[358] < 0.15 or laser_msg[357] < 0.15:
            self.colidiu = True
        else:
            self.colidiu = False
    
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
        self.twist.linear.x = 0.5
        self.twist.angular.z = - self.dif / 100
        
        #publica velocidade
        self.cmd_vel_pub.publish(self.twist)
        
        if x>0.25:
            self.lado_direito = True
        
        if self.lado_direito == True and x>-0.15 and x<0.15 and y>-0.05 and y<0.05:
            estado = "PARA"

        if self.cx_creeper is not None:
            if  0<self.cx_creeper<self.w//5 or self.w*4//5<self.cx_creeper<self.w:
                estado = "FOCA CREEPER"
        return estado
    
    def foca_creeper(self):
        estado = "FOCA CREEPER"
        self.twist.linear.x = 0
        centro = self.w//2

        if self.cx_creeper is not None:
            if self.cx_creeper > centro:
                self.twist.angular.z = -0.1
            elif self.cx_creeper <centro:
                self.twist.angular.z = 0.1
        
        if centro - 10 < self.cx_creeper < centro + 10:
            estado = "SEGUE CREEPER"

        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()

        return estado

    def pega_creeper(self):

        estado = "PEGA CREEPER"
        
        self.garra.publish(-1.0)
        self.ombro.publish(-0.5)
        rospy.sleep(2.0)
        self.garra.publish(0.0) ## fecha
        rospy.sleep(1.0)
        self.ombro.publish(1.5) ## para cima

        estado = "PARA"



        return estado


    def segue_creeper(self):
        estado = "SEGUE CREEPER"
        centro = self.w//2

        if centro - 30 < self.cx_creeper < centro + 30:
            self.twist.linear.x = 0.05
            self.twist.angular.z = 0

        elif self.colidiu:
            self.twist.linear.x = 0
            self.twist.angular.z = 0            
            estado = "PEGA CREEPER"

        else:
            estado = "FOCA CREEPER"   

        
        
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()
        return estado

    def stop(self):
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()
        return "PARA"

    #Controle
    def control(self):

        print(self.estado)

        if self.estado=="SEGUE RETA":
            self.estado = self.segue_reta()
        
        elif self.estado=="FOCA CREEPER":
            self.estado = self.foca_creeper()
        
        elif self.estado=="SEGUE CREEPER":
            self.estado = self.segue_creeper()

        elif self.estado == "PEGA CREEPER":
            self.estado = self.pega_creeper()

        elif self.estado=="PARA":
            self.estado = self.stop()
          
if __name__== "__main__":
    rospy.init_node('Controller')
    controller = MaquinaDeEstados()
    recebe_scan = rospy.Subscriber('/odom', Odometry , recebeu_leitura)

    while not rospy.is_shutdown():
        controller.control()
        

        
