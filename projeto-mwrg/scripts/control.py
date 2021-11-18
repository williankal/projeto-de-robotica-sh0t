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
from tf import transformations

# Dados de Odometria
x = -1
y = -1
theta = -1
contador = 0

def recebeu_leitura(dado):
    """
        Grava nas variáveis x e y a posição extraída da odometria
        Atenção: *não coincidem* com o x,y,z locais do drone
    """
    global x
    global y 
    global contador
    global theta

    x = dado.pose.pose.position.x
    y = dado.pose.pose.position.y

    # quat = dado.pose.pose.orientation
    # lista = [quat.x, quat.y, quat.z, quat.w]
    # angulos = np.degrees(transformations.euler_from_quaternion(lista))    

    # if contador % 50 == 0:
    #     print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    # contador = contador + 1
    # theta = np.radians(angulos[2])
    
class MaquinaDeEstados:

    def __init__(self):
        """
            Construtor da classe (publishers + subscribers + atributos)
        """

        #Velocidade
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        #Laser
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        #Garra  
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

        #Imagem
        self.w = 640 # comprimento lateral da tela (pixels)

        self.dif_subscriber = rospy.Subscriber('/dif', String, self.atualiza_dif)
        self.dif = -1 #diferença dada por image.py

        self.cx_creeper_subscriber = rospy.Subscriber('/cx_creeper', String, self.atualiza_cx_creeper)
        self.cx_creeper = None # posição em x do creeper

        self.angulo_linha_amarela_subscriber = rospy.Subscriber('/angulo_linha_amarelo', String, self.atualiza_angulo)
        self.angulo_linha_amarela = 0

        #Atributos lógicos
        self.pegou_creeper = False
        self.colidiu = False
        self.lado_direito = False
        self.estado = "SEGUE RETA"
        self.ultima_posicao = []
        self.posicao_atual = []

        #Funcionamento
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)
        
    #Funções de Callback
    def laser_callback(self, msg):
        """
            Cria uma lista com as distâncias encontradas por cada sensor e
            muda o self.colidiu (indica quando o robô está perto o bastante do
            creeper para acionar a garra)
        """
        laser_msg = msg.ranges
        if laser_msg[0] < 0.4 or laser_msg[1]< 0.4 or laser_msg[2]< 0.4 or laser_msg[3]< 0.4 or laser_msg[359] < 0.4 or laser_msg[358] < 0.4 or laser_msg[357] < 0.4:
            self.colidiu = True
        else:
            self.colidiu = False
    
    def atualiza_dif(self, msg):
        """
            Guarda os valores da dif dada pelo image.py
            dif = centro horizontal da tela subtraído pelo
            centro de massa horizontal da linha pontilhada da reta
        """
        self.dif = float(msg.data)

    def atualiza_cx_creeper(self,msg):
        """
            Guarda a posição do centro de massa horizontal do creeper
            dado pelo image.py em self.cx_creeper
        """
        data = None
        if msg.data is not None:
            data = float(msg.data)
        self.cx_creeper = data

    def atualiza_angulo(self, msg):
        """
            Guarda o ângulo feito com a linha amarela dado por image.py
            em self.angulo_linha_amarela
        """
        self.angulo_linha_amarela = float(msg.data)
    
    #Estados
    def segue_reta(self):
        """
            Robô segue a linha amarela utilizando controle proporcional diferencial
            Vai para o estado "PARA" se:
            1) Já tiver passado pelo lado direito (self.direito = True) e 
            2) Estiver perto da posição inicial (0,0)
            Vai para o estado "FOCA CREEPER" se:
            1) Ainda não tiver pegado o creeper (self.pegou_creeper = False) e
            2) Se o creeper estiver na direita da imagem
        """

        estado = "SEGUE RETA"        
        # Implementacao de controle proporcional diferencial
        Kp = 1/100
        Kd = 1/1000
        psi = self.angulo_linha_amarela
        #print("Psi:{}".format(psi))
        self.twist.linear.x = 0.5
        self.twist.angular.z = - Kp*(self.dif + (Kd*(- math.sin(psi))))
        
        
        #publica velocidade
        self.cmd_vel_pub.publish(self.twist)
        
        if x>0.25:
            self.lado_direito = True
        
        if self.lado_direito == True and x>-0.15 and x<0.15 and y>-0.05 and y<0.05:
            estado = "PARA"

        if self.cx_creeper is not None and self.pegou_creeper==False:
            if self.w*4//5<self.cx_creeper<self.w:
                estado = "FOCA CREEPER"
        return estado
    
    def foca_creeper(self):
        """
            Robô gira de forma a deixar o centro de massa horizontal 
            do creeper alinhado com o centro da imagem
            Vai para o estado "SEGUE CREEPER" se:
            1) O centro da imagem estar alinhado com o centro de massa do
            creeper (com uma margem de erro de 10 pixels)
        """
        estado = "FOCA CREEPER"
        self.twist.linear.x = 0
        centro = self.w//2

        if len(self.ultima_posicao) == 0:
            self.ultima_posicao.append(x)
            self.ultima_posicao.append(y)

        if self.cx_creeper is not None:
            delta = self.cx_creeper - centro
            self.twist.angular.z = -delta/100
        
        if centro - 10 < self.cx_creeper < centro + 10:
            estado = "SEGUE CREEPER"

        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()

        return estado

    def segue_creeper(self):
        """
            Robô vai para frente enquanto estiver alinhado com o 
            creeper (margem de erro de 30 pixels). Corrige
            sua trajetória no meio do caminho se estiver alinhado.
            Vai para o estado "PEGA CREEPER":
            1) Se estiver a uma distância correta do creeper
        """
        estado = "SEGUE CREEPER"
        centro = self.w//2

        delta = self.cx_creeper - centro
        self.twist.angular.z = -(delta/100)
        self.twist.linear.x = 0.1

        if self.colidiu:
            #print("Colidiu")
            self.twist.linear.x = 0
            self.twist.angular.z = 0            
            estado = "PEGA CREEPER"  
        
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()
        return estado

    def pega_creeper(self):
        """
            Levanta a garra e muda o atributo self.pegou_creeper para True
            Vai para o estado "RETORNA RETA"
        """

        estado = "RETORNA RETA"
        self.pegou_creeper = True

        self.posicao_atual.append(x)
        self.posicao_atual.append(y)
        
        self.garra.publish(-1.0)
        self.ombro.publish(-1.0)
        rospy.sleep(3.0)
        self.garra.publish(0.0) ## fecha
        self.ombro.publish(1.5) ## para cima
        rospy.sleep(3.0)
        
        return estado

    def retorna_reta(self):
        """
            Volta de ré para a reta até achar a reta de novo
        """
        # De image.py, temos:
        # self.dif = self.cx - self.w/2
        cx = self.dif + self.w/2
        # Usado para não criar outro par publisher/subscriber

        estado = "RETORNA RETA"
        self.twist.linear.x = -0.1
        self.twist.angular.z = 0
        if cx>70:
            estado = "SEGUE RETA"

        self.cmd_vel_pub.publish(self.twist)
        # distancia = math.sqrt((self.ultima_posicao[0] - self.posicao_atual[0])**2 + (self.ultima_posicao[1] - self.posicao_atual[1])**2)
        # self.cmd_vel_pub.publish(self.twist)
        # rospy.sleep(distancia//0.1)

        return estado

    def stop(self):
        """
            Para o robô permanentemente 
        """
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()
        return "PARA"

    #Controle
    def control(self):
        """
            Responsável pela alteração dos estados
        """
        
        print(self.estado)

        if self.estado=="SEGUE RETA":
            self.estado = self.segue_reta()
        
        elif self.estado=="FOCA CREEPER":
            self.estado = self.foca_creeper()
        
        elif self.estado=="SEGUE CREEPER":
            self.estado = self.segue_creeper()

        elif self.estado == "PEGA CREEPER":
            self.estado = self.pega_creeper()
        
        elif self.estado=="RETORNA RETA":
            self.estado = self.retorna_reta()

        elif self.estado=="PARA":
            self.estado = self.stop()
        
          
if __name__== "__main__":
    rospy.init_node('Controller')
    controller = MaquinaDeEstados()
    recebe_scan = rospy.Subscriber('/odom', Odometry , recebeu_leitura)

    while not rospy.is_shutdown():
        controller.control()

        

        
