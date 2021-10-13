#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import mobilenet as mnet
from sklearn.linear_model import LinearRegression, RANSACRegressor


def calcular_angulo_com_vertical(img, lm):
    """Não mude ou renomeie esta função
        deve receber uma imagem contendo uma reta, além da reggressão linear e determinar o ângulo da reta com a vertical, utilizando o metodo preferir.
    """
    # Utiliza o próprio coeficiente angular obtido pelo modelo RANSACRegressor para se calcular o ângulo entre a reta e a vertical.
    # Como estamos escrevendo x em função de y, esse coeficiente angular é dado como Δx/Δy.
    # Por esse motivo, calculando o arcotangente deste coeficiente angular já nos possibilita determinar o valor entre a reta e a vertical.
    coef_angular = lm.coef_
    angulo_rad = math.atan(coef_angular)
    angulo = math.degrees(angulo_rad)
    
    return angulo

def regressao_por_centro(bgr, x_array, y_array):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e estimar a melhor reta, utilizando o metodo preferir, que passa pelos centros. Retorne a imagem com a reta e os parametros da reta
        
        Dica: cv2.line(img,ponto1,ponto2,color,2) desenha uma linha que passe entre os pontos, mesmo que ponto1 e ponto2 não pertençam a imagem.
    """
    img = bgr.copy()
    # Como vamos classificar linhas quase verticais não podemos usar a regressão convencional.
    # Por isso, em vez de escrevermos os valores de y em função de x, escreveremos x em função de y.
    yr = y_array.reshape(-1,1) # Entradas do modelo
    xr = x_array.reshape(-1,) # Saídas do modelo
    
    # Utilizando o modelo RANSACRegressor para remover pontos "outliers" (necessário para casos como o da imagem em img/frame03.jpg)
    reg = LinearRegression()
    ransac = RANSACRegressor(reg)
    ransac.fit(yr, xr)
    lm = ransac.estimator_
    coef_angular, coef_linear = lm.coef_, lm.intercept_

    # Pega o menor e o maior valor de y 
    y_min = int(min(y_array)) # Precisa ser int para plotar na imagem
    y_max = int(max(y_array)) 
    # Aplica a equação da reta, utilizando o coeficiente angular e linear obtidos pelo
    # modelo RANSACRegressor para descobrir os valores de x correspondes aos valores de y encontrados
    x_min = int(coef_angular*y_min + coef_linear)
    x_max = int(coef_angular*y_max + coef_linear)
    # Desenha uma linha entre as coordenadas obtidas
    cv2.line(img, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

    return img, lm

def crosshair(img, point, size, color):    
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)

def processa(frame):
    '''Use esta funcao para basear o processamento do seu robo'''

    img, resultados = mnet.detect(frame)

    centro = (frame.shape[1]//2, frame.shape[0]//2)

    return centro, img, resultados

def processa_imagem(frame):

    img = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    img_contornos = frame.copy()


    hsv1 = np.array([20, 150, 150])
    hsv2 = np.array([30, 255, 255])

    mask = cv2.inRange(hsv, hsv1, hsv2)

    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5, 5)))

    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img_contornos, contornos, -1, [0, 0, 255], 2)

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
        except:
            pass
    
    if X:
        ## Regressão pelo centro
        X = np.array(X)
        Y = np.array(Y)
        img, lm = regressao_por_centro(img, X,Y)
        print("Angulo = %s"%calcular_angulo_com_vertical(img, lm))

    return img