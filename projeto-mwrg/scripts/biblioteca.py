import numpy as np
import math
import cv2
from sklearn.linear_model import LinearRegression

# Arquivo com as funcoes utilizadas

def segmenta_azul(img):
    """
    Utiliza funcao inRange do cv2 para segmentar imagem HSV e obter uma mascara
    """
    lower_blue = np.array([90, 140, 140],dtype=np.uint8)
    upper_blue = np.array([100, 255, 255],dtype=np.uint8)
    mascara = cv2.inRange(img, lower_blue, upper_blue)
    return mascara

def segmenta_green(img):
    """
    Utiliza funcao inRange do cv2 para segmentar imagem HSV e obter uma mascara
    """
    lower_green = np.array([55, 150, 200],dtype=np.uint8)
    upper_green = np.array([65, 255, 255],dtype=np.uint8)
    mascara = cv2.inRange(img, lower_green, upper_green)
    return mascara

def segmenta_orange(img):
    """
    Utiliza funcao inRange do cv2 para segmentar imagem HSV e obter uma mascara
    """
    lower_orange1 = np.array([0, 200, 200],dtype=np.uint8)
    upper_orange1 = np.array([4, 255, 255],dtype=np.uint8)
    lower_orange2 = np.array([177, 200, 200],dtype=np.uint8)
    upper_orange2 = np.array([180, 255, 255],dtype=np.uint8)
    mascara1 = cv2.inRange(img, lower_orange1, upper_orange1)
    mascara2 = cv2.inRange(img, lower_orange2, upper_orange2)
    mascara = cv2.bitwise_or(mascara1, mascara2)
    return mascara

def segmenta_yellow(img):
    """
    Utiliza funcao inRange do cv2 para segmentar imagem HSV e obter uma mascara
    """
    lower_yellow = np.array([22, 200, 200],dtype=np.uint8)
    upper_yellow = np.array([36, 255, 255],dtype=np.uint8)
    mascara = cv2.inRange(img, lower_yellow, upper_yellow)
    return mascara

def encontra_angulo_com_vertical(mascara):
    """
    Encontra os contornos da mascara (mascara da faixa amarela do chao), encontra os centros dos contornos e faz uma regressao linear com esses centros
    para encontrar o angulo que a linha esta fazendo com a vertical para usar esse angulo no controle proporcional derivativo
    """
    angulo = 0
    contornos, _ = cv2.findContours(mascara, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    lista_x_contornos = []
    lista_y_contornos = []
    
    for c in contornos:
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
        angulo = math.atan(coef_angular)
    
    return angulo

def encontra_centro_x(mascara):
    """
    Utiliza o metodo moments da biblioteca cv2 e a express~ao do centroide para encontrar o centro em x de algum objeto segmentado na mascara
    """
    M = cv2.moments(mascara)
    if M['m00'] > 0:
        centro_x = int(M['m10']/M['m00'])
        return centro_x
    else:
        return None

def encontra_centro_y(mascara):
    """
    Utiliza o metodo moments da biblioteca cv2 e a express~ao do centroide para encontrar o centro em y de algum objeto segmentado na mascara
    """
    M = cv2.moments(mascara)
    if M['m00'] > 0:
        centro_y = int(M['m01']/M['m00'])
        return centro_y
    else:
        return None

def encontra_maior_area_de_contorno(mascara):
    """
    Encontra maior area do contorno dos contornos encontrados em uma mascara
    """
    contornos, _ = cv2.findContours(mascara, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    maior_area = 0
    for contorno in contornos:
        area = cv2.contourArea(contorno)
        if area > maior_area:
            maior_area = area

    return maior_area

def controle_proporcional_diferencial(ye, psi):
    """
    Implementacao de controle proporcional diferencial para encontrar a velocidade angular
    para fazer o robo seguir a linha amarela

    A implementacao foi baseada no seguinte modelo:
    https://www.a1k0n.net/2018/11/13/fast-line-following.html
    """
    
    Kp = 1/100
    Kd = 1/1000

    vel_angular = - Kp*(ye + (Kd*(- math.sin(psi))))

    return vel_angular