import numpy as np
import math
import cv2
from sklearn.linear_model import LinearRegression

# Arquivo com as funcoes utilizadas

def segmenta_blue(img):
    """
    Utiliza função inRange do cv2 para segmentar imagem HSV e obter uma máscara
    """
    lower_blue = np.array([90, 140, 140],dtype=np.uint8)
    upper_blue = np.array([100, 255, 255],dtype=np.uint8)
    mascara = cv2.inRange(img, lower_blue, upper_blue)
    return mascara

def segmenta_green(img):
    """
    Utiliza função inRange do cv2 para segmentar imagem HSV e obter uma máscara
    """
    lower_green = np.array([55, 150, 200],dtype=np.uint8)
    upper_green = np.array([65, 255, 255],dtype=np.uint8)
    mascara = cv2.inRange(img, lower_green, upper_green)
    return mascara

def segmenta_orange(img):
    """
    Utiliza função inRange do cv2 para segmentar imagem HSV e obter uma máscara
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
    Utiliza função inRange do cv2 para segmentar imagem HSV e obter uma máscara
    """
    lower_yellow = np.array([22, 200, 200],dtype=np.uint8)
    upper_yellow = np.array([36, 255, 255],dtype=np.uint8)
    mascara = cv2.inRange(img, lower_yellow, upper_yellow)
    return mascara

def encontra_centro_x(M):
    """
    A partir da utilização do método "moments" da biblioteca cv2, utiliza a expressão do centróide para encontrar o centro em x de algum objeto segmentado na máscara
    """
    if M['m00'] > 0:
        centro_x = int(M['m10']/M['m00'])
        return centro_x
    else:
        return None

def encontra_centro_y(M):
    """
    A partir da utilização do método "moments" da biblioteca cv2, utiliza a expressão do centróide para encontrar o centro em y de algum objeto segmentado na máscara
    """
    if M['m00'] > 0:
        centro_y = int(M['m01']/M['m00'])
        return centro_y
    else:
        return None

def encontra_angulo_com_vertical(mascara):
    """
    Encontra os contornos da máscara (máscara da faixa amarela do chão), encontra os centros dos contornos e faz uma regressão linear com esses centros
    para encontrar o angulo que a linha está fazendo com a vertical para usar esse ângulo no controle proporcional derivativo
    """
    angulo = 0
    # Obtém contornos
    contornos, _ = cv2.findContours(mascara, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    lista_centros_x_contornos = []
    lista_centros_y_contornos = []
    
    for c in contornos:
        M_angulo = cv2.moments(c)
        # Obtém centros em x e em y dos contornos
        lista_centros_x_contornos.append(encontra_centro_x(M_angulo))
        lista_centros_y_contornos.append(encontra_centro_y(M_angulo))
    
    if lista_centros_x_contornos:
        # Transforma em array
        array_centros_x_contornos = np.array(lista_centros_x_contornos)
        array_centros_y_contornos = np.array(lista_centros_y_contornos)

        # Preparando dados dos centros dos contornos coletados
        # para utilizar um modelo de regressão linear
        yr = array_centros_y_contornos.reshape(-1,1)
        xr = array_centros_x_contornos.reshape(-1)

        reg = LinearRegression()
        reg.fit(yr,xr)
    
        # Pega o ângulo com a vertical para usar no controle proporcional diferencial
        coef_angular = reg.coef_
        angulo = math.atan(coef_angular)
    
    return angulo

def encontra_maior_area_de_contorno(mascara):
    """
    Encontra maior área do contorno dentre os contornos encontrados em uma máscara
    """
    # Encontra contornos
    contornos, _ = cv2.findContours(mascara, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
    # Loop para encontrar a maior área dentre os contornos obtidos
    maior_area = 0
    for contorno in contornos:
        area = cv2.contourArea(contorno)
        if area > maior_area:
            maior_area = area

    return maior_area

def controle_proporcional_diferencial(ye, psi):
    """
    Implementação de controle proporcional diferencial para encontrar a velocidade angular
    para fazer o robô seguir a linha amarela

    A implementação foi baseada no seguinte modelo:
    https://www.a1k0n.net/2018/11/13/fast-line-following.html
    """
    # Constantes
    Kp = 1/100
    Kd = 1/1000

    vel_angular = - Kp*(ye + (Kd*(- math.sin(psi))))

    return vel_angular
