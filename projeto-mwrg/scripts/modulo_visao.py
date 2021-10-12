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


def processa(frame):
    '''Use esta funcao para basear o processamento do seu robo'''

    img, resultados = mnet.detect(frame)

    centro = (frame.shape[1]//2, frame.shape[0]//2)

    return centro, img, resultados
