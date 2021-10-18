#!/usr/bin/env python3
# -*- coding:utf-8 -*-


#   exemplo adaptado do livro:
#   
#  Programming Robots with ROS.
#  A Practical Introduction to the Robot Operating System
#  Example 12-5. follower_p.py pag265
#  
#  Referendia PD:https://github.com/martinohanlon/RobotPID/blob/master/mock/mock_robot_pd.py 

import rospy
import numpy as np
import math
import cv2
import time
import sys
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class Follower:

    def __init__(self):
        
        self.bridge = CvBridge()
        self.cv_image = None
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                            Twist, 
                                            queue_size=1)
        
        self.laser_subscriber = rospy.Subscriber('/scan',
                                                 LaserScan, 
			                                    self.laser_callback)
                                                
        # self.imagem_subscriber = rospy.Subscriber('/image', String, self.control)

        self.dif_subscriber = rospy.Subscriber('/dif', String, self.atualiza_dif)
        
        self.twist = Twist()
        self.laser_msg = LaserScan()
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1
        self.dif = -1

        self.lastError = 0
        self.max_vel_linear = 0.2
        self.max_vel_angular = 2.0
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)

    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser(self, pos):
        return self.laser_msg.ranges[pos]

    def atualiza_dif(self, msg):
        self.dif = float(msg.data)
        print(msg.data)
    
    def control(self):
        ### BEGIN CONTROL
        
        #------controle P simples--------------------
    
        self.twist.linear.x = 0.2
        self.twist.angular.z = - self.dif / 100

        ### END CONTROL
        #publica velocidade
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("linear: %f angular: %f", self.twist.linear.x, self.twist.angular.z)
        self.rate.sleep()
    
    

# Main loop
if __name__== "__main__":
    rospy.init_node('MandaNodes')
    follower = Follower()

    while not rospy.is_shutdown():
        follower.control()
        
