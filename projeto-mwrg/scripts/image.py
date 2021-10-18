#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import rospy
import numpy as np
import math
import cv2
import time
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class Image_converter:

    def __init__(self):
        rospy.init_node('follower')

        self.bridge = CvBridge()
        self.cv_image = None
        self.image_sub = rospy.Subscriber('/camera/image/compressed',
                                            CompressedImage, 
                                            self.image_callback, 
                                            queue_size=4, 
                                            buff_size = 2**24)
        self.publica_dif = rospy.Publisher('/dif', String, queue_size=10)
        self.cx = -1
        self.cy = -1
        self.dif = -1


    def image_callback(self, msg):
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([22, 50, 50],dtype=np.uint8)
            upper_yellow = np.array([36, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            h, w, d = cv_image.shape
            search_top = 3*h//4
            search_bot = 3*h//4 + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            self.w = w
            self.h = h

            M = cv2.moments(mask)

            if M['m00'] > 0:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (self.cx, self.cy), 10, (0,0,255), -1)  
                
            self.dif = self.cx - self.w/2

            self.publica_dif.publish(str(self.dif))
            cv2.imshow("window", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print('ex', e)
    
    def centraliza_creeper(self, cor):
        try:
            if cor == "blue":
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                cor1 = np.array([86, 50, 50],dtype=np.uint8)
                cor2 = np.array([96, 255, 255],dtype=np.uint8)
                mask = cv2.inRange(hsv, cor1, cor2)

            if cor == "green":
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                cor1 = np.array([50, 50, 50],dtype=np.uint8)
                cor2 = np.array([60, 255, 255],dtype=np.uint8)
                mask = cv2.inRange(hsv, cor1, cor2)

            if cor == "orange":
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                cor1 = np.array([170, 50, 50],dtype=np.uint8)
                cor2 = np.array([10, 255, 255],dtype=np.uint8)
                mask = cv2.inRange(hsv, cor1, cor2)            

            M = cv2.moments(mask)

            if M['m00'] > 0:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
        except: 
            pass









def main(args):
  ic = Image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)