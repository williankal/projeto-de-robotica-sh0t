#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
from numpy.lib.twodim_base import _trilu_dispatcher
import rospy
import numpy as np
import math
import cv2
import cv2.aruco as aruco
import rospkg
import time
import argparse
import os
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class Mobile_net:
    
    def __init__(self):
        rospy.init_node('follower')

        self.rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.path = self.rospack.get_path('ros_projeto')
        path = self.path
        self.scripts = os.path.join(path,  "scripts")
        self.image1 = None
        self.proto = os.path.join(self.scripts,"MobileNetSSD_deploy.prototxt.txt")
        self.model = os.path.join(self.scripts, "MobileNetSSD_deploy.caffemodel")
        self.confianca = 0.5
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.detect, queue_size=4, buff_size = 2**24)
        
        
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
        "sofa", "train", "tvmonitor"]
        
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        self.net = cv2.dnn.readNetFromCaffe(self.proto, self.model)

    def detect(self, frame):

        try:
            image1 = self.bridge.compressed_imgmsg_to_cv2(frame,desired_encoding='bgr8')
            (h, w) = image1.shape[:2]
            blob = cv2.dnn.blobFromImage(cv2.resize(image1, (300, 300)), 0.007843, (300, 300), 127.5)

            # pass the blob through the network and obtain the detections and
            # predictions
            # print("[INFO] computing object detections...")
            self.net.setInput(blob)
            detections = self.net.forward()
            results = []


            # loop over the detections
            for i in np.arange(0, detections.shape[2]):
                # extract the confidence (i.e., probability) associated with the
                # prediction
                confidence = detections[0, 0, i, 2]

                # filter out weak detections by ensuring the `confidence` is
                # greater than the minimum confidence


                if confidence > self.confianca:
                    # extract the index of the class label from the `detections`,
                    # then compute the (x, y)-coordinates of the bounding box for
                    # the object
                    idx = int(detections[0, 0, i, 1])
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    # display the prediction
                    label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
                    #print("[INFO] {}".format(label))
                    cv2.rectangle(image1, (startX, startY), (endX, endY),
                        self.COLORS[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(image1, label, (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)

                    results.append((self.CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))
                    print(results)
            cv2.imshow('mobile', image1)

        except CvBridgeError as e:
            print('ex', e)

def main(args):
  mn = Mobile_net()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)