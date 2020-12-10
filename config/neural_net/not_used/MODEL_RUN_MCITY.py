#!/usr/bin/env python

import datetime
import os
import cv2
import time
import rospy
import sys
import numpy as np
from mirvehicle_msgs.msg import Control
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from image_converter import ImageConverter
from drive_run import DriveRun
from config import Config
from image_process import ImageProcess
#xMin = 0
#yMin = 380
#xMax = 800
#yMax = 800

class NeuralControl:
    def __init__(self):
        rospy.init_node('controller')
        self.ic = ImageConverter()
        self.image_process = ImageProcess()
        self.rate = rospy.Rate(30)
        self.drive= DriveRun(sys.argv[1])
        #rospy.Subscriber('/bolt/front_camera/image_raw', Image, self.controller_cb)
	rospy.Subscriber('/stereo_camera/image_rect_color', Image, self.controller_cb)
        self.image = None
        self.image_processed = False
        self.config = Config()

    def controller_cb(self, image): 
        img = self.ic.imgmsg_to_opencv(image)
        cropImg = img[self.config.capture_area[1]:self.config.capture_area[3],
                      self.config.capture_area[0]:self.config.capture_area[2]]
	#cropImg = img[yMin:yMax,xMin:xMax]
        img = cv2.resize(cropImg,(self.config.image_size[0],
                                  self.config.image_size[1]))
        self.image = self.image_process.process(img)

        if self.config.typeofModel == 4 or self.config.typeofModel == 5:
            self.image = np.array(self.image).reshape(1, self.config.image_size[1],
                                                         self.config.image_size[0],
                                                         self.config.image_size[2])
        self.image_processed = True


if __name__ == "__main__":
    try:
        neural_control = NeuralControl()
        while not rospy.is_shutdown():
            if neural_control.image_processed == True:
                prediction = neural_control.drive.run(neural_control.image)
		joy_pub = rospy.Publisher('/bolt', Control, queue_size = 10)
	        rate = rospy.Rate(30)
      	  	joy_data = Control()
                joy_data.steer = prediction
                joy_data.throttle = 0.3
        	joy_pub.publish(joy_data)
                print(prediction)
                neural_control.image_processed = False
                neural_control.rate.sleep()

    except KeyboardInterrupt:
	   print ('\nShutdown requested. Exiting...')
