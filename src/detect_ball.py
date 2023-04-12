#!/usr/bin/env python3
#imports
import rospy
import numpy as np
import roscpp
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")


#get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	#convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	#raise flag that the image was recieved
	img_received = True

	
if __name__ == '__main__':
	#define node, subscribers, and publishers
	rospy.init_node('detect_ball', anonymous = True)
	#define a subscriber to ream images
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
	#define a publisher to publish images to /ball_2D task
	img_pub = rospy.Publisher('/ball_2D', Image, queue_size = 1)
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			#convert rbg to hsv
			hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
			#define the lower ranges for the yellow mask
			lower_yellow_rgb = np.array([20,1,65])
			#define the upper ranges for the yellow mask
			upper_yellow_rgb = np.array([90,255,255])
			#create the yellow mask from those upper and lower ranges
			mask = cv2.inRange(hsv, lower_yellow_rgb, upper_yellow_rgb)
			#create a rectangle inside the image to reduce the size of area being affected by the mask
			rgb_rect = np.zeros((720, 1280), dtype = "uint8")
			rectangle = cv2.rectangle(rgb_rect, (200,120), (850, 600), 255, -1)
			#use bitwise_and to combine the mask and the rectangle so the mask is only applied the the areas in the rectangle
			mono_img = cv2.bitwise_and(rectangle, mask)
			#convert img to ros msg and publish, encode it as mono8 to be black/white
			img_msg = CvBridge().cv2_to_imgmsg(mono_img, encoding = "mono8")
			# publish the image
			img_pub.publish(img_msg)
		# pause until the next iteration			
		rate.sleep()



