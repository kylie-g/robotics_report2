#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import math
from sensor_msgs.msg import Image
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

msg_received = False

def get_points(XYZpoints):
	#initialize ros_points and msg_received
	global ros_points
	global msg_received
	ros_points = []
	XYZcords = []
	#loop through and recieve all the points
	for i in XYZpoints.points:
		XYZcords = [i.x, i.y, i.z]
		ros_points.append(XYZcords)
	msg_received = True
	
	#print(ros_points[0])
	
	#maybe move into one function?
def getRadiusParams(ros_points):
	#initialize A and B based on the example
	B = []
	A = []
	#calculate each x, y, z value based on slide 21 to get matricies for further calucations
	for p in ros_points:
		B.append(math.pow(p[0], 2) + math.pow(p[1], 2) + math.pow(p[2], 2))
		A.append([2*p[0], 2*p[1], 2*p[2], 1])
		
	#fit a and b to calc results
	B_array = np.array(B).reshape(len(B), 1)
	A_array = np.array(A).reshape(len(B), 4)
	AB = np.linalg.lstsq(A_array, B_array, rcond=None)
	
	#calc radius using r = sqrt(P[3] + xc^2 + yx^2 + zc^2)
	radius = np.sqrt(AB[0][3] + AB[0][0]**2 + AB[0][1]**2 + AB[0][2]**2)
	
	#set the sphereParams to xc, yc, zc, and radius
	sphParams = SphereParams(float(AB[0][0]), float(AB[0][1]), float(AB[0][2]), radius)
	return sphParams
	
	
def filterVal(value, fil_out, fil_gain):
	
	fil_in = value
	print('fil_in', fil_in, ' fil_out now:', fil_out)
			
	fil_out = (fil_gain*fil_in) + (1-fil_gain)*fil_out
	print('fil_out_next: ', fil_out)
	
	return(fil_out)
	
		

if __name__ == '__main__':
	#define node, subscribers, and publishers
	rospy.init_node('sphere_fit', anonymous = True)
	#define a subscriber to recieve the points
	pnt_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_points) 
	#get points
	#define a publisher to publish images to /sphere_params task and therefore set the radius
	pnt_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 1)
	# set the loop frequency
	rate = rospy.Rate(10)
	
	x_fil_out =  -0.015
	y_fil_out =  -0.015
	z_fil_out =  0.47
	rad_fil_out = 0.04
	
	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if msg_received:
		#call get radius params to get params
			param = getRadiusParams(ros_points)
			
			#filter params
			param.xc = filterVal(param.xc, x_fil_out, 0.05)
			x_fil_out = param.xc
			param.yc = filterVal(param.yc, y_fil_out, 0.05)
			y_fil_out = param.yc
			param.zc = filterVal(param.zc, z_fil_out, 0.05)
			z_fil_out = param.zc
			param.radius = filterVal(param.radius, rad_fil_out, 0.05*5)
			rad_fil_out = param.radius
			
			# publish the param
			pnt_pub.publish(param)
		# pause until the next iteration			
		rate.sleep()
