#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf2_geometry_msgs

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from tf.transformations import *
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
import tf2_geometry_msgs

sphere_param_points = SphereParams()
points_recieved = False

def newPlanPoint(plan, linX, linY, linZ, angX, angY, angZ):
	plan_point = Twist()
	# just a quick solution to send two target points
	# define a point close to the initial position
	plan_point.linear.x = linX
	plan_point.linear.y = linY
	plan_point.linear.z = linZ
	plan_point.angular.x = angX
	plan_point.angular.y = angY
	plan_point.angular.z = angZ
	# add this point to the plan
	plan.points.append(plan_point)
	
def get_sphere_params(params):
	global sphere_param_points
	global points_recieved
	sphere_param_points = params
	points_recieved = True


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	#add subscriber to get param points
	pnt_sub = rospy.Subscriber('/sphere_params', SphereParams, get_sphere_params)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	q_rot = Quaternion()
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	
	while not rospy.is_shutdown():
		
		# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available')
			loop_rate.sleep()
			continue
			
			
		# extract the xyz coordinates
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		
		
		# define a testpoint in the tool frame using sphere param coordinates
		pt_in_tool = tf2_geometry_msgs.PointStamped()
		pt_in_tool.header.frame_id = 'camera_color_optical_frame'
		pt_in_tool.header.stamp = rospy.get_rostime()
	
		pt_in_tool.point.x = sphere_param_points.xc
		pt_in_tool.point.y = sphere_param_points.xc
		pt_in_tool.point.z = sphere_param_points.xc
		#pt_in_tool.point.z= 0.1 # 10 cm away from flange
		
		# convert the 3D point to the base frame coordinates
		pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
		x, y, z, rad = pt_in_base.point.x, pt_in_base.point.y, pt_in_base.point.z, sphere_param_points.radius
		
		# define a plan variable
		plan = Plan()
		#first point needs to go to ball:
		newPlanPoint(plan, x, y, z, 0, -3.0206, 1.5704)
		
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
