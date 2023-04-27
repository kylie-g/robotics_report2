#!/usr/bin/env python3
#code based off of: https://github.com/hsaeidi-uncw/ur5e_control.git and https://github.com/hsaeidi-uncw/robot_control_lectures/tree/main/scripts
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
from std_msgs.msg import UInt8

sphere_param_points = SphereParams()
points_recieved = False
tool_pose_points = Twist()
tool_pose_recieved = False
manualinit = True

#get toolpose to get manual initialization coords
def get_tool_pose(data):
	global tool_pose_points
	global tool_pose_recieved
	global manualinit
	#if it is the manual initialization points
	if manualinit:
		tool_pose_points.linear.x = data.linear.x
		tool_pose_points.linear.y = data.linear.y
		tool_pose_points.linear.z = data.linear.z
		tool_pose_points.angular.x = data.angular.x
		tool_pose_points.angular.y = data.angular.y
		tool_pose_points.angular.z = data.angular.z
	tool_pose_recieved = True
	manualinit = False
		

#from phase 1 code
def newPlanPoint(plan, linX, linY, linZ, angX, angY, angZ, mode):
	plan_point = Twist()
	point_mode = UInt8()
	# just a quick solution to send two target points
	# define a point close to the initial position
	plan_point.linear.x = linX
	plan_point.linear.y = linY
	plan_point.linear.z = linZ
	plan_point.angular.x = angX
	plan_point.angular.y = angY
	plan_point.angular.z = angZ
	point_mode.data = mode
	# add this point to the plan
	plan.points.append(plan_point)
	plan.modes.append(point_mode)

#get the sphere params from the sphereparams subscriber and set to true if points are recieved
def get_sphere_params(params):
	global sphere_param_points
	global points_recieved
	sphere_param_points = params
	points_recieved = True

#checks to see if the movement should be tracked or not
movement_tracker = False
def track_movement(data):
	global movement_tracker
	movement_tracker = data.data


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	#add subscriber to get param points
	pnt_sub = rospy.Subscriber('/sphere_params', SphereParams, get_sphere_params)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	#get toolpose points of manual initialization
	pnt_sub = rospy.Subscriber('/ur5e/toolpose', Twist, get_tool_pose)
	
	#subscriber to get information on movement for tracking
	mvnt_sub = rospy.Subscriber('/movement_tracker', Bool, track_movement)
	
	#CODE FROM  https://github.com/hsaeidi-uncw/ur5e_control.git
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	
	while not rospy.is_shutdown():
		#CODE FROM  https://github.com/hsaeidi-uncw/ur5e_control.git
		# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available')
			loop_rate.sleep()
			continue
		
		#CODE MODIFIED FROM https://github.com/hsaeidi-uncw/ur5e_control.git
		# define a testpoint in the tool frame using sphere param coordinates
		pt_in_tool = tf2_geometry_msgs.PointStamped()
		pt_in_tool.header.frame_id = 'camera_color_optical_frame'
		pt_in_tool.header.stamp = rospy.get_rostime()
		# set those testpoints to the param coords
		pt_in_tool.point.x = sphere_param_points.xc
		pt_in_tool.point.y = sphere_param_points.yc
		pt_in_tool.point.z = sphere_param_points.zc
		
		
		#CODE MODIFIED FROM https://github.com/hsaeidi-uncw/ur5e_control.git
		# convert the 3D point to the base frame coordinates
		pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
		#set x, y, z to the base frame coordinates and rad to the sphere params
		ball_x, ball_y, ball_z, ball_rad = pt_in_base.point.x, pt_in_base.point.y, pt_in_base.point.z, sphere_param_points.radius
		print('Initizalized frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
		print('BASE frame:  x= ', format(ball_x, '.3f'), '(m), y= ', format(ball_y, '.3f'), '(m), z= ', format(ball_z, '.3f'),'(m)', format(ball_rad, '.3f'))
		
		#manual init coords
		lin_x = tool_pose_points.linear.x
		lin_y = tool_pose_points.linear.y
		lin_z = tool_pose_points.linear.z
		ang_x = tool_pose_points.angular.x
		ang_y = tool_pose_points.angular.y
		ang_z = tool_pose_points.angular.z
		print('toolpose frame:  x= ', format(lin_x, '.3f'), '(m), y= ', format(lin_y, '.3f'), '(m), z= ', format(lin_z, '.3f'),'(m)', format(ang_x, '.3f'))
		
		#MODIFIED FROM PHASE 1
		# define a plan variable
		plan = Plan()
		offset = .025
		
		#first point: starting point - initialzied and manual initialization
		newPlanPoint(plan, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z, 0)
		#second point: above ball
		newPlanPoint(plan, ball_x, ball_y, lin_z, ang_x, ang_y, ang_z, 0)
		#third point: where the ball is located
		newPlanPoint(plan, ball_x, ball_y, ball_z + offset, ang_x, ang_y, ang_z, 0)
		#fourth point: close grippers
		newPlanPoint(plan, ball_x, ball_y, ball_z + offset, ang_x, ang_y, ang_z, 2)
		#fifth point: striaght up from where the ball is located (lifting the ball)
		newPlanPoint(plan, lin_x, ball_y, lin_z, ang_x, ang_y, ang_z, 0)
		#sixth point: point where the ball is to be set down
		newPlanPoint(plan, lin_x, lin_y, ball_z+offset, ang_x, ang_y, ang_z, 0)
		#seventh point: open gripper
		newPlanPoint(plan, lin_x, lin_y, ball_z+offset, ang_x, ang_y, ang_z, 1)
		#eigth point: back to start
		newPlanPoint(plan, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z, 0)
		
		
		#traker checker: if it is true, let the user know it has stopped moving, otherwise publish as normal
		if movement_tracker:
			print("motion has been stopped temporarly")
		else:
			print("movement continued")
			plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
