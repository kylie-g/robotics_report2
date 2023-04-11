#!/usr/bin/env python3

import rospy
import math

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist

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


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# define a plan variable
	plan = Plan()
	#first point:
	newPlanPoint(plan, -0.7925, -0.133, 0.3634, 0, -3.0206, 1.5704)
	#second point:
	newPlanPoint(plan, -0.7925, -0.133, 0.10, 0, -3.0206, 1.5704)
	#third point:
	newPlanPoint(plan, -0.7925, 0.3, 0.3634, 0, -3.0206, 1.5704)
	#fourth point:
	newPlanPoint(plan, -0.7925, 0.3, 0.10, 0, -3.0206, 1.5704)
	

	
	
	while not rospy.is_shutdown():
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
