#! /usr/bin/env python

import roslib; roslib.load_manifest('planner_msgs')
import rospy
import time
# Brings in the SimpleActionClient
import actionlib
from geometry_msgs.msg import Pose2D
from planner_msgs.msg import goal, GoToGoal, GoToAction



def purepursuit_client():
	# Creates the SimpleActionClient, passing the type of the action
	# (GoTo) to the constructor.
	client = actionlib.SimpleActionClient('purepursuit_planner', GoToAction)

	# Waits until the action server has started up and started listening for goals.
	client.wait_for_server()
	
	g = GoToGoal(target = [goal(pose = Pose2D(-2.0, 0.86, 0.0), speed = 0.2), goal(pose = Pose2D(-1.0, 0.46, 0.0), speed = 0.2), 
	goal(pose = Pose2D(0.0, 0.0, 0.0), speed = 0.2), goal(pose = Pose2D(1.0, 0.0, 0.0), speed = 0.2), goal(pose = Pose2D(2.54, 1.0, 0.0), speed = 0.2),
	goal(pose = Pose2D(4.96, 2.18, 0.0), speed = 0.2), goal(pose = Pose2D(8.54, 2.22, 0.0), speed = 0.2)])
	
	rospy.loginfo('Sending goal')
	# Sends the goal to the action server.
	client.send_goal(g)
	#print 'Sleeping'
	#time.sleep(4)
	#print 'Canceling goal'
	#client.cancel_goal()
	# Waits for the server to finish performing the action.
	client.wait_for_result()

	# Prints out the result of executing the action
	return client.get_result() 

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('planner_test_client')
		result = purepursuit_client()
		rospy.loginfo( 'Result: %d'%(result.route_result))
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
