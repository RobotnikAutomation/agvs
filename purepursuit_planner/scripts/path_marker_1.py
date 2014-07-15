#!/usr/bin/env python

"""
Copyright (c) 2014, Robotnik Automation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""


import roslib; roslib.load_manifest("interactive_markers")
import rospy
import copy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarker
from interactive_markers.menu_handler import *

import actionlib
from geometry_msgs.msg import Pose2D
from purepursuit_planner.msg import goal, GoToGoal, GoToAction

# Client based on ActionServer to send goals to the purepursuit node
class PurepursuitClient():
	
	def __init__(self, planner_name):
		self.planner_name = planner_name
		# Creates the SimpleActionClient, passing the type of the action
		# (GoTo) to the constructor.
		self.client = actionlib.SimpleActionClient(planner_name, GoToAction)

	## @brief Sends the goal to 
	## @return 0 if OK, -1 if no server, -2 if it's tracking a goal at the moment
	def goTo(self, goal_list):
		# Waits until the action server has started up and started
		# listening for goals.
		if self.client.wait_for_server(timeout = rospy.Duration(3.0) ):
			#if self.getState() != GoalStatus.LOST:
			#	rospy.loginfo('PurepursuitClient: planner is tracking a goal')
			#	return -2
				
			g = GoToGoal(target = goal_list)
			rospy.loginfo('PurepursuitClient: Sendig %d waypoints'%(len(goal_list)))
			self.client.send_goal(g)
			return 0
		else:
			rospy.logerr('PurepursuitClient: Error waiting for server')
			return -1
	
	## @brief cancel the current goal
	def cancel(self):		
		rospy.loginfo('PurepursuitClient: cancelling the goal')
		self.client.cancel_goal()
	
	## @brief Get the state information for this goal
    ##
    ## Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED,
    ## PREEMPTED, ABORTED, SUCCEEDED, LOST.
    ##
    ## @return The goal's state. Returns LOST if this
    ## SimpleActionClient isn't tracking a goal.
	def getState(self):
		return self.client.get_state()
	
	## @brief Returns ret if OK, otherwise -1
	def getResult(self):
		ret = self.client.get_result()
		if not ret:
			return -1
		
		else:
			return ret

## @brief Class to manage  the creation of a Waypoint base on InteractiveMarker
class PointPath(InteractiveMarker):
	
	def __init__(self, frame_id, name, description, is_manager = False):
		InteractiveMarker.__init__(self)
		
		self.header.frame_id = frame_id
		self.name = name
		self.description = description
		
		self.marker = Marker()
		self.marker.type = Marker.CYLINDER
		self.marker.scale.x = 0.1
		self.marker.scale.y = 0.1
		self.marker.scale.z = 0.4
		self.marker.pose.position.z = 0.20
		if is_manager:
			self.marker.color.r = 0.8
			self.marker.color.g = 0.0
			self.marker.color.b = 0.0
			self.marker.color.a = 0.5
		else:
			self.marker.color.r = 0.0
			self.marker.color.g = 0.8
			self.marker.color.b = 0.0
			self.marker.color.a = 0.5
			
		self.marker_control = InteractiveMarkerControl()
		self.marker_control.always_visible = True
		self.marker_control.orientation.w = 1
		self.marker_control.orientation.x = 0
		self.marker_control.orientation.y = 1
		self.marker_control.orientation.z = 0
		self.marker_control.markers.append( self.marker )
		self.marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
		
		self.controls.append( self.marker_control )
		
	## @brief method called every time that an interaction is received	
	def processFeedback(self, feedback):
		#p = feedback.pose.position
		self.pose = feedback.pose
		#print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)
		

## @brief Manages the creation of waypoints and how to send them to Purepursuit
class PointPathManager(InteractiveMarkerServer):
	
	def __init__(self, name, frame_id, planner):
		InteractiveMarkerServer.__init__(self, name)
		self.list_of_points = []
		self.frame_id = frame_id
		self.counter_points = 0
		
		# Menu handler to create a menu
		self.menu_handler = MenuHandler()
		h_first_entry = self.menu_handler.insert( "Waypoints" )
		entry = self.menu_handler.insert( "Create New", parent=h_first_entry, callback=self.newPointCB)
		entry = self.menu_handler.insert( "Delete last", parent=h_first_entry, callback=self.deletePointCB );
		entry = self.menu_handler.insert( "Delete all", parent=h_first_entry, callback=self.deleteAllPointsCB );
		h_second_entry = self.menu_handler.insert( "Path" )
		entry = self.menu_handler.insert( "Go", parent=h_second_entry, callback=self.startRouteCB)	# Send the path from the first point to the last one
		entry = self.menu_handler.insert( "Stop", parent=h_second_entry, callback=self.stopRouteCB)	# Stops the current path 
		entry = self.menu_handler.insert( "Go back", parent=h_second_entry, callback=self.reverseRouteCB) # Sends the path from the last point to the first one
		
		# Creates the first point
		#self.list_of_points.append(PointPath(frame_id, 'p1', 'p1'))
		self.initial_point = PointPath(frame_id, 'PointManager', 'PointManager', True)
		self.insert(self.initial_point, self.initial_point.processFeedback)
		
		self.menu_handler.apply( self, self.initial_point.name )
		self.applyChanges()
		
		self.planner_client = PurepursuitClient(planner)
		
		#rospy.Timer(rospy.Duration(5), self.createNewPoint)
		
	
	## @brief Creates a new PointPath and save it a list
	def createNewPoint(self):
					
		##print 'Creating new point %d'%(self.counter_points)
		new_point = PointPath(self.frame_id, 'p%d'%(self.counter_points), 'p%d'%(self.counter_points))
		#new_point = PointPath(self.frame_id, '1', '1')
		
		if len(self.list_of_points) > 1:
			new_point.pose.position.x = self.list_of_points[self.counter_points-1].pose.position.x
			new_point.pose.position.y = self.list_of_points[self.counter_points-1].pose.position.y
		elif len(self.list_of_points) == 1:
			new_point.pose.position.x = self.list_of_points[0].pose.position.x
			new_point.pose.position.y = self.list_of_points[0].pose.position.y
	
		new_point.pose.position.x = new_point.pose.position.x  + 1.0
			
		
		#print 'Creating new point at position %.2lf, %.2lf, %.2lf'%(new_point.pose.position.x, new_point.pose.position.y, new_point.pose.position.z)
		
		self.list_of_points.append(new_point)
		self.insert(new_point, new_point.processFeedback)
		self.menu_handler.apply( self, 'p%d'%(self.counter_points) )
		self.applyChanges()
		self.counter_points = self.counter_points + 1
		
		#for i in self.list_of_points:
		#	print 'Point %s: %.2lf, %.2lf, %.2lf'%(i.name, i.pose.position.x, i.pose.position.y, i.pose.position.z)
			
		return
	
	## @brief Callback called to create a new poing	
	def newPointCB(self, feedback):
		#print 'newPointCB'
		self.createNewPoint()
		
	## @brief Callback called to create a new poing	
	def deletePointCB(self, feedback):
		if self.counter_points > 0:
			 p = self.list_of_points.pop()
			 self.counter_points = self.counter_points - 1
			 self.erase(p.name)
			 self.applyChanges()
			 
		 #print 'deletePointCB'	
	
	## @brief Callback called to create a new poing	
	def deleteAllPointsCB(self, feedback):
		for i in range(len(self.list_of_points)):
			p = self.list_of_points.pop()
			self.counter_points = self.counter_points - 1
			self.erase(p.name)
			 
		self.applyChanges()
			 
		#print 'deleteAllPointsCB'	
	
	## @brief Starts the route
	def startRouteCB(self, feedback):
		goals = self.convertListOfPointPathIntoGoal()
		print 'goals: %s'%(goals)
		self.planner_client.goTo(goals)
		return
	
	## @brief Starts the route on the inverse direction
	def reverseRouteCB(self, feedback):
		goals = self.convertListOfPointPathIntoGoal(inverse = True)
		#print 'goals: %s'%(goals)
		self.planner_client.goTo(goals)
		return
		
	## @brief Stops the current route if it's started
	def stopRouteCB(self, feedback):
		self.planner_client.cancel()
		return
	
	## @brief Starts the route (inverse order of waypoints)
	def convertListOfPointPathIntoGoal(self, inverse = False):
		converted_list = []
		if inverse:
			for i in reversed(self.list_of_points):
				converted_list.append(goal(pose = Pose2D(i.pose.position.x, i.pose.position.y, 0.0), speed = 0.4)) # For now speed constant
		else:
			for i in self.list_of_points:
				#print 'convertListOfPointPathIntoGoal: %.2lf, %.2lf'%(i.pose.position.x, i.pose.position.y)
				converted_list.append(goal(pose = Pose2D(i.pose.position.x, i.pose.position.y, 0.0), speed = 0.4)) # For now speed constant
		
		return converted_list
		
if __name__=="__main__":
	rospy.init_node("agvs_path_maker")
	
	server = PointPathManager('agvs_path_maker', frame_id = '/map', planner = 'purepursuit_planner')
	
	rospy.spin()

