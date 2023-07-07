#!/usr/bin/env python3
"""
	File Name: target_estimation.py
	Description: Just move to the manual targets
	Name: Allen Emmanuel Binny
"""
# ros includes
import rospy
import actionlib
from autoracing_msgs.msg import AutoRacingAction
from autoracing_msgs.msg import AutoRacingGoal
from autoracing_msgs.msg import Target
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Empty, EmptyResponse

# other
import numpy as np
from typing import final

class scotsActionClient:
	"""docstring for scotsActionClient"""
	def __init__(self):

		self.total_systhessis_time = 0
		self.total_completion_time = 0

		self._ac = actionlib.SimpleActionClient("/scots", AutoRacingAction)
		self._ac.wait_for_server()

		rospy.loginfo("Action Server is Up, starting to send goals.")

	# Function to send Goals to Action Servers
	def send_goal(self, targets):
		
		# Create Goal message for Simple Action Server
		goal = AutoRacingGoal(targets=targets)
		
		'''
			* done_cb is set to the function pointer of the function which should be called once 
				the Goal is processed by the Simple Action Server.

			* feedback_cb is set to the function pointer of the function which should be called while
				the goal is being processed by the Simple Action Server.
		''' 
		self._ac.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
		
		rospy.loginfo("Goal has been sent.")

	def get_time(self):
		return self.total_systhessis_time, self.total_completion_time

	def done_callback(self, status, result):
		self.total_systhessis_time += result.synthesis_time
		self.total_completion_time += result.completion_time
		
		rospy.loginfo("Target id: {}".format(result.target_id))
		rospy.loginfo("Synthesis Time: {}".format(result.synthesis_time))
		rospy.loginfo("Completion Time:  {}".format(result.completion_time))

	def feedback_callback(self, feedback):
		rospy.loginfo("Current Pose: ({}, {}, {})".format(round(feedback.curr_pose.x, 2), round(feedback.curr_pose.y, 2), round(feedback.curr_pose.theta, 2)))

class targetFinder:
	"""docstring for targetFinder"""
	def __init__(self, radius, action_client,robot_dimensions=[0.4, 0.3]):
		self.robot_max_length = robot_dimensions[0]		# Manimum length of the robot in meters
		self.robot_max_width = robot_dimensions[1]		# Miximum width of the robot in meters
		self.radius = radius				# size of the target window in pixel (to convert meters => target_window * resolution), note that it is a square and must always be an odd number
		self.pose_robot = Pose2D()
		self.action_client = action_client

		self.if_send_new_goal = False
		self.new_goal_service_name = "/new_goal"
		self.new_goal_service = rospy.Service(self.new_goal_service_name, Empty, self.new_goal_callback)


	def odom_callback(self, data):
		self.pose_robot.x = data.pose.pose.position.x
		self.pose_robot.y = data.pose.pose.position.y
		
	def select_targets(self,x,y):
		target = [x-self.radius,x+self.radius,y-self.radius,y+self.radius]
		return target
		
	def new_goal_callback(self, req):
		rospy.loginfo("Got new goal request.")
		self.if_send_new_goal = True
		return EmptyResponse()

	def send_new_goal(self, tr):
		if(self.if_send_new_goal):
			rospy.loginfo("Sending new goal to action server.")
			self.action_client.send_goal(tr)
			self.if_send_new_goal = False
		else:
			rospy.loginfo("send_new_goal flag is false, goal not sent.")


if __name__ == '__main__':
	rospy.init_node("target_generatopn")

	action_client = scotsActionClient()

	# TODO : Tuning Parameter
	radius = 1

	target_finder = targetFinder(radius=radius, action_client=action_client, robot_dimensions=[0.4, 0.3])

	rospy.Subscriber("odom", Odometry, target_finder.odom_callback)

	rate = rospy.Rate(1)
	print("Target Finder activated")
	try:
		while not rospy.is_shutdown():
			start = rospy.Time.now()
			targets = []
			tr = Target()
			#Here we have only for one target
			tr.id = 0
			tr.window = radius*2
			x = 1.9
			y = 8.3
			#x = 5.8
			#y = 1
			tr.points = target_finder.select_targets(x,y)
			print(tr.points)
			end = rospy.Time.now()
			targets.append(tr)
			#Code for 1 target ends
			
			print("Total Time. {}".format(end - start))

			if any(targets) and target_finder.if_send_new_goal:
					print("Goal targets, %r" % targets)
					target_finder.send_new_goal(targets)
					action_client._ac.wait_for_result()
			rate.sleep()
	except KeyboardInterrupt:
		pass