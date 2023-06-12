#!/usr/bin/env python3
"""
	File Name: target_estimation.py
	Description: Divide the map into 4 portions and find the targets automatically based on the 
				 position on which quadrant and orientation.
	Name: Allen Emmanuel Binny
"""
# ros includes
import rospy
import actionlib
from autoracing_msgs.msg import AutoRacingAction
from autoracing_msgs.msg import AutoRacingGoal
from autoracing_msgs.msg import Target
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose2D

# other
import numpy as np
from typing import final

class targetFinder:
	"""docstring for targetFinder"""
	def __init__(self, resolution=0.05, target_window=4, robot_dimensions=[0.4, 0.3]):

		# assert isinstance(target_window, (int)), "Target window must be integer, Recieved type %r" % type(target_window).__name__

		assert round(robot_dimensions[0] * robot_dimensions[1], 2) <= round((resolution * target_window) ** 2, 2), "Robot size(area) must not be bigger than target window(area), given robot size is %r m*m, while target window is %r m*m" %(robot_dimensions[0] * robot_dimensions[1], (resolution * target_window) ** 2)
		# assert target_window >= 4, "Target window must be greater than or equal to 9 (0.4*0.4 m*m), Recieved %r" % target_window
		# assert target_window % 2 != 0, "Target window must be an odd number, Recieved %r" % target_window

		self.resolution = resolution					# Size of each grid in meters
		self.robot_max_length = robot_dimensions[0]		# Manimum length of the robot in meters
		self.robot_max_width = robot_dimensions[1]		# Miximum width of the robot in meters
		self.target_window = target_window				# size of the target window in pixel (to convert meters => target_window * resolution), note that it is a square and must always be an odd number
		self.odom_msg = Pose2D

	def odom_callback(self, data):
		self.odom_msg.x = data.x
		self.odom_msg.y = data.y
		self.odom_msg.theta = data.theta
	
	def find_quadrant(self, maps,width,height):
		#For Quadrant 1
		if self.odom_msg.x < (width) and self.odom_msg.x < (height/2):
			pass

		#For Quadrant 2
		elif self.odom_msg.x < (width/2) and self.odom_msg.x < (height/2):
			pass
		
		#For Quadrant 3
		elif self.odom_msg.x < (width/2) and self.odom_msg.x < (height/2):
			pass

		#For Quadrant 4
		else:
			pass
		
	def get_targets(self, maps, width, height):
		"""
		This function finds out all potential targets in the map
		If a point is near to a frontier while also being far away enough from a wall, then a region around the point that is far enough from the frontier itself 
		is chosen at the potential target

		Parameters
			The function only needs a map

		Returns
			It returns the start and end points of all the regions that can be chosen as targets
		"""
		maps = maps[0]
		potential_targets = []	 

		for i in range(width):
			for j in range(height):
				# Made a change for the occupancy grid
				if maps[i][j] == 0:
					[neighbor_map, x_s, y_s, x_e, y_e] = self.neighbors(self.target_window, i, j, maps, width, height)
					if not any(1 in n for n in neighbor_map):  # Check if there are any obstacles nearby
						potential_targets.append([[x_s, y_s], [x_e, y_e]])

		return potential_targets
	
	def neighbors(self, radius, row_number, column_number, submaps, width, height):
		"""
		This function finds (radius/2) number of neighbors of given point on all four sides
		
		Parameters
			radius: how many neighbors do you want to obtain either horizontally or vertically
			row_number: x coordinate of the point
			column_number: y coordinate of the point
			submaps: The part of the map within which we find the neighbors

		Returns
			The function returns the neighbors and the x and y coordinates of the starting and ending element of the neighbors array with respect to the submap coordinate 
		"""
		radius = int(radius // 2)
		row_number = int(row_number)
		column_number = int(column_number)

		# If the selected has neighbors all around it within the map
		if row_number < width - radius and column_number < height - radius and row_number > radius and column_number > radius: 
			return submaps[row_number - radius : row_number + radius, column_number - radius : column_number + radius], row_number - radius, column_number - radius, row_number + radius, column_number + radius
		else:
			return [[]], row_number - radius, column_number - radius, row_number + radius, column_number + radius

	def select_targets(self,all_targets):
		targets = []
		# THIS IS FOR COLUMBIA.YAML
		# targets.append([[11.5, 5.5], [12.5, 6.5]])
		# targets.append([[15, 13], [16, 14]])
		# targets.append([[10, 10.5], [11, 11.5]])
		# targets.append([[4, 3.5], [5, 4.5]])
		# targets.append([[7, 2], [8, 3]])

		# This is for BERLIN.YAML

		# targets.append([[16.5, 23.5], [18.5, 25.5]])
		# targets.append([[21, 18.5], [23, 20.5]])
		# targets.append([[16.5, 11], [18.5, 13]])
		# targets.append([[16.5, 7], [18.5, 9]])
		# targets.append([[17.5, 2.5], [19.5, 4.5]])
		# targets.append([[15.5, 1], [17.5, 3]])
		# targets.append([[13.5, 2.5], [15.5, 4.5]])
		# targets.append([[12.5, 7], [14.5, 9]])
		# targets.append([[12.5, 11.5], [14.5, 13.5]])
		# targets.append([[10.5, 13], [12.5, 15]])
		# targets.append([[8.5, 15.5], [10.5, 17.5]])
		# targets.append([[9.5, 18.5], [11.5, 20.5]])
		# targets.append([[10.5, 20.5], [12.5, 22.5]])
		# targets.append([[7, 20], [9, 22]])
		# targets.append([[5.5, 20.5], [7.5, 22.5]])
		# targets.append([[5, 21.5], [7, 23.5]])
		# targets.append([[7.5, 24.5], [9.5, 26.5]])
		# targets.append([[12, 25.5], [14, 27.5]])

		# For Berlin.yaml portion
		# targets.append([[7.5,11.5],[9.5,13.5]])

		# For torrino.yaml portion
		targets.append([[3.5,6.5],[6.5,9.5]])

		# For mtl.yaml portion
		# targets.append([[6,6],[8,8]])

		# targets.append([[19, 19], [24, 20]])
		# # targets.append([[16, 6], [19, 7]])
		# targets.append([[16, 1.5], [17, 2.5]])
		# targets.append([[13, 6], [15, 7]])
		# targets.append([[9.5, 13], [13, 14]])
		# targets.append([[10, 21], [13, 22]])
		# targets.append([[13, 25], [14, 28]])

		# targets.append([[11, 10.5], [12, 11.5]]) #For mtl map
		# targets.append([[5.5, 2], [6.5, 3]])
		return targets


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

class mapData:
	"""docstring for mapData"""
	def __init__(self, action_client):
		self.width = 0
		self.height = 0
		self.resolution = 0
		self.maps = list()

		self.action_client = action_client

		self.map_topic_name = "/map"
		self.map_sub_handle = rospy.Subscriber(self.map_topic_name, OccupancyGrid, self.map_data_callback)

		rospy.loginfo("Waiting for data on /map topic..")
		rospy.wait_for_message(self.map_topic_name, OccupancyGrid, timeout=10)

		self.if_send_new_goal = False
		self.new_goal_service_name = "/new_goal"
		self.new_goal_service = rospy.Service(self.new_goal_service_name, Empty, self.new_goal_callback)

	def map_data_callback(self, msg):
		self.width = msg.info.width
		self.height = msg.info.height
		self.resolution = msg.info.resolution

		map_image = np.zeros((self.height, self.width, 1), dtype="int8")
		for i in range(0, self.height):
			for j in range(0, self.width):
				if msg.data[i * self.width + j] > 0 :
					map_image[i][j] = 1
				else:
					map_image[i][j] = int(msg.data[i * self.width + j])

		self.maps = list(map_image.T)

	def new_goal_callback(self, req):
		rospy.loginfo("Got new goal request.")
		self.if_send_new_goal = True
		return EmptyResponse()

	def send_new_goal(self, targets):
		if(self.if_send_new_goal):
			rospy.loginfo("Sending new goal to action server.")
			self.action_client.send_goal(targets)
			self.if_send_new_goal = False
		else:
			rospy.loginfo("send_new_goal flag is false, goal not sent.")

	def get_map(self):
		return self.maps

	def get_map_dimensions(self):
		return self.width, self.height, self.resolution

if __name__ == '__main__':
	rospy.init_node("target_estimation")

	action_client = scotsActionClient()
	mapdata = mapData(action_client)

	# TODO : Tuning Parameter
	target_window = 20

	_w, _h, resolution = mapdata.get_map_dimensions()
	
	target_finder = targetFinder(resolution=resolution, target_window=target_window, robot_dimensions=[0.4, 0.3])

	rospy.Subscriber("robot_pose", Pose2D, target_finder.odom_callback)

	rate = rospy.Rate(1)
	print("Target Finder activated")

	try:
		while not rospy.is_shutdown():
			maps = mapdata.get_map()
			width, height, resolution = mapdata.get_map_dimensions()

			targets = []
			start = rospy.Time.now()
			safe_targets = target_finder.select_targets(targets)
			end = rospy.Time.now()

			print("Total Time. {}".format(end - start))

			if(len(safe_targets) > 0):
				for i in range(len(safe_targets)):
					if not any(safe_targets[i]):
						continue
					
					tr = Target()
					tr.id = i
					tr.window = round((target_window - 1) * resolution, 2)


					for j in range(len(safe_targets[i])):
						tr.points.append(safe_targets[i][j][0])
					for j in range(len(safe_targets[i])):
						tr.points.append(safe_targets[i][j][1])
					
					targets.append(tr)
				
				if any(targets) and mapdata.if_send_new_goal:
					print("Goal targets, %r" % targets)
					mapdata.send_new_goal(targets)
					
					action_client._ac.wait_for_result()
			else:
				print("No targets found.. increasing target window.")
				target_window += 2

			rate.sleep()
	except KeyboardInterrupt:
		pass