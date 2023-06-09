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
from ackermann_msgs.msg import AckermannDriveStamped

# other
import numpy as np
from typing import final

class targetFinder:
	"""docstring for targetFinder"""
	def __init__(self, target_window=1, robot_dimensions=[0.4, 0.3]):

		assert isinstance(target_window, (int)), "Target window must be integer, Recieved type %r" % type(target_window).__name__
		# assert round(robot_dimensions[0] * robot_dimensions[1], 2) <= round((resolution * target_window) ** 2, 2), "Robot size(area) must not be bigger than target window(area), given robot size is %r m*m, while target window is %r m*m" %(robot_dimensions[0] * robot_dimensions[1], (resolution * target_window) ** 2)
		# assert target_window >= 4, "Target window must be greater than or equal to 9 (0.4*0.4 m*m), Recieved %r" % target_window
		# assert target_window % 2 != 0, "Target window must be an odd number, Recieved %r" % target_window

		self.radius = target_window				# size of the target window in pixel (to convert meters => target_window * resolution), note that it is a square and must always be an odd number
		self.robot_pose = Pose2D()
		self.targets = []

	def odom_callback(self, data):
		self.robot_pose.x = data.x
		self.robot_pose.y = data.y
		self.robot_pose.theta = data.theta
	
	
	def drive_callback(self, data):
		if data.drive.steering_angle > 0.2 :
			points = []
			points.append(self.robot_pose.x - self.radius)
			points.append(self.robot_pose.y - self.radius)
			points.append(self.robot_pose.x + self.radius)
			points.append(self.robot_pose.y + self.radius)
			self.targets.append(points)
		return

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