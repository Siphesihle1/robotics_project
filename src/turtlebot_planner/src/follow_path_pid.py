#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
from math import cos, sin, atan2, sqrt
from turtlebot_planner.msg import Path

ROTATING = 0
FORWARD = 1
FINISHED = 2

RATE = 100
DISTANCE_THRES = 0.15

class FollowPath:
	def __init__(self, path):
		self.rate = rospy.Rate(RATE)

		self.path = path

		# PID control parameters
		self.kp_angular = 0.9 # 0.5
		self.kd_angular = 0.1

		self.linear_vel = 0.165 # 0.15

		# Errors
		self.error_angular = 0.0
		self.prev_error_angular = 0.0

		self.state = ROTATING # Initial state

		# Publish to velocity command and subscribe to obtain position
		self.forward = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
		self.model_coords = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		
	def follow_waypoint(self, waypoint, final=False):
		while not rospy.is_shutdown():
			drone_coords = self.model_coords('mobile_base', '')
			
			current_pos = drone_coords.pose.position
			current_orientation = drone_coords.pose.orientation
			
			_, _, current_yaw = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
			
			# Calculate errors
			error_x = waypoint[0] - current_pos.x
			error_y = waypoint[1] - current_pos.y
			distance = sqrt(error_x ** 2 + error_y ** 2)
			desired_yaw = atan2(error_y, error_x)

			cmd_vel = Twist()

			if self.state == ROTATING:
				# PID control for angular velocity
				self.error_angular = self.norm_angle(desired_yaw - current_yaw)
				angular_term = self.kp_angular * self.error_angular + \
					self.kd_angular * (self.error_angular - self.prev_error_angular)

				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = angular_term
				self.forward.publish(cmd_vel)

				if abs(self.error_angular) < 0.01: self.state = FORWARD
			elif self.state == FORWARD:
				if distance < DISTANCE_THRES:		
					cmd_vel.linear.x = 0.0
					cmd_vel.angular.z = 0.0
					self.forward.publish(cmd_vel)

					self.state = FINISHED if final else ROTATING
					return
				
				cmd_vel.linear.x =  self.linear_vel
				cmd_vel.angular.z = 0.0
				self.forward.publish(cmd_vel)
			
			# store current error
			self.prev_error_angular = self.error_angular

			self.rate.sleep()

	# Normalize an angle to the range [-pi, pi] (prevents robot from rotating all the way around to find desired orientation)
	def norm_angle(self, angle):
		return atan2(sin(angle), cos(angle))
	
	def follow_path(self):
		for i in range(1, len(self.path)): # skip start location
			print("Heading to waypoint: ({}, {})".format(str(self.path[i][0]), str(self.path[i][1])))

			self.follow_waypoint(self.path[i], i == len(self.path) - 1)
		
		if self.state == FINISHED: 
			print("Destination reached!")

def path_info_callback(msg):
	print("Currently at location: ({}, {})".format(str(msg.path[:2][0]), str(msg.path[:2][1])))
	print("Received path with destination: ({}, {})".format(str(msg.path[-2:][0]), str(msg.path[-2:][1])))
	rospy.sleep(1.0)

	# Get path and convert into list of (x, y) coordinate pairs
	path = [(msg.path[i], msg.path[i+1]) for i in range(0, msg.length - 1, 2)]

	# Follow path
	path_follower = FollowPath(path)
	path_follower.follow_path()

if __name__ == '__main__':
	# Initialize path subscriber
	rospy.init_node('navigator', anonymous=True)

	path_subscriber = rospy.Subscriber("/path_info", Path, path_info_callback)
	
	rospy.spin()