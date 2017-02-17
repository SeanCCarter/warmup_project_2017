#!/usr/bin/env python

# File name: wallFollower.py
# Author: Sean Carter, Paul Krusell
# Python Version: 2.7

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import rospy



class wallFollower(object):
	""" Creates a ros node to command the neato to follow a wall
		Only works if the wall is on the right.
	"""

	def __init__(self):
		rospy.init_node("wallFollower")
		rospy.Subscriber("/scan", LaserScan, self.processLaser)
		self.r = rospy.Rate(10)
		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.visualizer = rospy.Publisher("Marker", Marker, queue_size=10)
		self.front_point = None

		# Waits for the first measurement to come in to start
		while self.front_point == None:
			self.r.sleep()
		print("starting.")

	def processLaser(self, L):
		""" callback function: Takes in laser scan data, and
			turns it into an average distance to the wall 
			from a selection of points across 360 degrees.

			Args:
				L: a list of distances from the laserscanner

			sets:
				middle: distance to a point directly right of the robot
				front_point: average of 10 points 40-50 degrees greater than middle
				back_point: average of 10 points 40-50 degrees less than middle
		"""

		#Average 10 points, excluding any with missing range data
		front_points = [p for p in L.ranges[310:320] if p != 0.0]
		back_points = [p for p in L.ranges[220:230] if p != 0.0]
		self.middle = L.ranges[270]
		self.front_point = sum(front_points)/len(front_points)
		self.back_point = sum(back_points)/len(back_points)

	def vizualize_wall(self):
		""" Constructs a line in the robot's coordinate system to
			represent the wall. The line connects the estimated 
			front and back points.

			This line is published to the /Marker rostopic
		"""

		#Points are converted from polar to cartesian here
		point1 = Point()
		#(-math.pi/4) represents the 45 degree rotation of the front point
		#from the front of the robot
		point1.x = math.cos((-math.pi/4))*self.front_point
		point1.y = math.sin((-math.pi/4))*self.front_point
		point2 = Point()
		#(-3*math.pi/4) represents the back point's 90 degree rotaion from
		#the front point
		point2.x = math.cos((-3*math.pi/4))*self.back_point
		point2.y = math.sin((-3*math.pi/4))*self.back_point
		my_marker = Marker(type=Marker.LINE_STRIP)
		my_marker.header.frame_id = "base_link"
		my_marker.color.a = 1
		my_marker.scale.x = .1
		my_marker.points = [point1, point2]
		self.visualizer.publish(my_marker)

	def doMath(self, distance):
		""" Helper function, that takes the robot's position and 
			rotation relative to the wall, and decides how fast
			the robot should be returning

			Args:
				distance: float or int, representing distance that the robot 
				should be trying to keep between itself and the wall

			Returns:
				The speed, in radians/s, the robot should be turning
		"""
		rotational_corr = -.1*(self.front_point-self.back_point)
		distance_corr = -.2*(self.middle - distance)
		return rotational_corr + distance_corr

	def run(self, distance=1):
		""" Main loop of the wall following algorithm

			Args:
				distance: float or int, representing distance that the robot 
				should be trying to keep between itself and the wall 
		"""
		forwards = Twist()
		forwards.linear.x = .1
		stop_command = Twist()
		while not rospy.is_shutdown():
			forwards.angular.z = self.doMath(distance)
			self.vizualize_wall()
			self.publisher.publish(forwards)
			self.r.sleep()

if __name__ == '__main__':
	node = wallFollower()
	node.run()
