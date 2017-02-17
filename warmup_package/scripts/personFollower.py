#!/usr/bin/env python

# File name: personFollower.py
# Author: Sean Carter, Paul Krusell
# Python Version: 2.7


from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math
import rospy

def polar_to_cartesian(r, theta):
	""" Converts polar coordinates to cartesian coordinates

		Args:
			r: radial coordinate
			theta: angular coordinate, in degrees

		Returns:
			A tuple (x, y), the cartesian coordinates
	"""
	theta = theta * 3.14159/180
	return (r*math.cos(theta), r*math.sin(theta))

def make_marker(x, y):
	""" Creates Marker for a sphere at a specified location
		
		Args:
			x: x coordinate of sphere, in meters
			y: y coordinate of sphere, in meters

		Returns:
			A Marker, located at (x, y) in the robot's coordinate
			frame, in the shape of a large sphere."""
	my_marker = Marker(type=Marker.SPHERE)
	my_marker.header.frame_id = "base_link"
	my_marker.pose.position.x = x
	my_marker.pose.position.y = y
	my_marker.color.a = 1
	my_marker.scale.x = .5
	my_marker.scale.y = .5
	my_marker.scale.z = .5
	return my_marker

def make_no_marker():
	""" Creates an invisible marker, since there is no other
		sensible way to stop displaying a marker

		Returns:
			A marker that can't be seen in rviz."""
	my_marker = Marker(type=Marker.SPHERE)
	my_marker.header.frame_id = "base_link"
	my_marker.pose.position.x = 0
	my_marker.pose.position.y = 0
	my_marker.color.a = 1
	my_marker.scale.x = 0
	my_marker.scale.y = 0
	my_marker.scale.z = 0
	return my_marker

class personFollower(object):
	""" Creates a ros node to command a neato to follow a person"""
	def __init__(self):
		rospy.init_node("personFollower")
		rospy.Subscriber("/scan", LaserScan, self.processLaser)
		self.r = rospy.Rate(10)
		self.commander = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.centroid = rospy.Publisher("/Centroid", Marker, queue_size=10)

		#Waits for rist measurement to strat following people
		while self.box == None:
			self.r.sleep()

	def processLaser(self, L):
		""" Takes in the laser range measurements from the neato,
			converts them to into cartesian coordinates in the robot's 
			'base link' frame, and then discards all the ones that aren't
			inside a 1x1 meter box, located 1 meter in front of the robot.

			Args:
				L: list of range measurements from the neato

			Sets:
				self.box: list of all points in a box in front of the robot"""

		#Convert all of the points into (x, y) tuples
		laser_points = [polar_to_cartesian(r, theta) for theta, r in enumerate(L.ranges)]

		#Store all points inside the area we want people to stand in in self.box
		self.box = [p for p in laser_points if (p[0]>1 and p[0]<2) and (p[1]>-.5 and p[1]<.5)]

	def findCenterMass(self):
		""" Finds where the person is standing, by averaging all of the 
			x and y coordinates
		"""
		if self.box: #Is there a person in front of the robot
			centerx = sum([p[0] for p in self.box])/len(self.box) #average x coordinates
			centery = sum([p[1] for p in self.box])/len(self.box) #average y coordinates
			self.center = (centerx, centery)
			marker = make_marker(centerx, centery)
			self.centroid.publish(marker) #visualize location of the person
		else:
			self.center = (1.5, 0)
			marker = make_no_marker()
			self.centroid.publish(marker) #stop vizualizing the person
			
	def doMath(self):
		""" Creates proportional controller to follow a person
			the further they are from an area in front of the
			robot, the faster it tries to center them

			Returns:
				Twist message: telling the neato how fast to turn,
				and how fast to move forwards.
		"""
		theta = math.atan2(self.center[1],self.center[0]) #angle between person and robot
		spin = Twist()
		spin.angular.z = .9*theta
		spin.linear.x = .3*(self.center[0] - 1.5) 
		return spin

	def run(self):
		while not rospy.is_shutdown():
			self.findCenterMass()
			self.commander.publish(self.doMath())
			self.r.sleep()

if __name__ == '__main__':
	node = personFollower()
	node.run()
