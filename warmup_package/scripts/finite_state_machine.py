#!/usr/bin/env python

# File name: finite_state_machine.py
# Author: Sean Carter, Paul Krusell
# Python Version: 2.7

from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
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

class FSM(object):
	""" A finite state machine controller, that switches between
		following a wall and following a person."""
	def __init__(self):
		rospy.init_node("FSM")
		self.WFSubscriber = rospy.Subscriber("/scan", LaserScan, self.WFprocessLaser)
		self.PFSubscriber = rospy.Subscriber("/scan", LaserScan, self.PFprocessLaser)
		self.r = rospy.Rate(10)
		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.visualizer = rospy.Publisher("Marker", Marker, queue_size=10)
		self.centroid = rospy.Publisher("/Centroid", Marker, queue_size=10)
		self.back_point = None
		self.front_point = None
		self.state = 1
		while self.front_point == None and self.back_point == None:
			self.r.sleep()
		print("starting.")

	def WFprocessLaser(self, L):
		""" callback function: Takes in laser scan data, and
			turns it into an average distance to the wall 
			from a selection of points across 360 degrees.
			Used for wall following.

			Args:
				L: a list of distances from the laserscanner

			sets:
				middle: distance to a point directly right of the robot
				front_point: average of 10 points 40-50 degrees greater than middle
				back_point: average of 10 points 40-50 degrees less than middle
		"""
		front_points = [p for p in L.ranges[310:320] if p != 0.0]
		back_points = [p for p in L.ranges[220:230] if p != 0.0]
		self.middle = L.ranges[270]
		self.front_point = sum(front_points)/len(front_points)
		self.back_point = sum(back_points)/len(back_points)

	def WFvizualize_wall(self):
		""" Constructs a line in the robot's coordinate system to
			represent the wall. The line connects the estimated 
			front and back points.

			This line is published to the /Marker rostopic
		"""
		point1 = Point()
		point1.x = math.cos((-math.pi/4))*self.front_point
		point1.y = math.sin((-math.pi/4))*self.front_point
		point2 = Point()
		point2.x = math.cos((-3*math.pi/4))*self.back_point
		point2.y = math.sin((-3*math.pi/4))*self.back_point
		my_marker = Marker(type=Marker.LINE_STRIP)
		my_marker.header.frame_id = "base_link"
		my_marker.color.a = 1
		my_marker.scale.x = .1
		my_marker.points = [point1, point2]
		self.visualizer.publish(my_marker)

	def WFdoMath(self, distance):
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

	def PFprocessLaser(self, L):
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
		return True

	def PFfindCenterMass(self):
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
			
	def PFdoMath(self):
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

	def PFrun(self):
		""" Runs the main loop for person following, while there is
			a person.

			Returns:
				Function name of next state to run (WFrun)
		"""
		while not rospy.is_shutdown() and self.box:
			self.PFfindCenterMass()
			self.publisher.publish(self.PFdoMath())
			self.r.sleep()
		return self.WFrun

	def WFrun(self, distance=.75):
		""" Main loop of the wall following algorithm
			Stops whenever someone enters the box defined in PFprocessLaser

			Args:
				distance: float or int, representing distance that the robot 
				should be trying to keep between itself and the wall 

			Returns:
				Function name of next state to run (PFrun)
		"""
		forwards = Twist()
		forwards.linear.x = .1
		stop_command = Twist()
		while not rospy.is_shutdown() and not self.box:
			forwards.angular.z = self.WFdoMath(distance) #distance
			self.WFvizualize_wall()
			self.publisher.publish(forwards)
			self.r.sleep()
		return self.PFrun

	def run(self):
		current_state = self.WFrun
		while not rospy.is_shutdown():
			current_state = current_state()


if __name__ == '__main__':
	node = FSM()
	node.run()
