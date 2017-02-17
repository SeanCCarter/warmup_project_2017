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
	theta = theta * 3.14159/180
	return (r*math.cos(theta), r*math.sin(theta))

def make_marker(x, y):
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
	"""docstring for wallFollower"""
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
		# self.str_pub = rospy.Publisher("std_msgs", Bool, queue_size=10)
		# state = rospy.Subscriber("/messages", Bool)
		while self.front_point == None and self.back_point == None:
			self.r.sleep()
		print("starting.")

	def WFprocessLaser(self, L):
		print ("laser scan 2")
		"takes a bunch of data from the laser scanner and uses it"
		front_points = [p for p in L.ranges[310:320] if p != 0.0]
		back_points = [p for p in L.ranges[220:230] if p != 0.0]
		self.middle = L.ranges[270]
		self.front_point = sum(front_points)/len(front_points)
		self.back_point = sum(back_points)/len(back_points)

	def WFvizualize_wall(self):
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
		""" it does math to figure out the right distance away from
		wall"""
		rotational_corr = -.1*(self.front_point-self.back_point)
		distance_corr = -.2*(self.middle - distance)
		return rotational_corr + distance_corr





# class personFollower(object):
# 	"""docstring for wallFollower"""
	# def __init__(self):
		# rospy.init_node("personFollower")
		# rospy.Subscriber("/scan", LaserScan, self.processLaser)
		# self.r = rospy.Rate(10)
		# self.commander = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		# self.centroid = rospy.Publisher("/Centroid", Marker, queue_size=10)
		# self.st_pub = rospy.Publisher("std_msgs", Bool, queue_size=10)
		# state = rospy.Subscriber("/messages", Bool)
		while self.box == None:
			self.r.sleep()

	def PFprocessLaser(self, L):
		print ("processing laser 2")
		"takes a bunch of data from the laser scanner and uses it"
		self.laser_points = [polar_to_cartesian(r, theta) for theta, r in enumerate(L.ranges)]
		self.box = [p for p in self.laser_points if (p[0]>1 and p[0]<2) and (p[1]>-.5 and p[1]<.5)]
		return True

	def PFfindCenterMass(self):
		if self.box:
			centerx = sum([p[0] for p in self.box])/len(self.box)
			centery = sum([p[1] for p in self.box])/len(self.box)
			self.center = (centerx, centery)
			marker = make_marker(centerx, centery)
			self.centroid.publish(marker)
		else:
			self.center = (1.5, 0)
			marker = make_no_marker()
			self.centroid.publish(marker)
			
	def PFdoMath(self):
		theta = math.atan2(self.center[1],self.center[0])
		spin = Twist()
		spin.angular.z = .9*theta
		spin.linear.x = .3*(self.center[0] - 1.5) 
		return spin

	def PFrun(self):
		while not rospy.is_shutdown() and self.box:
			# if st = 1:
			self.PFfindCenterMass()
			self.publisher.publish(self.PFdoMath())
			self.r.sleep()
			# elif st = 0:
			# 	# processLaser()
			# 	rospy.spin()
		return self.WFrun

	def WFrun(self, distance=.75):
		forwards = Twist()
		forwards.linear.x = .1
		stop_command = Twist()
		while not rospy.is_shutdown() and not self.box:
			# if state == 0:
			forwards.angular.z = self.WFdoMath(distance) #distance
			self.WFvizualize_wall()
			self.publisher.publish(forwards)
			self.r.sleep()
			# elif state == 1:
				# rospy.spin()
		return self.PFrun

	def run(self):
		current_state = self.WFrun
		while not rospy.is_shutdown():
			current_state = current_state()


if __name__ == '__main__':
	node = FSM()
	node.run()
