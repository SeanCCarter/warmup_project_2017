#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math
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

class personFollower(object):
	"""docstring for wallFollower"""
	def __init__(self):
		# rospy.init_node("personFollower")
		rospy.Subscriber("/scan", LaserScan, self.processLaser)
		self.r = rospy.Rate(10)
		self.commander = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.centroid = rospy.Publisher("/Centroid", Marker, queue_size=10)
		# self.st_pub = rospy.Publisher("std_msgs", Bool, queue_size=10)
		# state = rospy.Subscriber("/messages", Bool)
		while self.box == None:
			self.r.sleep()

	def processLaser(self, L):
		"takes a bunch of data from the laser scanner and uses it"
		self.laser_points = [polar_to_cartesian(r, theta) for theta, r in enumerate(L.ranges)]
		self.box = [p for p in self.laser_points if (p[0]>1 and p[0]<2) and (p[1]>-.5 and p[1]<.5)]
		st = Bool()
		# if self.box:
		# 	st = 1
		# 	self.st_pub.publish(st)
		# else:
		# 	st = 0
		# 	self.st_pub.publish(st)
		return True

	def findCenterMass(self):
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
			
	def doMath(self):
		theta = math.atan2(self.center[1],self.center[0])
		spin = Twist()
		spin.angular.z = .9*theta
		spin.linear.x = .3*(self.center[0] - 1.5) 
		return spin

	def run(self):
		while not rospy.is_shutdown():
			# if st = 1:
			self.findCenterMass()
			self.commander.publish(self.doMath())
			self.r.sleep()
			# elif st = 0:
			# 	# processLaser()
			# 	rospy.spin()

if __name__ == '__main__':
	node = personFollower()
	node.run()
