#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import math
import rospy

from wallFollower import wallFollower
from personFollower import personFollower

def polar_to_cartesian(r, theta):
		theta = theta * 3.14159/180
		return (r*math.cos(theta), r*math.sin(theta))
		
class stateMachine(object):
	""" makes a finite state machine for wall following 
		and person followin"""
	def __init__(self):
		rospy.init_node("stateMachine")

		rospy.Subscriber("/scan", LaserScan, self.processLaser)
		self.r = rospy.Rate(10)
		self.wallFollower = wallFollower()
		self.personFollower = personFollower()
		self.state = 1

	

	def processLaser(self, L):
		"takes a bunch of data from the laser scanner and uses it"
		self.laser_points = [polar_to_cartesian(r, theta) for theta, r in enumerate(L.ranges)]
		self.box = [p for p in self.laser_points if (p[0]>1 and p[0]<2) and (p[1]>-.5 and p[1]<.5)]
		if self.box:
			self.state = 2
		else:
			self.state = 1

	def state1(self):
		""" the wall follow state"""
		print ("state 1")
		self.wallFollower.run()
		if self.state == 2:
			return state2

	def state2(self):
		""" follows a person"""
		print ("state 2")
		self.personFollower.run()
		if self.state == 1:
			return state1

if __name__ == '__main__':
	node = stateMachine()
	node.state1()