#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Twist
import rospy

class wallFollower(object):
	"""docstring for wallFollower"""
	def __init__(self):
		rospy.init_node("wallFollower")
		rospy.Subscriber("/scan", LaserScan, self.processLaser)
		self.r = rospy.Rate(10)
		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.front_point = None
		while self.front_point == None:
			self.r.sleep()


	def processLaser(self, L):
		"takes a bunch of data from the laser scanner and uses it"
		self.front_point = L.ranges[315]
		self.back_point = L.ranges[225]
		# self.front_point = [p for p in L.ranges[310:320] if p != 0.0]
		# self.back_point = [p for p in L.ranges[265:275] if p != 0.0]


	def run(self, distance=1):
		forwards = Twist()
		forwards.linear.x = .1
		stop_command = Twist()
		while not rospy.is_shutdown():
			forwards.angular.z = -.9*(self.front_point-self.back_point)
			self.publisher.publish(forwards)
			self.r.sleep()

if __name__ == '__main__':
	node = wallFollower()
	node.run()
