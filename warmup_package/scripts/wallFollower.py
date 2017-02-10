#!/usr/bin/env python

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import rospy



class wallFollower(object):
	"""docstring for wallFollower"""
	def __init__(self):
		# rospy.init_node("wallFollower")
		rospy.Subscriber("/scan", LaserScan, self.processLaser)
		self.r = rospy.Rate(10)
		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.visualizer = rospy.Publisher("Marker", Marker, queue_size=10)
		self.front_point = None
		# self.str_pub = rospy.Publisher("std_msgs", Bool, queue_size=10)
		# state = rospy.Subscriber("/messages", Bool)
		while self.front_point == None:
			self.r.sleep()
		print("starting.")

	def processLaser(self, L):
		"takes a bunch of data from the laser scanner and uses it"
		# self.front_point = L.ranges[315]
		# self.back_point = L.ranges[225]
		front_points = [p for p in L.ranges[310:320] if p != 0.0]
		back_points = [p for p in L.ranges[220:230] if p != 0.0]
		self.middle = L.ranges[270]
		self.front_point = sum(front_points)/len(front_points)
		self.back_point = sum(back_points)/len(back_points)

	def vizualize_wall(self):
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

	def doMath(self, distance):
		""" it does math to figure out the right distance away from
		wall"""
		rotational_corr = -.1*(self.front_point-self.back_point)
		distance_corr = -.2*(self.middle - distance)
		return rotational_corr + distance_corr

	def run(self, distance=1):
		forwards = Twist()
		forwards.linear.x = .1
		stop_command = Twist()
		while not rospy.is_shutdown():
			# if state == 0:
			forwards.angular.z = self.doMath(distance)
			self.vizualize_wall()
			self.publisher.publish(forwards)
			self.r.sleep()
			# elif state == 1:
				# rospy.spin()

if __name__ == '__main__':
	node = wallFollower()
	node.run()
