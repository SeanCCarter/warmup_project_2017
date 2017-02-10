#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import math
import rospy

def polar_to_cartesian(r, theta):
	theta = theta * 3.14159/180
	return (r*math.cos(theta), r*math.sin(theta))

class obstacleAvoider(object):
	def __init__(self):
		rospy.init_node("obstacleAvoider")
		rospy.Subscriber("/scan", LaserScan, self.processLaser)
		self.r = rospy.Rate(10)
		self.commander = rospy.Publisher("cmd_vel",Twist, queue_size=10)
		self.visualizer = rospy.Publisher("/Marker", Marker, queue_size=10)
		self.front_points = None
		while self.front_points == None:
			self.r.sleep()


	def processLaser(self, L):
		"takes a bunch of data from the laser scanner and uses it"
		# self.front_point = L.ranges[315]
		# self.back_point = L.ranges[225]
		self.front_points = L.ranges[-46:-1] + L.ranges[0:45]
		self.back_points = L.ranges[135:225]

	def doMath(self, distance_weight = 1):
		""" returns a vector pointing to center of mass of obstacles,
			weighted by the distance from the robot to the obstacle"""
		points = []
		for i, r in enumerate(self.front_points):
			if r != 0.0:
				theta = (i-45)*math.pi/180
				points.append((r*math.cos(theta), r*math.sin(theta), r))

		vectorx = 0
		vectory = 0
		for i, point in enumerate(points):
			if point[0] and point[1]: #Don't want to divide by 0
				vectorx += point[0]/(distance_weight*point[2]**3)
				vectory += point[1]/(distance_weight*point[2]**3)

		#Adding up al.l points ruins proportional control
		#A wall might have a huge weght, no matter the distance
		if vectorx:
			vectorx = vectorx/len(points)
		if vectory:
			vectory = vectory/len(points)
		self.obstacle_vector = (vectorx, vectory)

	def visualize_obstacle(self):
		my_marker = Marker(type=Marker.ARROW)
		my_marker.header.frame_id = "base_link"
		vector_point = Point()
		vector_point.x = self.obstacle_vector[0]
		vector_point.y = self.obstacle_vector[1]
		my_marker.points = [Point(), vector_point]
		my_marker.scale.x = .1
		my_marker.scale.y = .2
		my_marker.color.a = 1
		self.visualizer.publish(my_marker)

	def command_robot(self, base_speed = .3, obstacle_weight = .1, turn_weight = .5):
		command = Twist()
		obstacle = (math.hypot(self.obstacle_vector[0], self.obstacle_vector[1]), #magnitude of vector
			math.atan2(self.obstacle_vector[1],self.obstacle_vector[0])) #theta of vector
		# command.linear.x = base_speed - obstacle_weight*obstacle[0] 
		command.linear.x = base_speed - obstacle_weight*obstacle[0]*self.obstacle_vector[0]
		# command.angular.z = turn_weight*(3.14/2 - obstacle[1])*obstacle[0]
		command.angular.z = turn_weight*self.obstacle_vector[1]*-obstacle[0]

		self.commander.publish(command)

	def run(self):
		while not rospy.is_shutdown():
			self.doMath(2)
			self.visualize_obstacle()
			self.command_robot(.3, .2, 3)
			self.r.sleep()

if __name__ == '__main__':
	node = obstacleAvoider()
	node.run()