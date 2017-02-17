#!/usr/bin/env python

# File name: obstacleAvoider.py
# Author: Sean Carter, Paul Krusell
# Python Version: 2.7

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
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

class obstacleAvoider(object):
	""" Creates a ros node to make a neato drive forwards without hitting anthing
		Technically, it avoids a single platonic obstacle"""
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
		""" Callback function that takes the neato laser range data,
			and discards any points that aren't in the 90 degree arc
			in front of the robot
			
			Args:
				L: List of laser scan ranges from the neato

			Sets:
				self.front_points: list of all points in front of the
				robot"""

		#The first and last points in L are the same, so we skip it once
		self.front_points = L.ranges[-46:-1] + L.ranges[0:45]

	def doMath(self, distance_weight = 1):
		""" creates a vector pointing to 'center of mass' of obstacles,
			weighted by the distance from the robot to the obstacle
			The larger the vector, the more obstacles there are

			Args:
				distance_weight: float or int, made to help calibrate the avoidance
				the larger it is, the less the robot uses distant obstacles to help
				navigate

			Sets:
				self.obstacle_vector: abstract vector thing representing importance
				of obstacles, and whether they are in front of the robot"""
		points = []
		for i, r in enumerate(self.front_points):
			if r != 0.0: #Ignore any point for which there is no data
				theta = (i-45)*math.pi/180 #find angle between point and the center of the robot
				points.append((r*math.cos(theta), r*math.sin(theta), r)) #Location of point in 'base link'

		vectorx = 0
		vectory = 0
		for i, point in enumerate(points):
			if point[0] and point[1]: #Don't want to divide by 0
				#The closer a point is, the larger its effect on the obstacle vector
				#The cube of the distance is used, because it is MUCH more important
				#to avoid one very close point, than many far away. 
				vectorx += point[0]/(distance_weight*point[2]**3)
				vectory += point[1]/(distance_weight*point[2]**3)

		#This normalizes the vector we created
		#Otherwise, we couldn't fine tune the proportional control
		#The more points the robot saw, the faster it would turn
		vectorx = vectorx/len(points)
		vectory = vectory/len(points)
		self.obstacle_vector = (vectorx, vectory)

	def visualize_obstacle(self):
		""" Constructs an arrow Marker. The closer obstacles are,
			the larger the arrow. The abstract obstacle that the
			robot is avoiding """
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
		""" Publishes a command to the robot, to tell it how fast to turn,
			and how fast to rotate

			Args:
				base_speed: in m/s, how fast the robot should move forwards
				obstacle_weight: float or int - how important to consider obstacles in front of the robot
				turn_weight: how fast should the robot rotatewhen obstacles are from the side
		"""
		command = Twist()

		#converts self.obstacle_vector into polar coordinates
		obstacle = (math.hypot(self.obstacle_vector[0], self.obstacle_vector[1]), #magnitude of vector
			math.atan2(self.obstacle_vector[1],self.obstacle_vector[0])) #theta of vector

		#the base speed is how fast to go normaly, and the robot slows down when it has an
		#obstacle, proportionally to the distance to the obstacle * its magnitude in the x
		#direction
		command.linear.x = base_speed - obstacle_weight*obstacle[0]*self.obstacle_vector[0]

		#The turn rate of the robot is based how close the obstacle is to center,
		#and proportional to how close the obstacle is to the x axis
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