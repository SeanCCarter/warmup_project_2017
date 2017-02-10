#!/usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import math
import rospy

def calc_distance(x1, y1, x2, y2):
	return ((x2-x1)**2 + (y2-y1)**2)**(.5)

class odomNeatoCommander():
	def __init__(self):
		rospy.init_node("Marker")
		rospy.Subscriber("/odom", Odometry, self.process_position)
		self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		self.r = rospy.Rate(50)
		self.position = None
		self.theta = None
		while self.position == None:
			self.r.sleep()

	def process_position(self, m):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		pose = m.pose.pose
		orientation_tuple = (pose.orientation.x,
							 pose.orientation.y,
							 pose.orientation.z,
							 pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		self.position = (pose.position.x, pose.position.y)
		self.theta = angles[2] + 3.14159
	
	def go_forward(self, d, speed = .1):
		''' d: distance to travel, in meters
			speed: speed, in m/s
			returns: nothing
		'''
		start_point = self.position
		distance = 0
		go_command = Twist()
		go_command.linear.x = speed
		stop_command = Twist()
		stop_command.linear.x = 0
		while distance < d:
			distance = calc_distance(start_point[0], start_point[1], self.position[0], self.position[1])
			self.publisher.publish(go_command)
			self.r.sleep()
		self.publisher.publish(stop_command)

	def rotate(self, degrees, speed = 10):
		''' degrees: number of degrees to rotate the robot left
			speed: how fast to rotate the robot (degrees/s)
			returns: nothing
			TODO: distance turned has error of about .15 radians each turn
		'''
		start_point = self.theta
		end_point = ((degrees * 3.14159/180) + start_point)%(2*3.14159)
		spin_command = Twist()
		spin_command.angular.z = speed * 3.14/180
		stop_command = Twist()
		stop_command.angular.z = 0
		if end_point > start_point:
			while self.theta < end_point:
				self.publisher.publish(spin_command)
				self.r.sleep()
		else:
			while self.theta > (start_point - .02) or self.theta < end_point:
				self.publisher.publish(spin_command)
				self.r.sleep()
		self.publisher.publish(stop_command)

	def squrr(self):
		"makes a square"
		self.go_forward(.5)
		self.rotate(90)
		self.go_forward(.5)
		self.rotate(90)
		self.go_forward(.5)
		self.rotate(90)
		self.go_forward(.5)
		self.rotate(90)


if __name__ == '__main__':
	node = odomNeatoCommander()
	node.squrr()

