#!/usr/bin/env python

# File name: teleop.py
# Author: Sean Carter, Paul Krusell
# Python Version: 2.7

import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist
import rospy

def getKey():
	''' Waits untill the user presses a key in the terminal
		and then returns the name of the key

		Returns: key, as a string
	'''
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#Getting the terminal settings, setting basic variables
settings = termios.tcgetattr(sys.stdin)
key = None
forwards = 0
turning = 0

#Setting up the publisher
rospy.init_node("teleop")
publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
message = Twist()

#loops every time a key is pressed
while key != '\x03' and not rospy.is_shutdown():
	key = getKey()
	#.3 is about as fast as the robot can turn or move
	#.1 increments gives several speed options under that cap
	if key == 'w' and forwards < .3:
		forwards += .1
	elif key == 's' and forwards > -.3:
		forwards -= .1
	elif key == 'd' and turning > -.3:
		turning -= .1
	elif key == 'a' and turning < .3:
		turning += .1
	else:
		pass
	message.linear.x = forwards
	message.angular.z = turning

	publisher.publish(message)
    

