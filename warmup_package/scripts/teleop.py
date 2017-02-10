#!/usr/bin/env python

import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist
import rospy

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#Getting the settings, setting basic variables
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
    

