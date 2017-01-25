#!/usr/bin/env python

from visualization_msgs.msg import Marker
import rospy

rospy.init_node("Marker")

my_marker = Marker(type=Marker.SPHERE)
my_marker.header.frame_id = "base_link"
my_marker.pose.position.x = 1
my_marker.pose.position.y = 2
my_marker.color.a = 1
my_marker.scale.x = .5
my_marker.scale.y = .5
my_marker.scale.z = .5
publisher = rospy.Publisher('/Marker', Marker, queue_size=10)

r = rospy.Rate(1)
while not rospy.is_shutdown():
	publisher.publish(my_marker)
	r.sleep()
print "node is finished"
