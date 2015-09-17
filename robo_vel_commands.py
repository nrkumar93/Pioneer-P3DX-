#!/usr/bin/env python

# imports
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# parameters
topic_cmd_vel = '/RosAria/cmd_vel'
update_interval = 0.1 # [s]
vel_lin = 0.5 # [m/s]
vel_ang = 0.0 # [rad/s]

# launch node and create a publisher
rospy.init_node('cmd_vel_test')
pub = rospy.Publisher(topic_cmd_vel, Twist)

# loop until shutdown
while not rospy.is_shutdown():

	# publish the defined linear and angular velocity
	pub.publish(Twist(Vector3(vel_lin, 0, 0), Vector3(0, 0, vel_ang)))
	
	# sleep the defined interval
	rospy.sleep(update_interval)
