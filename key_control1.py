#!/usr/bin/env python
import roslib 
roslib.load_manifest('beginner_tutorials')
import rospy
from geometry_msgs.msg import Twist
import curses.wrapper
import curses

def talker(screen):
	pub=rospy.Publisher('/RosAria/cmd_vel',Twist)
	rospy.init_node('keyboard_vel_cmd')
	twist=Twist()
	while not rospy.is_shutdown():
		key=screen.getch()
		screen.refresh()
		if key == curses.KEY_UP:
			twist=Twist()
			twist.linear.x=0.2
		elif key == curses.KEY_DOWN:
			twist=Twist()
			twist.linear.x=-0.2
		elif key == curses.KEY_RIGHT:
			twist=Twist()
			twist.angular.z=-0.2
		elif key == curses.KEY_LEFT:
			twist=Twist()
			twist.angular.z=0.2
		else:
			twist=Twist()
		pub.publish(twist)
#		rospy.sleep(0.1)
		
if __name__=='__main__':
	try:
		curses.wrapper(talker)
	except rospy.ROSInterruptException:
		print "exception raised"
		pass

