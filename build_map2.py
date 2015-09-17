#!/usr/bin/env python
import roslib
roslib.load_manifest('beginner_tutorials')
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
import numpy
import tf
from math import *

#plt.ion()
#ihl, = plt.plot([],[],'bo')
#plt.ion()
obj_x=[0.0 for i in range(8)]
obj_y=[0.0 for i in range(8)]
bot_x=0.0
bot_y=0.0
bot_z=0.0
bot_w=0.0
bot_euler=0.0
obj_x_proj=[0.0]*8
obj_y_proj=[0.0]*8
theta=[1.57079,0.872664,0.523598,0.1745329,-0.1745329,-0.523598,-0.872664,-1.57079]
r=[0.0]*8
sensor_range=1

def callback_sonar(data):
	global obj_x, obj_y
#       global hl
#       rospy.loginfo(rospy.get_name() + " % len(data.points))
#        print data.points[1].x
	for i in range(8):
		obj_x[i]=data.points[i].x
		obj_y[i]=data.points[i].y
#       hl.set_xdata(numpy.append(hl.get_xdata(),data.points[1].x))
#       hl.set_ydata(numpy.append(hl.get_ydata(),data.points[1].y))
#       plt.ion()
#        plt.plot(data.points[1].x,data.points[1].y,'bo')
#       plt.plot(1,2,'bo')
#       plt.plot(3,4,'bo')
#	plt.plot(4,5,'bo')
#       plt.plot(3.2,4.2,'bo')
#        plt.draw()
#       plt.plot(data.points[0].x,data.points[0].y,'bo')
#       plt.draw()

def callback_pose(data):
	global bot_x, bot_y, bot_euler, bot_z, bot_w
#	print data.pose.pose.position
	bot_x= data.pose.pose.position.x
	bot_y= data.pose.pose.position.y
	bot_z= data.pose.pose.orientation.z
	bot_w= data.pose.pose.orientation.w
	bot_euler= tf.transformations.euler_from_quaternion([0,0,bot_z,bot_w])[2]	

def sonar_pose_listener():
	while not rospy.is_shutdown():
        	global bot_x, bot_y, obj_x, obj_y, obj_x_proj, obj_y_proj, r
		rospy.init_node('sonar_listener', anonymous=True)
        	rospy.Subscriber("/RosAria/sonar", PointCloud, callback_sonar)
#	        rospy.sleep(0.5)
#		rospy.init_node('pose_listener',anonymous=True)
		rospy.Subscriber("/RosAria/pose",Odometry,callback_pose)
		print "hello"
#		rospy.sleep(0.5)
		for i in range(8):
			r[i]=sqrt(pow(obj_x[i],2)+pow(obj_y[i],2))
			obj_x_proj[i]=r[i]*cos(bot_euler+theta[i])
			obj_y_proj[i]=r[i]*sin(bot_euler+theta[i])
		for i in range(8):
			if r[i]<sensor_range:
				plt.plot(obj_x_proj[i]+bot_x,obj_y_proj[i]+bot_y,marker='s',markerfacecolor='none')
		plt.plot(bot_x,bot_y,'r.')
		plt.draw()
#		print bot_euler
#		print obj_x_proj+bot_x
#		print obj_y_proj+bot_y


if __name__ == '__main__':
        plt.ion()
#       plt.plot(0,0,'ro')
        sonar_pose_listener()

