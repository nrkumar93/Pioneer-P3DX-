#!/usr/bin/env python
import roslib
roslib.load_manifest('beginner_tutorials')
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
import tf
from pylab import *
import message_filters
import cv2

world=zeros((201,201))
count=1

def callback(sonar_data,pose_data):
	l0=0
	update=log(0.7/0.3)
	global world, count
	obj_x=[0.0]*8
	obj_y=[0.0]*8
	r=[0.0]*8
	obj_x_proj=[0.0]*8
	obj_y_proj=[0.0]*8
	obj_final_x=[0.0]*8
	obj_final_y=[0.0]*8
	theta=[1.57079,0.872664,0.523598,0.1745329,-0.1745329,-0.523598,-0.872664,-1.57079]
	sensor_range=4
	for i in range(8):
		obj_x[i]=sonar_data.points[i].x
		obj_y[i]=sonar_data.points[i].y
	bot_x=pose_data.pose.pose.position.x
	bot_y=pose_data.pose.pose.position.y
	bot_z=pose_data.pose.pose.orientation.z
	bot_w=pose_data.pose.pose.orientation.w
	bot_euler=tf.transformations.euler_from_quaternion([0,0,bot_z,bot_w])[2]
	for i in range(8):
		r[i]=sqrt(pow(obj_x[i],2)+pow(obj_y[i],2))
		obj_x_proj[i]=r[i]*cos(bot_euler+theta[i])
		obj_y_proj[i]=r[i]*sin(bot_euler+theta[i])
		obj_final_x[i]=obj_x_proj[i]+bot_x
		obj_final_y[i]=obj_y_proj[i]+bot_y
		if r[i]<sensor_range:
			m=(obj_final_y[i]-bot_y)/(obj_final_x[i]-bot_x)
			c=bot_y-m*bot_x
			if bot_x<obj_final_x[i]:
				x1=bot_x
				while x1<obj_final_x[i]:
					y1=m*x1+c
					x1_mat=int(x1*10+100)
					y1_mat=int(y1*10+100)
					world[x1_mat,y1_mat]=world[x1_mat,y1_mat]-update-l0
					x1=x1+0.1
			else:
				x1=obj_final_x[i]+0.1
				while x1<bot_x:
					y1=m*x1+c
					x1_mat=int(x1*10+100)
					y1_mat=int(y1*10+100)
					world[x1_mat,y1_mat]=world[x1_mat,y1_mat]-update-l0
					x1=x1+0.1
			world[int(obj_final_x[i]*10+100),int(obj_final_y[i]*10+100)]=world[int(obj_final_x[i]*10+100),int(obj_final_y[i]*10+100)]+update-l0
	cv2.imshow('w1',world)
	cv2.waitKey(50)
	print count
	count=count+1
		

def sonar_pose_listener():
	rospy.init_node('sonar_pose_listener', anonymous=True)
	sonar_list=message_filters.Subscriber('/RosAria/sonar', PointCloud)
	pose_list=message_filters.Subscriber('/RosAria/pose', Odometry)
	ts=message_filters.TimeSynchronizer([sonar_list, pose_list],10)
	ts.registerCallback(callback)
	rospy.spin()

if __name__=='__main__':
	cv2.namedWindow('w1',cv2.CV_WINDOW_AUTOSIZE)
	sonar_pose_listener()

