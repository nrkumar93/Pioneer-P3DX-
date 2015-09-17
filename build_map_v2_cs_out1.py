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
import pickle

world=zeros((401,401))
count=0
grid_size_meter=0.25
multi_const=4
add_const=200

def callback(sonar_data,pose_data):
	l0=0
	update=log(0.7/0.3)
	global world, count, multi_const, add_const, grid_size_meter
	count+=1
	obj_x=[0.0]*8
	obj_y=[0.0]*8
	r=[0.0]*8
	obj_x_proj=[0.0]*8
	obj_y_proj=[0.0]*8
	obj_final_x=[0.0]*8
	obj_final_y=[0.0]*8
	theta=[1.57079,0.872664,0.523598,0.1745329,-0.1745329,-0.523598,-0.872664,-1.57079]
	sensor_range=5
	for i in range(8):
		obj_x[i]=sonar_data.points[i].x
		obj_y[i]=sonar_data.points[i].y
	bot_x=pose_data.pose.pose.position.x
	bot_y=pose_data.pose.pose.position.y
	bot_z=pose_data.pose.pose.orientation.z
	bot_w=pose_data.pose.pose.orientation.w
	bot_euler=tf.transformations.euler_from_quaternion([0,0,bot_z,bot_w])[2]
	print bot_euler
	for i in range(8):
		r[i]=sqrt(pow(obj_x[i],2)+pow(obj_y[i],2))
		obj_x_proj[i]=r[i]*cos(bot_euler+theta[i])
		obj_y_proj[i]=r[i]*sin(bot_euler+theta[i])
		obj_final_x[i]=obj_x_proj[i]+bot_x
		obj_final_y[i]=obj_y_proj[i]+bot_y
		m=(obj_final_y[i]-bot_y)/(obj_final_x[i]-bot_x)
		c=bot_y-m*bot_x
		if bot_x<obj_final_x[i]:
			x1=bot_x
			while x1<obj_final_x[i]:
				y1=m*x1+c
				x1_mat=int(x1*multi_const+add_const)#
				y1_mat=int(y1*multi_const+add_const)#
				if r[i]<sensor_range:
					world[x1_mat,y1_mat]=world[x1_mat,y1_mat]-update-l0
				x1=x1+grid_size_meter#
		else:
			x1=obj_final_x[i]+grid_size_meter#
			while x1<bot_x:
				y1=m*x1+c
				x1_mat=int(x1*multi_const+add_const)#
				y1_mat=int(y1*multi_const+add_const)#
				if r[i]<sensor_range:
					world[x1_mat,y1_mat]=world[x1_mat,y1_mat]-update-l0
				x1=x1+grid_size_meter#
		if r[i]<sensor_range:
			#print "obstacle"
			world[int(obj_final_x[i]*multi_const+add_const),int(obj_final_y[i]*multi_const+add_const)]=world[int(obj_final_x[i]*multi_const+add_const),int(obj_final_y[i]*multi_const+add_const)]+update-l0#
	world_prob=exp(world)/(1+exp(world))
	cv2.imshow('w1',world_prob)
	cv2.waitKey(10)
	print count	
	if count==2000:
		fout=open('rise_lab.txt','w')
		pickle.dump(world_prob,fout)
		fout.close()
		count=0
	

def sonar_pose_listener():
	rospy.init_node('sonar_pose_listener', anonymous=True)
	sonar_list=message_filters.Subscriber('/RosAria/sonar', PointCloud)
	pose_list=message_filters.Subscriber('/RosAria/pose', Odometry)
	ts=message_filters.TimeSynchronizer([sonar_list, pose_list],10)
	ts.registerCallback(callback)
	rospy.spin()

if __name__=='__main__':
	rospy.sleep(10)
	cv2.namedWindow('w1',cv2.CV_WINDOW_AUTOSIZE)
	sonar_pose_listener()

