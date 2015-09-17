#!/usr/bin/env python
import roslib
roslib.load_manifest('beginner_tutorials')
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
import tf
import message_filters
from pylab import *	
from random import gauss, random
from matplotlib import pyplot as plt1
import pickle
import cv2
import sys
from copy import copy,deepcopy

M=4000
bot_x_1_mat=100
bot_y_1_mat=100
bot_euler_1=0.0
x_1=[0]*M
y_1=[0]*M
alpha_1=[0]*M
count=0
wslow=0.0
wfast=0.0
f=open('data3.txt','r')
world=pickle.load(f)
f.close()
for i in range(201):
	for j in range(201):
		if world[i,j]>0.9:
			world[i,j]=1
		else:
			world[i,j]=0

world3=deepcopy(world)
for m in range(M):
	x_1[m]=int(random()*150)+25
	y_1[m]=int(random()*150)+25
	alpha_1[m]=random()*2*pi-pi
	#if alpha_1[m]>pi:
		#alpha_1[m]=alpha_1[m]-2*pi
		#print "gen",x_1[m],y_1[m],alpha_1[m]
	world3[x_1[m],y_1[m]]=2
cv2.imshow('w3',world3)
cv2.waitKey(10)
timestamp_1=0

def initialise():
	global M
	world3=deepcopy(world)
	for m in range(M):
		x_1[m]=int(random()*150)+25
		y_1[m]=int(random()*150)+25
		alpha_1[m]=random()*2*pi-pi
		#if alpha_1[m]>pi:
			#alpha_1[m]=alpha_1[m]-1.8*pi
		#print "gen",x_1[m],y_1[m],alpha_1[m]
		world3[x_1[m],y_1[m]]=2
	cv2.imshow('w3',world3)
	cv2.waitKey(10)

#particles for the first iteration not yet done!

def weighted_choice(w):
	totals=[]
	running_total=0
	for w1 in w:
		running_total+=w1
		totals.append(running_total)
	rndnum=random()*running_total
	for i, total in enumerate(totals):
		if rndnum<=total:
			return i

def normalpdf(x, mean, sd):
	var = float(sd)**2
	denom=(2*pi*var)**0.5
	num=exp(-(float(x)-float(mean))**2/(2*var))
	return num/denom

def callback(sonar_data, pose_data):
	print pose_data.header.stamp.secs
	global bot_x_1_mat, bot_y_1_mat, bot_euler_1, x_1, y_1, alpha_1, world, count, wslow, wfast, timestamp_1
	timestamp=pose_data.header.stamp.secs
	if timestamp in range(timestamp_1, timestamp_1+7):
		#timestamp_1=timestamp
		return
	timestamp_1=timestamp
	count+=1
	print count
	obj_x=[0.0]*8
	obj_y=[0.0]*8
	r=[0.0]*8
	obj_x_proj=[0.0]*8
	obj_y_proj=[0.0]*8
	obj_final_x=[0.0]*8
	obj_final_y=[0.0]*8
	obj_x_mat=[0]*8
	obj_y_mat=[0]*8
	theta=[1.57079,0.872664,0.523598,0.1745329,-0.1745329,-0.523598,-0.872664,-1.57079]
	sensor_range=4
	for i in range(8):
		obj_x[i]=sonar_data.points[i].x
		obj_y[i]=sonar_data.points[i].y
	bot_x=pose_data.pose.pose.position.x
	#print "bot_x",bot_x
	bot_y=pose_data.pose.pose.position.y
	bot_z=pose_data.pose.pose.orientation.z
	bot_w=pose_data.pose.pose.orientation.w
	bot_euler=tf.transformations.euler_from_quaternion([0,0,bot_z,bot_w])[2]
	#print bot_euler
	#print "hello",bot_euler, bot_euler_1
	bot_x_mat=int(bot_x*10+100)
	#print "bot",bot_x_mat
	bot_y_mat=int(bot_y*10+100)
	for i in range(8):
		r[i]=sqrt(pow(obj_x[i],2)+pow(obj_y[i],2))
		obj_x_proj[i]=r[i]*cos(bot_euler+theta[i])
		obj_y_proj[i]=r[i]*sin(bot_euler+theta[i])
		obj_final_x[i]=obj_x_proj[i]+bot_x
		obj_final_y[i]=obj_y_proj[i]+bot_y
		obj_x_mat[i]=int(obj_final_x[i]*10+100)
		obj_y_mat[i]=int(obj_final_y[i]*10+100)

	#x and y should be in decimeter, ie in matrix
	x=[0]*M
	y=[0]*M
	alpha=[0.0]*M
	w=[1]*M
	#print "hey", bot_x_mat, bot_x_1_mat
	#r_mov=sqrt(pow(bot_x_mat-bot_x_1_mat,2)+pow(bot_y_mat-bot_y_1_mat,2))
	if count==1:
		bot_x_1_mat=deepcopy(bot_x_mat)
		bot_y_1_mat=deepcopy(bot_y_mat)
		bot_euler_1=deepcopy(bot_euler)
	alpha_mov=bot_euler-bot_euler_1
	if alpha_mov>pi:
		alpha_mov=(alpha_mov-2*pi)
	if alpha_mov<-1*pi:
		alpha_mov=(alpha_mov+2*pi)
	#print r_mov, alpha_mov
	#bot_x_1_mat=deepcopy(bot_x_mat)
	#bot_y_1_mat=deepcopy(bot_y_mat)
	#bot_euler_1=deepcopy(bot_euler)
	wavg=0.0
	for m in range(M):
		#print m
		#motion model
		alpha[m]=alpha_1[m]+alpha_mov
		#x[m]=int(x_1[m]+r_mov*cos(alpha[m]))
		#y[m]=int(y_1[m]+r_mov*sin(alpha[m]))
		x[m]=x_1[m]+(bot_x_mat-bot_x_1_mat)
		y[m]=y_1[m]+(bot_y_mat-bot_y_1_mat)
		x[m]=int(x[m]+gauss(0,1.5))
		y[m]=int(y[m]+gauss(0,1.5))
		alpha[m]=alpha[m]+gauss(0,0.2)
		if alpha[m]>pi:
			alpha[m]=alpha[m]-2*pi
		if alpha[m]<-1*pi:
			alpha[m]=alpha[m]+2*pi
		x_1[m]=deepcopy(x[m])
		y_1[m]=deepcopy(y[m])
		alpha_1[m]=deepcopy(alpha[m])
		#print x[m],y[m],alpha[m]
		#measurement model 
		for i in range(8):
			slope=tan(alpha[m]+theta[i])
			c=y[m]-slope*x[m] #got the line along the sonar from the bot
			obstacle=False
			if (alpha[m]+theta[i]) < pi/2.0 and (alpha[m]+theta[i]) > -1*pi/2.0:
				x1=x[m]+1
				y1=slope*x1+c
				x1=int(x1)
				y1=int(y1)
				r1=sqrt(pow(x1-x[m],2)+pow(y1-y[m],2))
				#print "hello",x1,y1, r1
				obstacle=False
				global world
				while r1<40 and 0<x1<200 and 0<y1<200:
					#print "new",x1,y1
					if world[x1,y1]==1:
						zx=x1
						zy=y1
						obstacle=True
						break
					x1=x1+1
					y1=slope*x1+c
					x1=int(x1)
					y1=int(y1)
					r1=sqrt(pow(x1-x[m],2)+pow(y1-y[m],2))
			if (alpha[m]+theta[i]) > pi/2.0 or (alpha[m]+theta[i]) < -1*pi/2.0:
				#print alpha[m], pi/2.0, -1*pi/2.0
				x1=x[m]-1
				y1=slope*x1+c
				x1=int(x1)
				y1=int(y1)
				r1=sqrt(pow(x1-x[m],2)+pow(y1-y[m],2))
				#print "hello2", x1,y1, r1
				#print x[m],r1
				obstacle=False
				while r1<40 and 0<x1<200 and 0<y1<200:
					#print "new2",x1,y1
					if world[x1,y1]==1:
						zx=x1
						zy=y1
						obstacle=True
						break
					x1=x1-1
					y1=slope*x1+c
					x1=int(x1)
					y1=int(y1)
					r1=sqrt(pow(x1-x[m],2)+pow(y1-y[m],2))
			if obstacle:
				#print "omg!"
				zr=sqrt(pow(zx-x[m],2)+pow(zy-y[m],2))
				ztheta=alpha[m]+theta[i]
				r_orig=sqrt(pow(obj_y_mat[i]-(bot_y*10+100),2)+pow(obj_x_mat[i]-(bot_x*10+100),2))
				theta_orig=bot_euler+theta[i]
				pz_r_hit=normalpdf(r_orig,zr,5)
				pz_theta_hit=normalpdf(theta_orig,ztheta,0.4)
				pz_hit=pz_r_hit*pz_theta_hit
				w[m]=w[m]*pz_hit
				#print m,r_orig,zr,pz_r_hit,pz_theta_hit
				
			else:
				if r[i]<4:
					w[m]=w[m]*exp(-65)
				else:
					w[m]=w[m]*0.09 
		wavg=wavg+w[m]/M
	bot_x_1_mat=deepcopy(bot_x_mat)
	bot_y_1_mat=deepcopy(bot_y_mat)
	bot_euler_1=deepcopy(bot_euler)
	decay_slow=exp(-5)
	decay_fast=exp(-0.54)
	wslow=wslow+decay_slow*(wavg-wslow)
	wfast=wfast+decay_fast*(wavg-wfast)
	world1=deepcopy(world)
	print "wavg", wavg, "wslow", wslow, "wfast", wfast
	rand_samp_num=int(max(0,1-wfast/wslow)*1500)
	print rand_samp_num
	#rand_samp_num=0
	for m in range(M-rand_samp_num):
		#print w[m]
		index=weighted_choice(w)
		#print index
		x_1[m]=deepcopy(x[index])
		y_1[m]=deepcopy(y[index])
		alpha_1[m]=deepcopy(alpha[index])	
		#plt.plot(x_1[m],y_1[m],'r.')
		#plt.draw()
		world1[x_1[m],y_1[m]]=2
	for m in range(M-rand_samp_num,M):
		x_1[m]=int(random()*150)+25
		y_1[m]=int(random()*150)+25
		alpha_1[m]=random()*2*pi-pi
		#if alpha_1[m]>pi:
		#	alpha_1[m]=alpha_1[m]-1.8*pi
		world1[x_1[m],y_1[m]]=2
	
	#plt1.ion()
	#plt1.imshow(world1)
	#plt1.draw()	
	cv2.imshow('w3',world1)
	cv2.waitKey(2)
	#rospy.signal_shutdown('done')
	#if count==12:
	#	count=0
	#	initialise()
		
def location_listener():
	rospy.init_node('loc_listener',anonymous=True)
	sonar_list=message_filters.Subscriber('/RosAria/sonar', PointCloud)
	pose_list=message_filters.Subscriber('/RosAria/pose', Odometry)
	#while not rospy.is_shutdown():
	ts=message_filters.TimeSynchronizer([sonar_list, pose_list],1)
	#rospy.sleep(1)
	ts.registerCallback(callback)
	#print "hello there"
	#rospy.signal_shutdown('done')
	#plt.imshow(world1)
	#plt.draw()
	rospy.spin()

if __name__=='__main__':
	#plt.ion()
	cv2.namedWindow('w3',cv2.CV_WINDOW_AUTOSIZE)
	#world1[:,:,0]=world
	#world1[:,:,1]=world
	#world1[:,:,2]=world
	#world1=world1*255
	#cv2.imshow('w1',world)
	#cv2.waitKey(10)
	#plt.imshow(world)
	#plt.draw()
	location_listener()
