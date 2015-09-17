#!/usr/bin/env python
import roslib
roslib.load_manifest('beginner_tutorials')
import rospy
import cv2
import pickle
from copy import deepcopy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import message_filters
from pylab import *
import thread

'''
this code can create the path, given the initial and final distination coordinates,
it assumes that the odometry is perfect, it travels smooth, and explodes the obstacle grid to avoid collision
can navigate in dynamic environment, ie replan and navigate

Do not increase the sleep time for move/rot, as it will affect being real time
if you add extra code and if you lose real time, try reducing these sleep times 
and inorder to maintain same dist covered, may be u can try incresing the speed
'''
#what to do if it sees an obstacle, as soon as a new plan is made?

def dict_is_empty(d):
	for k in d:
		return False
	return True

#f=open('world_cs_out1.txt','r')
#world5=pickle.load(f)
#f.close()
world5=zeros((401,401))#
multi_const=4
add_const=200
grid_size_meter=0.25
x_length=world5.shape[0]
y_length=world5.shape[1]
explode_length=2
goal_x=200
goal_y=213
# here , for the present exp sake I am making unexplored area as obstacles, so that it does not get used while planning
for i in range(x_length):
	for j in range(y_length):
		if world5[i,j]>0.9:
			world5[i,j]=1
		elif world5[i,j]==0.5:
			world5[i,j]=2
		else:
			world5[i,j]=0
world=deepcopy(world5)
for i in range(x_length):
	for j in range(y_length):
		if world5[i,j]==1:
			for k in range(-1*explode_length,explode_length+1):
				for l in range(-1*explode_length,explode_length+1):
					world[i+k,j+l]=1
path1=[]
time_stamp_1=0
step_count=0
obstacle=False
go_to_prev_pos=False
rot_1=""

def find_path(start):	
	global world,world5,path1, step_count, x_length, y_length, goal_x, goal_y
	g={}
	d={}
	for i in range(x_length):
		for j in range(y_length):
			if world[i,j]==0:
				g[str(i)+' '+str(j)]={}

	for i in range(1,x_length-1):
		for j in range(1,y_length-1):
			if world[i,j]==0:
				if world[i+1,j]==0:
					g[str(i)+' '+str(j)][str(i+1)+' '+str(j)]=1
				if world[i,j+1]==0:
					g[str(i)+' '+str(j)][str(i)+' '+str(j+1)]=1
				if world[i-1,j]==0:
					g[str(i)+' '+str(j)][str(i-1)+' '+str(j)]=1
				if world[i,j-1]==0:
					g[str(i)+' '+str(j)][str(i)+' '+str(j-1)]=1
	start_x=int((start.split())[0])
	start_y=int((start.split())[1])
	world4=deepcopy(world5)
	world4[start_x,start_y]=2
	world4[goal_x,goal_y]=2
	cv2.imshow('w4',world4)
	cv2.waitKey(10)

	#start='100 100'
	end=str(goal_x)+' '+str(goal_y)
	q={start:0}
#	print "start",start, world[132,138]
	p={}
	while not dict_is_empty(q):
		prior_q=sorted([(v,k) for (k,v) in q.items()])
		first=prior_q[0]
		d[first[1]]=first[0]
		del q[first[1]]
		if first[1] == end:
			break
		for w in g[first[1]]:
			length=d[first[1]]+g[first[1]][w]
			if w not in d:
				if w not in q or length<q[w]:
					q[w]=length
					p[w]=first[1]

	path=[]
	while 1:
		path.append(end)
		if end==start:
			break
		end=p[end]
	path.reverse()
	path1=deepcopy(path)
	world3=deepcopy(world5)
	for node in path:
		node_i=node.split()
		nodex=int(node_i[0])
		nodey=int(node_i[1])
		world3[nodex,nodey]=2	
	cv2.imshow('w5',world3)
	cv2.waitKey(1000)
	step_count=0


def rotate(rot,bot_euler): 
	pub=rospy.Publisher('/RosAria/cmd_vel',Twist)
	twist=Twist()
	print "rotate"
	twist.linear.x=0.0
	if rot=="north":
		twist.angular.z= 0.01 +abs((bot_euler)-pi/2)*0.3
	elif rot=="south":
		twist.angular.z= 0.01+ (abs((bot_euler)+(pi/2)))*0.3
	elif rot=="east":
		twist.angular.z= 0.01+ (abs(bot_euler))*0.3
	elif rot=="west":
		twist.angular.z=0.01+(abs(abs(bot_euler)-pi))*0.3
	pub.publish(twist)
#	rospy.sleep(0.2)
#	twist.angular.z=0.0
#	pub.publish(twist)

def move():
	print "move"
	pub=rospy.Publisher('/RosAria/cmd_vel',Twist)
	twist=Twist()
	twist.angular.z=0.0
	twist.linear.x=0.1
	pub.publish(twist)
#	rospy.sleep(0.25)
#	twist.linear.x=0.0
#	pub.publish(twist)

def stop():
	print "stop"
	pub=rospy.Publisher('/RosAria/cmd_vel',Twist)
	twist=Twist()
	twist.linear.x=0.0
	twist.angular.z=0.0
	pub.publish(twist)


def check_obstacle(sonar_data,pose_data):
	global world, obstacle, multi_const, add_const, explode_length 
	#is_obstacle=[0]*8
	obj_x=[0.0]*8
	obj_y=[0.0]*8
	r=[0.0]*8
	obj_x_proj=[0.0]*8
	obj_y_proj=[0.0]*8
	obj_final_x_mat=[0.0]*8
	obj_final_y_mat=[0.0]*8
	theta=[1.57079,0.872664,0.523598,0.1745329,-0.1745329,-0.523598,-0.872664,-1.57079]
	#sensor_range=4
	for i in range(1,7):
		obj_x[i]=sonar_data.points[i].x
		obj_y[i]=sonar_data.points[i].y
	bot_x=pose_data.pose.pose.position.x
	bot_y=pose_data.pose.pose.position.y
	bot_z=pose_data.pose.pose.orientation.z
	bot_w=pose_data.pose.pose.orientation.w
	bot_euler=tf.transformations.euler_from_quaternion([0,0,bot_z,bot_w])[2]
	obstacle=False
	for i in range(1,7):
		r[i]=sqrt(pow(obj_x[i],2)+pow(obj_y[i],2))
		obj_x_proj[i]=r[i]*cos(bot_euler+theta[i])
		obj_y_proj[i]=r[i]*sin(bot_euler+theta[i])
		obj_final_x_mat[i]=int((obj_x_proj[i]+bot_x)*multi_const) +add_const
		obj_final_y_mat[i]=int((obj_y_proj[i]+bot_y)*multi_const)+add_const
		#print bot_x*10+100, obj_final_x_mat, bot_y*10+100, obj_final_y_mat
		print "r", r[i]
		if r[i]<0.5 and r[i]!=0:
			#print "r",i,r[i]
			obstacle=True
			print "obstacle hit"
			thread.start_new_thread(stop,())
			#is_obstacle[i]=1
			for k in range(-1*explode_length,explode_length+1):
				for l in range(-1*explode_length,explode_length+1):
					world[obj_final_x_mat[i]+k,obj_final_y_mat[i]+l]=1
	if obstacle:
		new_pos=path1[step_count-1].split()
		world[int(new_pos[0]), int(new_pos[1])]=0
	#obstacle=False
#	print int(bot_x*10+100), obj_final_x_mat, int(bot_y*10+100), obj_final_y_mat
	#for i in range(8):
	#	if is_obstacle[i]==1:
	#		obstacle=True
			#thread.start_new_thread(stop,())
	#		break
	print "obstacle"
	
def callback(sonar_data,pose_data):
	global step_count, time_stamp_1, path1, go_to_prev_pos, obstacle, multi_const, add_const, rot_1
	#print obstacle
	if not go_to_prev_pos:
		check_obstacle(sonar_data,pose_data)
	if step_count==len(path1)-1:
		thread.start_new_thread(stop,())
		print "done"
		return
	print pose_data.header.stamp.secs
	time_stamp=pose_data.header.stamp.secs
#	if time_stamp==time_stamp_1:
#		return
	time_stamp_1=time_stamp
	bot_x=pose_data.pose.pose.position.x
	bot_y=pose_data.pose.pose.position.y
	bot_z=pose_data.pose.pose.orientation.z
	bot_w=pose_data.pose.pose.orientation.w
	#print bot_z,bot_w
	bot_euler=tf.transformations.euler_from_quaternion([0,0,bot_z,bot_w])[2]	
	#print bot_euler
	bot_x_mat=int(bot_x*multi_const)+add_const	#
	bot_y_mat=int(bot_y*multi_const)+add_const	#
#	print step_count
	curr_pos_count=path1[step_count].split()
	curr_pos_x_count=int(curr_pos_count[0])
	curr_pos_y_count=int(curr_pos_count[1])
	curr_pos_x=bot_x_mat
	curr_pos_y=bot_y_mat
	print "obstacle", obstacle
	if obstacle:
		print "inside go_to_prev"
		go_to_prev_pos=True
	if go_to_prev_pos:
		next_pos=path1[step_count-1].split()
		#else:
		#	next_pos=curr_pos_count
		#	go_to_prev_pos=False
	else:
		next_pos=path1[step_count+1].split()
	print "bot_x_mat, bot_y_mat,curr-pos,nex_pos",bot_x_mat, bot_y_mat,curr_pos_x,curr_pos_y,next_pos
	next_pos_x=int(next_pos[0])
	next_pos_y=int(next_pos[1])
	if not (bot_x_mat==curr_pos_x and bot_y_mat==curr_pos_y):	
		if not (bot_x_mat==next_pos_x and bot_y_mat==next_pos_y):
			print "going wrong :("
	print step_count, obstacle, go_to_prev_pos
	rot=""
	if bot_x_mat==next_pos_x and bot_y_mat==next_pos_y:
		if go_to_prev_pos:
			thread.start_new_thread(stop,())
			start=str(bot_x_mat)+' '+str(bot_y_mat)
			find_path(start)
			go_to_prev_pos=False
		else:
			step_count+=1
	else:
		if curr_pos_x==next_pos_x:
			if next_pos_y>curr_pos_y:
				rot="north"
			if next_pos_y<curr_pos_y:
				rot="south"
		if curr_pos_y==next_pos_y:
			if next_pos_x>curr_pos_x:
				rot="east"
			if next_pos_x<curr_pos_x:
				rot="west"
		print rot, bot_euler
		if rot=="north":
			if not (bot_euler<(pi/2+0.015) and bot_euler>(pi/2-0.015)):
				thread.start_new_thread(rotate,(rot,bot_euler))
			else:
				thread.start_new_thread(move,())
		if rot=="south":
			if not ( bot_euler>(-1*pi/2-0.015) and bot_euler<(-1*pi/2+0.015)):
				thread.start_new_thread(rotate,(rot,bot_euler))
			else:
				thread.start_new_thread(move,())
		if rot=="east":
			if not ( bot_euler<0.015 and bot_euler>-0.015):
				thread.start_new_thread(rotate,(rot,bot_euler))
			else:
				thread.start_new_thread(move,())
		if rot=="west":
			if not ( bot_euler>(pi-0.015) or bot_euler<(-1*pi+0.015)):
				thread.start_new_thread(rotate,(rot,bot_euler))
			else:
				thread.start_new_thread(move,())

def navigator():
	rospy.init_node('navigator', anonymous=True)
	sonar_listener=message_filters.Subscriber('/RosAria/sonar',PointCloud)
	pose_listener=message_filters.Subscriber('/RosAria/pose',Odometry)
	ts=message_filters.TimeSynchronizer([sonar_listener, pose_listener],10)
	ts.registerCallback(callback)
#	rospy.Subscriber('/RosAria/pose',Odometry,callback)
	rospy.spin()
if __name__=='__main__':
	start='200 200'
	rospy.sleep(10)
	find_path(start)
	navigator()
