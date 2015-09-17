#!/usr/bin/env python
import roslib
roslib.load_manifest('beginner_tutorials')
import rospy
import cv2
import pickle
from copy import deepcopy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from pylab import *
import thread

'''
this code can create the path given, the initial and final distination coordinates,
it assumes that the odometry is perfect, it travels smooth, and explodes the obstacle grid to avoid collision

Do not increase the sleep time for move/rot, as it will affect being real time
if you add extra code and if you lose real time, try reducing these sleep times 
and inorder to maintain same dist covered, may be u can try incresing the speed
'''


def dict_is_empty(d):
	for k in d:
		return False
	return True

f=open('data3.txt','r')
world5=pickle.load(f)
f.close()

g={}
d={}
for i in range(201):
	for j in range(201):
		if world5[i,j]>0.9:
			world5[i,j]=1
		else:
			world5[i,j]=0
			#g[str(i)+' '+str(j)]={}
			#c[str(i)+str(j)]=float("inf")

world=deepcopy(world5)
for i in range(201):
	for j in range(201):
		if world5[i,j]==1:
			for k in range(-3,4):
				for l in range(-3,4):
					world[i+k,j+l]=1

for i in range(201):
	for j in range(201):
		if world[i,j]==0:
			g[str(i)+' '+str(j)]={}

for i in range(1,200):
	for j in range(1,200):
		if world[i,j]==0:
			if world[i+1,j]==0:
				g[str(i)+' '+str(j)][str(i+1)+' '+str(j)]=1
			if world[i,j+1]==0:
				g[str(i)+' '+str(j)][str(i)+' '+str(j+1)]=1
			if world[i-1,j]==0:
				g[str(i)+' '+str(j)][str(i-1)+' '+str(j)]=1
			if world[i,j-1]==0:
				g[str(i)+' '+str(j)][str(i)+' '+str(j-1)]=1
			
#print g
world4=deepcopy(world5)
world4[100,100]=2
world4[142,130]=2
cv2.imshow('w4',world4)
cv2.waitKey(100)

start='100 100'
end='142 130'
#print world[120,120]
q={start:0}
p={}
while not dict_is_empty(q):
	prior_q=sorted([(v,k) for (k,v) in q.items()])
	#print prior_q
	first=prior_q[0]
	d[first[1]]=first[0]
	del q[first[1]]
	#print first
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
path1=path
#print path
world3=deepcopy(world5)
for node in path:
	node_i=node.split()
	nodex=int(node_i[0])
	nodey=int(node_i[1])
	world3[nodex,nodey]=2	
cv2.imshow('w5',world3)
cv2.waitKey(5000)
step_count=0
time_stamp_1=0


def rotate(rot,bot_euler): 
	pub=rospy.Publisher('/RosAria/cmd_vel',Twist)
	twist=Twist()
	print "rotate", abs(bot_euler)*0.5
	twist.linear.x=0.0
	if rot=="north":
		twist.angular.z= 0.01 +abs((bot_euler)-pi/2)*0.5
	elif rot=="south":
		twist.angular.z= 0.01+ (abs((bot_euler)+(pi/2)))*0.5
	elif rot=="east":
		twist.angular.z= 0.01+ (abs(bot_euler))*0.5
	elif rot=="west":
		twist.angular.z=0.01+(abs(abs(bot_euler)-pi))*0.5
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
#	rospy.sleep(0.45)
#	twist.linear.x=0.0
#	pub.publish(twist)

def stop():
	pub=rospy.Publisher('/RosAria/cmd_vel',Twist)
	twist=Twist()
	twist.linear.x=0.0
	twist.angular.z=0.0
	pub.publish(twist)

def callback(pose_data):
	global step_count, time_stamp_1
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
	bot_x_mat=int(bot_x*10+100)
	bot_y_mat=int(bot_y*10+100)
	curr_pos=path1[step_count].split()
	curr_pos_x=int(curr_pos[0])
	curr_pos_y=int(curr_pos[1])
	next_pos=path1[step_count+1].split()
	next_pos_x=int(next_pos[0])
	next_pos_y=int(next_pos[1])
	if not (bot_x_mat==curr_pos_x and bot_y_mat==curr_pos_y):
		if not (bot_x_mat==next_pos_x and bot_y_mat==next_pos_y):
			print "going wrong :("
	print step_count
	rot=""
	if bot_x_mat==next_pos_x and bot_y_mat==next_pos_y:
		step_count+=1
	else:
		if curr_pos_x==next_pos_x:
			if next_pos_y==curr_pos_y+1:
				rot="north"
			if next_pos_y==curr_pos_y-1:
				rot="south"
		if curr_pos_y==next_pos_y:
			if next_pos_x==curr_pos_x+1:
				rot="east"
			if next_pos_x==curr_pos_x-1:
				rot="west"
		#print rot, bot_euler
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
	rospy.Subscriber('/RosAria/pose',Odometry,callback)
	rospy.spin()
if __name__=='__main__':
	navigator()
