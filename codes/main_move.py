#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int8
from sensor_msgs.msg import CompressedImage
import turtle_video_siljun


rospy.init_node('main', anonymous=True)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)
pub_stage=rospy.Publisher('/stage',Int8,queue_size=5)
angular=0


##########################################################################################
###########################<<< Trainnig parking sign >>>##################################

orb = cv2.ORB_create()
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

imgTrainColor=cv2.imread('parking.jpg')
imgTrainGray = cv2.cvtColor(imgTrainColor, cv2.COLOR_BGR2GRAY)

kpTrain = orb.detect(imgTrainGray,None)
kpTrain, desTrain = orb.compute(imgTrainGray, kpTrain)

#########################################################################################
###########################<<< Initial Value Definition >>>##############################

time = rospy.Duration(1.6)

lower_red=np.array([170,160,115]) # HSV Ranges used in sinho
upper_red=np.array([180,255,255])
lower_green=np.array([70,75,140])
upper_green=np.array([88,255,255])

lower_blue=np.array([97,100,45]) # HSV Ranges used in jucha
upper_blue=np.array([115,255,240])

dist_chadan=100 # Sonar sensor initial values
dist_tunnel=100

f_g=1; f_r=0; s_g=0 # Constants in sinho

match_len=0; line_count=0; park_count=0; lt=0

angular_vel=0 # initial angular vel
angular=0
stage=100 # 100 is nomal (0=sinho, 1=jucha, 2=chadan, 3=tunnel)

sinho_state=0



#########################################################################################
###################################<<< Functions >>>#####################################

def turtlestop(): ### Function that stop the turtlebot3 when node is shut down

	twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)



def turtlemove(linear,angular): ### Function that move the turtlebot

	rospy.on_shutdown(turtlestop)
	#print(angular)
	twist=Twist()
	twist.linear.x=linear
	twist.angular.z=angular
	pub.publish(twist)



def state_sin(data):
	global sinho_state
	sinho_state=data.data



def state_jucha(num): ### Function that save value used in jucha from 'in_jucha'
	global number
	number=num



def chadan_dist(distance_chadan): ### Function that save Sonar data used in chadan from sonar sensor
	global dist_chadan
	dist_chadan=distance_chadan.data



def tunnel_dist(distance_tunnel): ### Function that save Sonar data used in tunnel from sonar sensor
	global dist_tunnel
	dist_tunnel=distance_tunnel.data
	


def shinho(sinho_state,angular): ###Function that run when stage=0
	global f_r; global s_g

	f_r=sinho_state[0]
	s_g=sinho_state[1]
	keypoints=sinho_state[2]

	if f_g==1 and f_r==0 and s_g==0:		
		#keypoints_red=turtle_video_siljun.find_color(blob_ROI,lower_red,upper_red,stage)  
		print('first green signal detected.')

		if keypoints==1:
			f_r=1
		else:
			turtlemove(0.07,angular)

		return 0

	if f_g==1 and f_r==1 and s_g==0:
		#keypoints_green=turtle_video_siljun.find_color(blob_ROI,lower_green,upper_green,stage)	
		print('red signal detected. waiting secondary green signal.')
		turtlemove(0,0)

		if keypoints==1:						
			s_g=1
		return 0

	if f_g==1 and f_r==1 and s_g==1:
		print('second green signal detected.') 
		turtlemove(0.08,angular)
		s_g=2
		return 100
		


def jucha(num,angular): ### Function that run when stage=1

	global line_count; global park_count; global lt;
	print(line_count)

	if line_count==0:

		if num[0]==0:
			turtlemove(0.1,angular)
			return 1

		else:
			line_count=1
			return 1

	elif line_count==1:

		if num[0]==1 and lt==0:
			turtlemove(0.1,angular)
			return 1

		elif num[0]==0 and lt==0:

			if num[1]==1:
				turtlemove(0.11,-0.7)
				rospy.sleep(rospy.Duration(2))
				turtlemove(0.1,0)
				rospy.sleep(rospy.Duration(1.7))
				turtlemove(0,0)
				rospy.sleep(time)
				turtlemove(-0.1,0)
				rospy.sleep(rospy.Duration(1.7))
				turtlemove(-0.11,0.7)
				rospy.sleep(rospy.Duration(2))
				turtlemove(0,0)	
				park_count=1
				lt=1
				return 100

			else:
				lt=1
				return 1
			
		else:	
			if angular<1.9:		
				turtlemove(0.09,angular)
			else:
				turtlemove(0.06,0)

			if num[0]==1:				
				line_count=2
				return 1

			return 1

	elif line_count==2:

		if num[0]==1 and lt==1:
			turtlemove(0.09,angular)
			return 1

		elif num[0]==0 and park_count==0 and lt==1:
			turtlemove(0.11,-0.7)
			rospy.sleep(rospy.Duration(2))
			turtlemove(0.1,0)
			rospy.sleep(rospy.Duration(2))
			turtlemove(0,0)
			rospy.sleep(time)
			turtlemove(-0.1,0)
			rospy.sleep(rospy.Duration(2))
			turtlemove(-0.11,0.7)
			rospy.sleep(rospy.Duration(2))
			turtlemove(0,0)
			lt=2
			return 100

		elif park_count==1 and lt==1:
			lt=2
			return 1
		else:
			turtlemove(0.09,angular)

			if num[0]==1:
				line_count=3
				print(line_count)
				return 100

			return 1



def chadan(dist): ### Function that run when stage=2

	print(dist)
	
	if dist<20 and dist>5:
		turtlemove(0,0)
		return 2

	else:
		rospy.sleep(rospy.Duration(2))
		return 100
	


def tunnel(dist): ### Function that run when stage=3

	print(dist)
	
	if dist<15 and dist>6:		
		return 3

	else:
		return 100



def checking_stage(ss): ### Function that save the stage value from 'main'
	global stage
	stage=ss.data



def angular_Selecting(ros_data): #Function setting angular velocity

	global angular; global stage
	np_arr = np.fromstring(ros_data.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

	K = np.array([[  220,     0.  ,  320],
              	  [    0. ,   200,   240],
              	  [    0. ,   0.  ,  1. ]])
	# zero distortion coefficients work well for this image
	D = np.array([0., 0., 0., 0.])

	# use Knew to scale the output
	Knew = K.copy()
	Knew[(0,1), (0,1)] = 0.5 * Knew[(0,1), (0,1)]
	img_undistorted = cv2.fisheye.undistortImage(image_np, K, D=D, Knew=Knew)

	####<<< Selecting angular velocity >>>####
	image_np, angular=turtle_video_siljun.line_trace(img_undistorted,100,verbose=False)
	
	if stage==0:		
		stage=shinho(sinho_state, angular)

	elif stage==1 and number:
		stage=jucha(number.data, angular)

	elif stage==2:
		stage=chadan(dist_chadan)

	elif stage==3:
		stage=tunnel(dist_tunnel)
		
	elif stage==100:
		print("**********normal************")
		if angular>0.23 or angular<-0.23:
			turtlemove(0.11,angular)
			print('curve')

		else:
			turtlemove(0.13,angular)
		if dist_chadan<20 and dist_chadan>5 and dist_tunnel>12 :
			stage=2
			print('chadan!')

		elif dist_tunnel<12 and dist_tunnel>7:
			stage=3
			print('tunnel!')

	pub_stage.publish(stage)

	#########################<<< Show processed image >>>##############################
	cv2.imshow('undistorted', img_undistorted)
	cv2.waitKey(1)&0xFF



#####################################################################################################

rospy.Subscriber('/camera/image/compressed',CompressedImage, angular_Selecting,  queue_size = 1) ## Used for Line Tracing
rospy.Subscriber('/stage',Int8,checking_stage)
rospy.Subscriber('/jucha_state',Int8MultiArray,state_jucha)
rospy.Subscriber('/sinho_state',Int8MultiArray,state_sin)
rospy.Subscriber('/chadan_sonar_dist',Float32,chadan_dist)
rospy.Subscriber('/tunnel_sonar_dist',Float32,tunnel_dist)

rospy.spin()

#####################################################################################################