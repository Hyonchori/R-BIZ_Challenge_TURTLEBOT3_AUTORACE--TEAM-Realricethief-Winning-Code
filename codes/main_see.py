#!/usr/bin/env python


import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import Float32


# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError



rospy.init_node('video_processor', anonymous=True)

######################################################################################################
####################<<< MODULE that we made >>>#######################################################


import turtle_video_siljun

pub_stage=rospy.Publisher('/stage',Int8,queue_size=5)
pub_sinho=rospy.Publisher('/sinho_state',Int8MultiArray,queue_size=5)


######################################################################################################
###############################<<< Trainnig parking sign >>>##########################################


orb = cv2.ORB_create()
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

imgTrainColor=cv2.imread('parking.jpg')
imgTrainGray = cv2.cvtColor(imgTrainColor, cv2.COLOR_BGR2GRAY)

kpTrain = orb.detect(imgTrainGray,None)
kpTrain, desTrain = orb.compute(imgTrainGray, kpTrain)


###################################################################################
####################<<< Initial Value Definition >>>###############################


time = rospy.Duration(1.6)

lower_red=np.array([165,160,115]) # HSV Ranges used in sinho
upper_red=np.array([185,255,255])
lower_green=np.array([60,75,140])
upper_green=np.array([68,255,255])

lower_blue=np.array([85,150,90]) # HSV Ranges used in jucha   (97,40,60)~(115,140,140) when screen down
upper_blue=np.array([105,230,170])

dist_chadan=100 # Sonar sensor initial values
dist_tunnel=100

f_g=1; f_r=0; s_g=0 # Constants in sinho

match_len=0; line_count=0; park_count=0; lt=0 # Constants in jucha

angular_vel=0 # initial angular vel
angular=0
stage=100 # 100 is nomal (0=sinho, 1=jucha, 2=chadan, 3=tunnel)




######################################################################################
#################################<<< Funtions >>>#####################################


def state_jucha(num): ### Function that save value used in jucha from 'in_jucha'
	global number
	number=num



def chadan_dist(distance_chadan): ### Function that save Sonar data used in chadan from sonar sensor
	global dist_chadan
	dist_chadan=distance_chadan.data



def tunnel_dist(distance_tunnel): ### Function that save Sonar data used in tunnel from sonar sensor
	global dist_tunnel
	dist_tunnel=distance_tunnel.data



def shinho(blob_ROI,stage,angular): ###Function that run when stage=0

	global f_r; global s_g; global sinho_state
	
	sinho_state=Int8MultiArray()

	if f_g==1 and f_r==0 and s_g==0:		
		keypoints_red=turtle_video_siljun.find_color(blob_ROI,lower_red,upper_red,stage)  
		print('first green signal detected.')

		if keypoints_red:
			f_r=1
			sinho_state.data=np.array([1,0,1])
			pub_sinho.publish(sinho_state)
		else:
			sinho_state.data=np.array([0,0,0])
			pub_sinho.publish(sinho_state)
		return 0

	if f_g==1 and f_r==1 and s_g==0:
		keypoints_green=turtle_video_siljun.find_color(blob_ROI,lower_green,upper_green,stage)	
		print('red signal detected. waiting secondary green signal.')

		sinho_state.data=np.array([1,0,0])
		pub_sinho.publish(sinho_state)
		if keypoints_green:						
			s_g=1
			sinho_state.data=np.array([1,1,1])
			pub_sinho.publish(sinho_state)
		return 0

	if f_g==1 and f_r==1 and s_g==1:
		print('second green signal detected.')
		s_g=2
		sinho_state.data=np.array([1,2,0])
		pub_sinho.publish(sinho_state)
		return 100


def jucha(num,angular): ### Function that run when stage=1

	global line_count; global park_count; global lt;
	print(line_count)

	if line_count==0:

		if num[0]==0:
			return 1

		else:
			line_count=1
			return 1

	elif line_count==1:

		if num[0]==1 and lt==0:
			return 1

		elif num[0]==0 and lt==0:

			if num[1]==1:
				park_count=1
				lt=1
				return 100

			else:
				lt=1
				return 1
			
		else:
			if num[0]==1:				
				line_count=2
				return 1
			return 1

	elif line_count==2:
		if num[0]==1 and lt==1:
			return 1

		elif num[0]==0 and park_count==0 and lt==1:
			lt=2
			return 100

		elif park_count==1 and lt==1:
			lt=2
			return 1

		else:
			if num[0]==1:
				line_count=3
				return 100

			return 1
	


def chadan(dist): ### Function that run when stage=2
	if dist<17 and dist>7:
		return 2
	else:
		return 100
	


def tunnel(dist): ### Function that run when stage=3
	if dist<15 and dist>7:		
		return 3
	else:
		return 100	




def Stage_Selecting(ros_data):
	global angular
	#color picking tool : https://alloyui.com/examples/color-picker/hsv/
    ####<<< Direct conversion to CV2 >>>####
	np_arr = np.fromstring(ros_data.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

	####<<< DEFINE_ROI >>>####
	blob_ROI=image_np[100:,:]
	parking_ROI=image_np

	####<<< Make variance global >>>####
	global stage
	global match_len
	angular=angular_vel

	############################<<< Stage Selecting >>>############################	
	if s_g<2 and stage==100:		
		keypoints_green=turtle_video_siljun.find_color(blob_ROI,lower_green,upper_green,0)
		if keypoints_green:
			stage=0
			print('sinho!')


	if line_count<3 and stage==100:
		keypoints_blue=turtle_video_siljun.find_color(parking_ROI,lower_blue,upper_blue,1)
		if keypoints_blue:
			match_len=turtle_video_siljun.parking_match(keypoints_blue,image_np,orb,bf,desTrain)
			print(match_len)
			if match_len>=3:
				stage=1
				print('jucha!')


	if stage==100:
		if dist_chadan<17 and dist_chadan>7 and dist_tunnel>12 :
			stage=2
			print('chadan!')

		elif dist_tunnel<12 and dist_tunnel>7:
			stage=3
			print('tunnel!')

	pub_stage.publish(stage)
	
	#########################<<< Select Function depening on stage >>##########################
	if stage==0:
		stage=shinho(blob_ROI,stage,angular)
	elif stage==1 and number:
		stage=jucha(number.data,angular)
	elif stage==2:
		stage=chadan(dist_chadan)
	elif stage==3:
		stage=tunnel(dist_tunnel)
	else:
		print("***********normal************")
	
	#########################<<< Show processed image >>>##############################
	cv2.imshow('video_picam',image_np)
	cv2.waitKey(1)&0xFF
	########################<<< Pub stage to other node >>>############################
	


######################################################################################################

rospy.Subscriber('/camera/image/compressed2',CompressedImage, Stage_Selecting,  queue_size = 1) ## Used for Detecting Sinho,Jucha
rospy.Subscriber('/chadan_sonar_dist', Float32, chadan_dist)
rospy.Subscriber('/tunnel_sonar_dist', Float32, tunnel_dist)
rospy.Subscriber('/jucha_state', Int8MultiArray, state_jucha)

rospy.spin()
	
######################################################################################################