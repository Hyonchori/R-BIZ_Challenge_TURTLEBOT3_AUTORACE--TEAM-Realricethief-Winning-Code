# R-BIZ Challenge TURTLEBOT3 AUTORACE / Realricethief Winning-Code

팀명: 레알밥도둑 / 
로봇명: 간장게장

![teamlogo](/teamlogo.png) <수제 팀로고>

조원 : [이도규](https://github.com/ldkl123), [정민재](https://github.com/mj585), [정현철](https://github.com/junghyeonchiul), [조민수](https://github.com/KoG-8)

연락처 : 정민재(010-3833-5688), 이도규(010-2818-6732), 정현철(010-9162-2075)

## 목차
1. 개요

2. 주행 알고리즘

3. 알고리즘 설명
***
***

## 1. 개요
**1.1. 터틀봇3 오토레이스란**

![intro](readme_images/TurtleBot3-AutoRace_Intro.png)

+ 제공받은 플랫폼인 TURTLEBOT3를 이용하여 ROS기반 자율주행 알고리즘 개발
+ 다양한 센서 활용해 로봇이 도로 환경에서 마주칠수 있는 상황에 대처할 수 있도록 프로그래밍

**1.2. 주요 미션 구간**

+ 신호등 구간: 빨강, 주황, 초록 삼색의 불빛에 따라 정지, 감속, 주행
+ 주차 구간: 주차 표지판 이후에 나오는 주차 공간에 장애물을 피하여 주차 후 주행
+ 차단바 구간: 주행중에 차단바가 내려오면 정지, 차단 바가 올라가면 다시 주행
+ 터널: 내부의 장애물 배치를 모른 상태로 터널을 들어간 뒤 자율적으로 탈출

**1.3. 사용 센서**
![robot](Hardware_item.png)

+ 기존 TURTLEBOT3에 첨부된 Lidar이외에 별도의 센서들 장착
+ 라인트레이싱 사용 센서: Raspberry-Pi-Cam Fish-Eye
+ 신호등 구간 사용 센서: Rasberry-Pi-Cam Noir
+ 주차 구간 사용 센서: Rasberry-Pi-Cam Noir, C920 Web Cam 
+ 차단바 구간 사용 센서: HS-SR04 Ultrasonic Sensor
+ 터널 구간 사용 센서: HS-SR04 Ultrasoni Sensor, LDS-01 Lidar

***
***
## 2. 주행 알고리즘
**2.1. 전체 알고리즘 개략도**

![algorithm](Algorithm.png)

+ 로봇 구동시 라인트레이싱 / **각 미션의 시작을 알리는 Flag 탐색 시작**
+ **신호등 구간의 Flag**: 지정한 범위의 HSV값을 갖는 Blob
+ **주차 구간의 Flag**: 좌측면에 장착된 파이캠 Image의 ROI(Region of Interest)내에서 주차 표지판 이미지와 겹치는 부분
+ **차단바 구간의 Flag**: 전방에 장착된 초음파 센서가 감지하는 20cm 이하의 값
+ **터널 구간의 Flag**: 상단에 장착된 초음파 센서가 감지하는 15cm 이하의 값

**2.2. 센서와 노드간 토픽 통신 개략도**

![nodes](ROS_nodes.png)

+ 센서들은 각자 정해진 topic이름으로 자신이 감지하는 data를 발행
+ topic으로 발행되는 data들을 처리하여 각 미션을 통과하는 ROS_node를 만듦 
+ TURTLEBOT은 /cmd_vel topic을 이용하여 속도로 제어

***
***
## 3. 알고리즘 설명
**3.1. 라인트레이싱 (main_move.py / turtle_video.py)**

![distort](/readme_images/distorted_image.png)

+ 라인트레이싱에 사용되는 Fisheye Image는 시야각이 넓은 대신 왜곡이 매우 심하여 Line검출이 어려움 
+ 따라서 왜곡을 보정해주는 작업이 필요
***
~~~
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
~~~
![distort](/readme_images/undistorted.png)
+ Camera Matrix를 이용하여 왜곡을 보정한 모습
***
~~~
gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) ### Process image to make finding line easy
ROI=gray[350:,140:500]
ROI=cv2.GaussianBlur(ROI,(7,7),0)
thr=cv2.adaptiveThreshold(ROI,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
blur=cv2.medianBlur(thr,9)	
edge=cv2.Canny(blur,180,360)
~~~
+ 왜곡을 보정한 Image를 각종 Blur, Thresholding, Canny_Edge 처리
***
~~~
	left_edge=edge[:,:edge.shape[1]/2] ### in left side, it finds only '/'type when jucha stage
	right_edge=edge[:,edge.shape[1]/2:] ### in right side, it finds only '\'type when jucha stage
	L_lines=cv2.HoughLines(left_edge,1,np.pi/180,30)
	R_lines=cv2.HoughLines(right_edge,1,np.pi/180,30)
~~~
![distort](/readme_images/undistorted_image.png)
+ HoughLines함수를 통해 도로에 표시된 직선을 검출
***
~~~
	lineL=[] ### value initializing
	lineR=[]
	L=0	
	R=0
	i=0
	Ldegree=0
	Rdegree=0

	if R_lines is not None:
		R_lines=[l[0] for l in R_lines]
		for rho,theta in R_lines:
    			a = np.cos(theta)
    			b = np.sin(theta)
    			x0 = a*rho
    			y0 = b*rho
    			x1 = int(x0 + 1000*(-b))
    			y1 = int(y0 + 1000*(a))
    			x2 = int(x0 - 1000*(-b))
    			y2 = int(y0 - 1000*(a))
			degree=np.arctan2(y2-y1,x2-x1)*180/np.pi
			if degree>10 and R==0:
				i+=1
				Rdegree=degree
				R+=2
				cv2.line(frame,(x1+320,y1+160),(x2+320,y2+160),(0,100,100),3)
				break
			else:
				continue
		
	if L_lines is not None:
		L_lines=[l[0] for l in L_lines]
		for rho,theta in L_lines:
    			a = np.cos(theta)
    			b = np.sin(theta)
    			x0 = a*rho
    			y0 = b*rho
    			x1 = int(x0 + 1000*(-b))
    			y1 = int(y0 + 1000*(a))
    			x2 = int(x0 - 1000*(-b))
    			y2 = int(y0 - 1000*(a))
			degree=np.arctan2(y2-y1,x2-x1)*180/np.pi
			if degree<-10 and L==0:	
				i+=1
				Ldegree=degree
				L+=2
				cv2.line(frame,(x1+170,y1+160),(x2+170,y2+160),(0,100,100),3)
				break
			else:
				continue
~~~
~~~
	if i==2:
		return frame,-(Ldegree+Rdegree)*0.05 ### if there are two lines, then angular_vel depends on difference of angle
	elif i==1:
		if Ldegree==0:
			return frame,-(Rdegree-92)*0.04
		else:
			return frame,-(Ldegree+92)*0.04
	else:
		return frame,-0.001
~~~
+ 검출한 직선이 이루는 각도를 구한 뒤 그 값으로 로봇의 속도를 P제어
***

**3.2. 신호등 구간 (main_see.py / turtle_video.py / blob_param.py)**

![sinho](/readme_images/sinho_green.png)
+ 신호등의 초기 상태인 초록 불빛을 로봇이 발견하면 신호등 구간으로 인식하게 한다.
***
~~~
	np_arr = np.fromstring(ros_data.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

	####<<< DEFINE_ROI >>>####
	blob_ROI=image_np[100:,:]
	parking_ROI=image_np
	
	############################<<< Stage Selecting >>>############################	
	if s_g<2 and stage==100:		
		keypoints_green=turtle_video_siljun.find_color(blob_ROI,lower_green,upper_green,0)
		if keypoints_green:
			stage=0
			print('sinho!')
~~~
~~~
def find_color(frame,lower,upper,stage):  ### Color detecting to find sinho_signal or jucha_sign

	detector = blob_param_siljun.setting(stage)
	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) ### process rgb_image to hsv_image 
	mask_red = cv2.inRange(hsv,lower,upper)
	reversmask = 255-mask_red                   ### Detect blobs
	keypoints = detector.detect(reversmask)
	
	if stage==0:
		cv2.imshow('zzz',reversmask)
        
	if len(keypoints)>0 and stage==1:
		point=[]	
		for i in keypoints:
			point.append(i.pt)	
		return point
    
	elif stage==0:	
		return keypoints
    
	#return keypoints ### return whether it finds color
~~~
~~~
def setting(stage):### stage=0 -> shingho , stage=1->parking

	if stage==0: #shinho
		# Setup SimpleBlobDetector parameters.
		params = cv2.SimpleBlobDetector_Params()
		 
		# Change thresholds
		params.minThreshold = 0;
		params.maxThreshold = 256;
		 
		# Filter by Area.
		params.filterByArea = True
		params.minArea = 500
		params.maxArea=2300
		 
		# Filter by Circularity
		params.filterByCircularity = True
		params.minCircularity = 0.4
		 
		# Filter by Convexity
		params.filterByConvexity = True
		params.minConvexity = 0.1
		 
		# Filter by Inertia
		params.filterByInertia = False
		params.minInertiaRatio = 0.01
		 
		# Create a detector with the parameters
		ver = (cv2.__version__).split('.')
		if int(ver[0]) < 3 :
		    detector = cv2.SimpleBlobDetector(params)
		else : 
		    detector = cv2.SimpleBlobDetector_create(params)
		return detector
~~~
+ main_see.py에서 Pi-Cam Noir가 받은 Image를 turtle_video.py의 함수로 처리
+ turtle_video.py에서 blob_param.py에 있는 parameter사용
+ Image에 설정한 크기의 blob을 발견했을 경우 keypoint를 return
*** 
![sinho](/readme_images/sinho_blob.png=320x240)
