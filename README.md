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

## 1. 개요
**1.1. 터틀봇3 오토레이스란**

![intro](readme_images/TurtleBot3-AutoRace_Intro.png)

+ 제공받은 플랫폼인 TURTLEBOT3를 이용하여 ROS기반 자율주행 알고리즘 개발
+ 다양한 센서를 활용해 로봇이 도로 환경에서 마주칠수 있는 상황에 대처할 수 있도록 프로그래밍

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

## 2. 주행 알고리즘
**2.1. 전체 알고리즘 개략도**

![algorithm](Algorithm.png)

+ 로봇 구동시 라인트레이싱 / 각 미션의 시작을 알리는 Flag 탐색 시작
+ 신호등 구간의 Flag: 지정한 범위의 HSV값을 갖는 Blob
+ 주차 구간의 Flag: 좌측면에 장착된 파이캠 Image의 ROI(Region of Interest)내에서 주차 표지판 이미지와 겹치는 부분
+ 차단바 구간의 Flag: 전방에 장착된 초음파 센서가 감지하는 20cm 이하의 값
+ 터널 구간의 Flag: 상단에 장착된 초음파 센서가 감지하는 15cm 이하의 값

**2.2. 센서와 노드간 통신 개략도**

![nodes](ROS_nodes.png)

+ 센서들은 각자 정해진 topic이름으로 자신이 감지하는 data를 발행
