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
+ 신호등, 표지판, 도로선을 인식/검출하기 위한 파이캠, 웹캠
+ 차단바, 터널을 효과적으로 감지하기 위한 초음파센서
