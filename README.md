
# ROS2_turtlebot3_fork lift_project

## 프로제트 기간 : 2021. 11. 01 ~ 2021. 11. 29

### 환경구성 : Ubuntu20.04, ROS2 foxy, Turtlebot3

### 사용기술 : Open_CV, aruco marker, rviz2, python, Arduino Serial Bus

#### 팀원 : 이주현, 조민석
  - 지도교수 : ground_Zero

## 배경 

○ 무인지게차는 자동화를 통해 물류순환속도를 높이고, 재고공간을 효율적으로 활용하게 해 생산성을 높일 수 있는 것이 가장 큰 특징이다.

○ 한 계획된 경로로 작업을 수행하기에 작업장에서의 사고 위험 역시 크게 줄일 수 있다. 이 때문에 반복작업이 빈번하게 이뤄지는 물류창고나 
   24시간 무인가동이 필요한 사업장, 좁은 공간의 작업장 등에서 활용되어 작업 효율을 높일 수 있다.

○ 특히 모든 설비와 장치가 무선통신으로 연결되는 스마트공장을 구축한 제조업체들의 수요에도 부합될 것으로 기대된다.


# 주요기능
 OpenCV 기반의 aruco marker를 사용하여 마커의 2D Pose 정보를 읽어와 터틀봇을 마커의 중앙으로 위치 시키고 Arduino Serial을 사용하여 서보 모터를 컨트롤하여 미션을 수행합니다.
 
 - 구동 영상보기 : https://www.youtube.com/watch?v=5JBPTG4YDPo&t=8s
 - 핵심코드 : track_marker.py 
 
![캡처](https://user-images.githubusercontent.com/84003327/143807282-f6518f57-8946-497a-b856-36948122f147.PNG)

1. 목표에 도착한 로봇이 marker를 찾기위하여 회전을 합니다.
2. marker를 찾은 로봇을 마커의 중앙으로 위치하기 위한 이동을 실시 합니다.
3. marker 앞에서 미션을 수행한 로봇은 종료지점으로 복귀합니다.

연구기록 YouTube -- > https://www.youtube.com/channel/UCLSgng38L1zVYUgOHEe1yOg

![KakaoTalk_20211117_181032963](https://user-images.githubusercontent.com/84003327/143807179-f56f3e64-c1ec-4cea-995a-69c2057d4059.jpg)

![node_graph](https://user-images.githubusercontent.com/84003327/143834486-d9819854-dd50-4132-b44b-c6767171a981.png)


-실행 절차

라즈베리 안에서 실행 하세요

```ros2 run opencv_cam opencv_cam_main --ros-args --params-file .ros/cv_cam_params.yaml```

```ros2 launch turtlebot3_bringup robot.launch.py```

```ros2 run my_rclpy_pkg lift_ctrl```

사용자의 remote PC 에서 실행 하세요

```ros2 run ros2_aruco aruco_node```

camera_info가 늦게 나타나는 현상이 있습니다 천천히 실행하거나 ```ros2 topic list``` 를 실행해서 camera_info 를 확인하세요.

```rqt```

```ros2 run ar_track pub_tb3_pose2d```

```ros2 run ar_track track_marker2 1```
--> 마커 ID 가 [ 1 ] 인것을 찾는다


## 아쉬운점

○ launch 파일로 묶기에는 시간이 촉박했음

○ 프로젝트 마감 시간에 맞추다 보니 slam을 추가 하지 못함

