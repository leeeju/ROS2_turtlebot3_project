
# ROS2_turtlebot3_fork lift_project

## 프로제트 기간 : 2021. 11. 01 ~ 2021. 11. 29

### 환경구성 : Ubuntu20.04, ROS2 foxy, Turtlebot3

### 사용기술 : Open_CV, aruco marker, rviz2, python, Arduino Serial Bus

#### 팀원 : 이주현
  - 지도교수 : ground_Zero
  
  연구기록 YouTube -- > https://www.youtube.com/channel/UCLSgng38L1zVYUgOHEe1yOg

## Ⅰ배경 

○ 무인지게차는 자동화를 통해 물류순환속도를 높이고, 재고공간을 효율적으로 활용하게 해 생산성을 높일 수 있는 것이 가장 큰 특징이다.

○ 한 계획된 경로로 작업을 수행하기에 작업장에서의 사고 위험 역시 크게 줄일 수 있다. 이 때문에 반복작업이 빈번하게 이뤄지는 물류창고나 
   24시간 무인가동이 필요한 사업장, 좁은 공간의 작업장 등에서 활용되어 작업 효율을 높일 수 있다.

○ 특히 모든 설비와 장치가 무선통신으로 연결되는 스마트공장을 구축한 제조업체들의 수요에도 부합될 것으로 기대된다.


## Ⅱ주요기능
 OpenCV 기반의 aruco marker를 사용하여 마커의 2D Pose 정보를 읽어와 터틀봇을 마커의 중앙으로 위치 시키고 Arduino Serial을 사용하여 서보 모터를 컨트롤하여 미션을 수행합니다.
 
![캡처](https://user-images.githubusercontent.com/84003327/143807282-f6518f57-8946-497a-b856-36948122f147.PNG)

1. 목표에 도착한 로봇이 marker를 찾기위하여 회전을 합니다.
2. marker를 찾은 로봇을 마커의 중앙으로 위치하기 위한 이동을 실시 합니다.
3. marker 앞에서 미션을 수행한 로봇은 종료지점으로 복귀합니다.

## Ⅲ. 시스템 구성 및 아키텍처


![ddddddd](https://user-images.githubusercontent.com/84003327/188041167-a2557cd4-4e5e-4c85-ae60-fd3735691e50.png)

 로봇이 제자리에서 회전을 하면서 open_CV를 통하여 marker의 pose정보를 습득한다, 마커에서 발행되는 pose정보를 바탕으로 로봇이 이동 거리와 각도를 계산하여 위치정보를 받아오고 정보
 를 가지고 마커의 정중앙으로 이동을 시작한다, 정중앙으로 이동을 완료 후 지정된 마커 앞 OOcm까지 이동을 실시 하며 이동을 완료 한 후 lift를 들어올려 미션을 수행한다. 

## Ⅳ 기대효과 및 활용분야

  스마트 지게차는 물류창고의 신속한 물류 정리 및 근무 인력의 피로도 감소, 체계적인 창고 재고관리 효과를 가져올 수 있으며 택배 물류창고, 대형마트 물류창고와 같은 대규모 물류보관 
  창고에서 활용이 가능하며 소형화 양산 가능 시 할인마트 에서도 활용 가능한 제원이 될 것이다.


## Ⅵ. 기타첨부
 로봇은 목표를 찾아 가기 위해 자기 자신의 위치를 특정 하기을 해야 한다 그 방법으로는 대표적으로 slam을 사용하지만 이번엔 카메라에서 발행하는 2D pose를 echo 함으로 토픽을 불러온다, 
 그 토픽을 바탕으로 마커가 발행하는 x,y,z의 대한 토픽과 비교를 하여 자신의 위치에서 마커를 찾아가기 위한 공식을 만들게 된다, 아래 사진과 같이 마커의 위치와 카메라(로봇)의 위치를 비
 교하는 공식을 만들어 소스코드 에 적용시키면 서로의 pose정보를 바탕으로 로봇이 마커의 중앙으로 이동 할 수 있는 알고리즘이 완성되게 된다, 해당 코드는 track_marker.py를 보면 알 수 
 있으며 어려울 수 있는 공식중 orientation 좌표계를 quaternion으로 변환 사용하였으며 이동간의 속도 제어를 통하여 마커를 찾아가기 위한 움직임을 정교하게 함.


![sss](https://user-images.githubusercontent.com/84003327/188041500-5e105454-3cfb-49e7-ae69-81b2277e2efc.png)

좌표제어를 위한 변환에는 아래 그림과 같이 변환하여 사용하였으며


![123](https://user-images.githubusercontent.com/84003327/188041532-f2be4d9e-ce59-45fa-9240-03af839dbc80.png)

 초기 마커가 발행하는 pose와 카메라(로봇)의 pose는 서로의 축 정보가 달라서 위와 같이 pose,z의 정보를 position.z에 넣어서 사용하는 방식을 채용함으로 코드
 를 간단하게 만듦, 그 이유는 초반 마커의 정면이 x축이라 예상 하였지만 확인 결과 반대의 상황이었기 때문이다, 따라서 많은 실험을 통하여 검증하고 알아낸 후 코드에 적용하는 방식을 사용
 하였다.

![aaa](https://user-images.githubusercontent.com/84003327/188041764-34d7e078-6237-42a5-bdfd-2875603934d5.png)

 위 실험에 대해서는 orientation.x,y,z,w에서의 변화는 예측했지만, Quaternion 에 대한 개념이 없는 상태에서는 구체적인 예측은 할 수 없었다. 실험 결과는 일직선 위에서 마주보는 위치에
 서 Orientation.z 가 0에 가까운 값을 유지하다, 로봇이 z축을 회전축으로 +회전(ccw)할 경우 -값으로, -회전(cw)할 경우 +값으로 변하는 것을 관측할 수 있었다.


![화면 캡처 2022-09-02 105153](https://user-images.githubusercontent.com/84003327/188041931-367809ce-191c-4daf-87e3-ff0642e8f0a4.png)

 lift의 경우 Arduino Serial의 경우 pwm 모터 제어를 통하여 구현 하였으며 nano 보드를 납땜하여 구현함, 구현 코드의 경우 Lift.ino를 참조 하면 되고 ROS를 통한 Arduino Serial 통신제
 어를 위한 통신 포트 개방를 개방 하였으며 '/lift_ctrl_msg'를 통하여 아두이노를 제어함으로 목표지점인 마커 앞 5cm에서 lift를 들어 올려라 와 같은 구현이 가능하게 만듦


 - 구동 영상보기 : https://www.youtube.com/watch?v=5JBPTG4YDPo&t=8s
 - 핵심코드 : track_marker.py

![KakaoTalk_20211117_181032963](https://user-images.githubusercontent.com/84003327/143807179-f56f3e64-c1ec-4cea-995a-69c2057d4059.jpg)
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

```ros2 run ar_track track_marker 1```
--> 마커 ID 가 [ 1 ] 인것을 찾는다


## 아쉬운점

○ launch 파일로 묶기에는 시간이 촉박했음

○ 프로젝트 마감 시간에 맞추다 보니 slam을 추가 하지 못함

