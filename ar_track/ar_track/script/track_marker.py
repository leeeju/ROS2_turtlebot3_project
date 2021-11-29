import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from turtlesim.msg import Pose as Pose2d
from geometry_msgs.msg import Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sqrt, sin, cos, pi
from tf_transformations import euler_from_quaternion#, quaternion_from_euler
from ar_track.move_tb3 import MoveTB3

TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker
# 마커의 포즈정보를 받아와서 터틀봇을 움직이는 코드 입니다 
# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPEED = MAX_LIN_SPEED * 0.075
ANG_SPEED = MAX_ANG_SPEED * 0.075

R = 1.5708


class TrackMarker(Node):

#1. rotate until found target
#2. if found_target --> slow down angular speed
#3. -0.005 < marker's positon.x < 0.005 --> stop move, get theta, calculate path
#4. move by calculated path
 
    """   
                                                    ////////////| ar_marker |////////////
            y                      z                --------+---------+---------+--------
            ^  x                   ^                        |     R-0/|\R-0    R|
            | /                    |                        |       /0|0\       |
     marker |/                     | robot                  |      /  |  \      |
            +------> z    x <------+                        |     /   |   \     |
                                  /                         |  dist   |  dist   |
                                 /                          |   /     |     \   |
                                y                           |  /      |      \  |
                                                            | /       |       \0|
                                                            |/R-0    R|R    R-0\|
    pose.x = position.z                             (0 < O) x---------+---------x (0 > O)
    pose.y = position.x              [0]roll    (pos.x > O) ^                   ^ (pos.x < O)
    theta  = euler_from_quaternion(q)[1]pitch*              |                   |            
                                     [2]yaw               robot               robot
    """   
    def __init__(self):
        
        super().__init__('track_marker')
        qos_profile = QoSProfile(depth=10)
        
        self.sub_ar_pose  = self.create_subscription(
            ArucoMarkers,           # topic type
            'aruco_markers',        # topic name
            self.get_marker_pose_,  # callback function
            qos_profile)
            
        self.pub_tw = self.create_publisher(Twist, '/cmd_vel', qos_profile) #움직임을 위한
        self.pub_ctrl= self.create_publisher(String, '/lift_ctrl_msg', qos_profile) #목표지점에서 미션을 수행하기 위한
        
        self.pose = Pose()
        self.tw   = Twist()
        self.tb3  = MoveTB3()
        self.lift = String()
        
        self.theta  = 0.0
        self.th_ref = 0.0
        self.z_ref  = 0.0
        
        self.target_found         = False
        self.arrived_ref_position = False
        self.get_ref_value        = False
        self.first_rotate_end     = False
        self.move_front_marker    = False
        self.second_rotate_end    = False
        self.align_finished       = False
        self.approach_marker_end  = False
        self.mission_complete     = False
        
        
    def get_marker_pose_(self, msg):
        """
        orientation x,y,z,w ----+
                                +--4---> +-------------------------+
        input orientaion of marker-----> |                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <------- |                         |
                                +--3---- +-------------------------+
        r,p,y angle <-----------+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """
        
        if self.mission_complete == False:
        
            if len(msg.marker_ids) == 0:    # no marker found
                self.target_found = False   # 마커를 찾지 못하면 0을 계속 내부에서 발행함 (찾기위한 회전이 계속됨)
            
            else: # if len(msg.marker_ids) != 0: # marker found at least 1EA
            
                for i in range(len(msg.marker_ids)):
                
                    if msg.marker_ids[i] == TARGET_ID:  # target marker found  # 마커를 찾았을 때 
                        if self.target_found == False:
                            self.target_found = True
                            print("----- target marker found!")
                        
                        self.pose  = msg.poses[i]
                        self.theta = self.get_theta(self.pose)
                        
                        if self.get_ref_value == False:
                        
                            if   self.pose.position.x < -0.025:
                                self.tw.angular.z =  0.125 * ANG_SPEED
                            elif self.pose.position.x >  0.025:
                                self.tw.angular.z = -0.125 * ANG_SPEED
                            else:
                                self.tw.angular.z =  0.0
                                if self.arrived_ref_position == False:
                                    self.arrived_ref_position = True
                                    print("----- arrived reference position!")
                                    self.th_ref = self.theta
                                    self.z_ref  = self.pose.position.z
                                    self.get_ref_value = True
                                
                            self.pub_tw.publish(self.tw)
                            print("---")
                        
                        else:   # self.get_ref_value == True
                            
                            if self.first_rotate_end == False:                                
                                
                                angle = R - self.th_ref
                                
                                if angle > R:
                                    angle = pi - angle
                                
                                if   self.th_ref > radians( 10):
                                    self.tb3.rotate( angle)
                                elif self.th_ref < radians(-10):
                                    self.tb3.rotate(-angle)
                                else:   pass
                                
                                print("----- 1st rotation finished!")
                                self.first_rotate_end = True
                                
                            else:
                                if self.move_front_marker == False:
                                    dist = abs(self.z_ref * sin(self.th_ref) * 1.1)
                                    self.tb3.straight(dist)
                                    print("----- move to front of marker end!")
                                    self.move_front_marker = True
                                    
                                else:
                                    if self.second_rotate_end == False:
                                        if   self.th_ref >  radians(10):
                                            self.tb3.rotate(-R * 0.975)
                                        elif self.th_ref < -radians(10):
                                            self.tb3.rotate( R)
                                        else:   pass
                                        
                                        print("----- 2nd rotation finished!")
                                        self.second_rotate_end = True                                
                                
                                    else:
                                        if self.align_finished == False:
                                            if   self.pose.position.x < -0.0175:
                                                self.tw.angular.z =  0.125 * ANG_SPEED
                                                self.pub_tw.publish(self.tw)
                                            elif self.pose.position.x >  0.0175:
                                                self.tw.angular.z = -0.125 * ANG_SPEED
                                                self.pub_tw.publish(self.tw)
                                            else:
                                                self.tw.angular.z =  0.0
                                                self.pub_tw.publish(self.tw)
                                                self.align_finished = True
                                                print("----- align to marker finished! (%s)" \
                                                        %(self.pose.position.x))                                     
                                            
                                        else:
                                            if self.approach_marker_end == False:
                                                print(self.pose.position.z)
                                                dist = self.pose.position.z - 0.12
                                                self.tb3.straight(dist)
                                                self.approach_marker_end = True
                                                print("----- approach to marker finished!")
                                                self.lift.data = "lift_up"
                                                self.publish_lift_ctrl(self.lift)
                                                print("----- load palette finished!")
                                                self.mission_complete = True
                    else:
                        self.target_found = False
        else:
            print("----- mission complete!")
            self.destroy_node()
            rclpy.shutdown()
            
        
    def get_theta(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(q)
        theta = euler[1]        
        return theta            
    
        
    def publish_lift_ctrl(self, ctrl_msg):
        msg = String()
        msg = ctrl_msg
        self.pub_ctrl.publish(msg)
        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TrackMarker()
    
    node.tw.angular.z = 0.5 * ANG_SPEED
    
    while rclpy.ok():
        if node.theta != 0.0:   break
        node.pub_tw.publish(node.tw)
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.tw.angular.z = 0.0
    node.pub_tw.publish(node.tw)
    
    try:
        rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

