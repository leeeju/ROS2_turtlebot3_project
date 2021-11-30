import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import degrees, sqrt, pi

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPD = MAX_LIN_SPEED * 0.125
ANG_SPD = MAX_ANG_SPEED * 0.125


class MoveTB3(Node):

    def __init__(self):
        super().__init__('move_tb3')
        qos_profile = QoSProfile(depth=10)
        # define subscriber
        self.sub_ar_pose = self.create_subscription(
            Pose,               # topic type
            'tb3pose2d',        # topic name
            self.get_tb3_pose_, # callback function
            qos_profile)
        #self.timer  = self.create_timer(0.1, self.get_tb3_pose_)
        self.pub_tw = self.create_publisher(Twist, 'cmd_vel', qos_profile)       
        
        self.tb3pose  = self.org = Pose()
        #self.node = Node()
        
         
    def get_tb3_pose_(self, msg):
        self.tb3pose = msg
        
    
    def update_org(self):
        self.org = self.tb3pose
        print(self.org.theta)
        
    def elapsed_dist(self):
        return sqrt(pow((self.tb3pose.x - self.org.x), 2) + pow((self.tb3pose.y - self.org.y), 2))
        
    
    def straight(self, distance):
        rclpy.spin_once(self)
        tw = Twist()
        self.update_org()
        print("straight start from (%s, %s)" %(round(self.org.x, 2), round(self.org.y, 2)))
        
        if distance >= 0:   # +distance
            tw.linear.x =  LIN_SPD
        else:               # -distance
            tw.linear.x = -LIN_SPD
                        
        #self.pub_tw.publish(tw)        
        while rclpy.ok():
            rclpy.spin_once(self)
            self.pub_tw.publish(tw)
            if self.elapsed_dist() < abs(distance):  pass
            else:   break
            #print("%s(m) of %s(m)" %(round(self.elapsed_dist(),2), round(abs(distance),2)))
        
        tw.linear.x = 0.0;    self.pub_tw.publish(tw)
        print("straight stop to    (%s, %s)" %(round(self.tb3pose.x, 2), round(self.tb3pose.y, 2)))
    
        
    def elapsed_angle(self):
        return abs(self.tb3pose.theta - self.org.theta)
        
    def rotate(self, angle):
        rclpy.spin_once(self)
        tw = Twist()
        self.update_org()
        print("rotate start from: %s" %(round(degrees(self.org.theta), 2)))
        
        if angle >= 0.0:	# angle(+): rotate left(ccw)
            tw.angular.z =  ANG_SPD;
        else:			# angle(-): rotate right(cw)
            tw.angular.z = -ANG_SPD;
            
        #self.pub_tw.publish(tw)
        while rclpy.ok():
            rclpy.spin_once(self)
            self.pub_tw.publish(tw)
            if self.elapsed_angle() < abs(angle):    pass
            else:   break
            #print("%s of %s" %(round(degrees(self.elapsed_angle()),2) ,round(degrees(abs(angle)),2)))
            
        tw.angular.z =  0.0;  self.pub_tw.publish(tw)
        print("rotate stop to   : %s" %(round(degrees(self.tb3pose.theta), 2)))

