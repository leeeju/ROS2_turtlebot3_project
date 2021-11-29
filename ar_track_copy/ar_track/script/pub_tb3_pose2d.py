import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import radians, degrees, pi
from tf_transformations import euler_from_quaternion#, quaternion_from_euler

class TB3Pose2D(Node):

    def __init__(self):    
        super().__init__('pub_tb3_pose2d')
        qos_profile = QoSProfile(depth=10)
        
        self.sub_ar_pose = self.create_subscription(
            Odometry,       # topic type
            'odom',         # topic name
            self.get_odom_, # callback function
            qos_profile)
        self.pub_pose2d = self.create_publisher(Pose, 'tb3pose2d', qos_profile)
                
        self.prv_theta = 0.0
        self.theta_sum = 0.0        
        
    def get_odom_(self, msg):
        
        pos_x, pos_y, theta = self.get_pose(msg)
        
        pose2d       = Pose()   # turtlesim.msg.Pose()
        pose2d.x     = pos_x
        pose2d.y     = pos_y
        # pose2d.theta
        pose2d.linear_velocity  = msg.twist.twist.linear.x
        pose2d.angular_velocity = msg.twist.twist.linear.x
        
        if   (theta - self.prv_theta) >  radians(270): # 5.0: #  5.0(rad) =  286.479(deg)
            d_theta = (theta - self.prv_theta) - 2 * pi            
        elif (theta - self.prv_theta) < -radians(270): #-5.0: # -5.0(rad) = -286.479(deg)
            d_theta = (theta - self.prv_theta) + 2 * pi
        else:
            d_theta = (theta - self.prv_theta)

        self.theta_sum = self.theta_sum + d_theta
        self.prv_theta = theta
               
        pose2d.theta = self.theta_sum
        
        self.pub_pose2d.publish(pose2d)
        self.print_pose(pose2d)
        
        
    def get_pose(self, msg):
        
        q = ( msg.pose.pose.orientation.x,
              msg.pose.pose.orientation.y,
              msg.pose.pose.orientation.z, 
              msg.pose.pose.orientation.w )
                                            # quart[0] = roll
        euler = euler_from_quaternion(q)    # quart[1] = pitch
        theta = euler[2]                    # quart[2] = yaw <----
        
    	# make theta within from 0 to 360 degree
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        return pos_x, pos_y, theta
        
        
    def print_pose(self, msg):
        print("x = %s, y = %s, th = %s = %s" %(round(msg.x,2),round(msg.y,2),round(msg.theta,3),round(degrees(msg.theta),2)))


def main(args=None):
    rclpy.init(args=args)
    node = TB3Pose2D()
    
    try:
        rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()


