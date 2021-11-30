import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from turtlesim.msg import Pose as Pose2d
from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sin, cos, pi
from tf_transformations import euler_from_quaternion#, quaternion_from_euler

TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker  
# 실행 할때 찾고자 하는 마커의 ID를 아규먼트로 넣어 줍니다, 아이디가 다른 마커는 찾지 못합니다 
class MarkerPose(Node):
    
    def __init__(self):
        super().__init__('pub_marker_pose2d')
        qos_profile = QoSProfile(depth=10)
        
        # define subscriber
        self.sub_ar_pose = self.create_subscription(
            ArucoMarkers,           # topic type
            'aruco_markers',        # topic name
            self.get_marker_pose_,  # callback function
            qos_profile)
        self.pub_pose2d = self.create_publisher(Pose2d, 'marker_pose2d', qos_profile) #마커의 포즈 정보를 받아 옵니다 
        self.pose   = Pose()
        
    def get_marker_pose_(self, msg):

        pose   = Pose()
        pose2d = Pose2d()
        
        
        if len(msg.marker_ids) != 0:
            for i in range(len(msg.marker_ids)):
                if msg.marker_ids[i] == TARGET_ID:
                    pose  = msg.poses[i]
                    theta = self.get_theta(pose)
                    
                    self.pose = msg.poses[i]
                    print(degrees(theta))
                    #self.print_marker_pose()
        
    def get_theta(self, msg):#마커의 orientation x,y,z,w 의 값을 가져옴
        
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        
        euler = euler_from_quaternion(q)
         #오일러 방정식과 쿼터니언 방정식을 활용한 로봇의 위치와 마커의 위치를 찾기 위함
        theta = euler[1]
        '''
        # make theta from 0 to 360(deg)
        if theta < 0:
            theta = theta + radians(360)
        if theta > 2 * pi:
            theta = theta - radians(360)
        '''
        return theta
        
        
    
    def print_marker_pose(self):
        print("position_x = %s" %(self.pose.position.x))
        print("position_y = %s" %(self.pose.position.y))
        print("position_z = %s" %(self.pose.position.z))
        print("orientation_x = %s" %(self.pose.orientation.x))
        print("orientation_y = %s" %(self.pose.orientation.y))
        print("orientation_z = %s" %(self.pose.orientation.z))
        print("orientation_w = %s" %(self.pose.orientation.w))
        print("")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = MarkerPose()
    
    try:
        rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

