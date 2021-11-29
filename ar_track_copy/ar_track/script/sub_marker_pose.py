import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
#from turtlesim.msg import Pose
from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers

TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker

class MarkerPose(Node):
    
    def __init__(self):
        super().__init__('sub_marker_pose')
        qos_profile = QoSProfile(depth=10)
        
        # define subscriber
        self.sub_ar_pose = self.create_subscription(
            ArucoMarkers,           # topic type
            'aruco_markers',        # topic name
            self.get_marker_pose,   # callback function
            qos_profile)
        
        self.pose = Pose()
        
    def get_marker_pose(self, msg):
        if len(msg.marker_ids) != 0:
            for i in range(len(msg.marker_ids)):
                if msg.marker_ids[i] == TARGET_ID:
                    self.pose = msg.poses[i]
                    self.print_marker_pose()
    
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

