import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ros2_aruco_interfaces.msg import ArucoMarkers

class PubAruco(Node):
    """
    header:
      stamp:
        sec: 1638101289
        nanosec: 750166293
      frame_id: camera
    marker_ids:
    - 1
    poses:
    - position:
        x: -0.0013986673081593328
        y: 0.002566983638430342
        z: 0.41009026866048165
      orientation:
        x: 0.9937564141372246
        y: -0.007127948657676227
        z: -0.11052362835449114
        w: -0.013487375000974943
    """
    def __init__(self):
        super().__init__('pub_aruco_once')
        qos_profile  = QoSProfile(depth=10)
        self.pub_ar  = self.create_publisher(ArucoMarkers, '/aruco_markers', qos_profile)
        self.timer   = self.create_timer(1, self.pub_aruco_)
        self.markers = ArucoMarkers()
        self.count   = 0
        self.markers[0].marker_ids = 0
        self.markers[0].pose.position.x = 0.0
        self.markers[0].pose.position.y = 0.0
        self.markers[0].pose.position.z = 0.0
        self.markers[0].pose.orientation.x = 0.0
        self.markers[0].pose.orientation.y = 0.0
        self.markers[0].pose.orientation.z = 0.0
        self.markers[0].pose.orientation.w = 0.0
        
    def pub_aruco_(self):
        
        self.pub_ar.publish(self.markers[0])
        
        self.count = self.count + 1
        
        if self.count == 3:
            node.destroy_node()
            rclpy.shutdown()
            
        
def main(args=None):
    rclpy.init(args=args)
    node = PubAruco()
        
    try:
        rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

