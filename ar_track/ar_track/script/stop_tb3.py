import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

class StopTB3(Node): # 비상정지 노드
    
    def __init__(self):
        super().__init__('remote_tb3_lift')
        qos_profile   = QoSProfile(depth=10)
        self.pub_tw   = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        # create_timer 1초마가 주기적으로 값을 생성해서 인위적으로 마커id 1을 찾았다는 착각을 주게 해서 로봇을 강제로 멈춤
        self.timer    = self.create_timer(1, self.stop_tb3_)
        
    def stop_tb3_(self):
        tw = Twist()
        self.pub_tw.publish(tw)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = StopTB3()
        
    try:
        print("### publish topic '/cmd_vel' to stop turtlebot3 every second!") 
        rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

