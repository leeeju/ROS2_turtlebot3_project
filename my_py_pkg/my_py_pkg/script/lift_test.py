import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from my_py_pkg.get_char import GetChar


class LiftTest(Node):
    
    def __init__(self):
        super().__init__('lift_test')
        qos_profile = QoSProfile(depth=10)
        self.pub_ctrl= self.create_publisher(String, '/lift_ctrl_msg', qos_profile)
        
    def publish_lift_ctrl(self, ctrl_msg):
        msg = String()
        msg = ctrl_msg
        self.pub_ctrl.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node     = LiftTest()
    kb_input = GetChar()
    msg      = String()
    ch       = ' '
        
    try:    
        while ch != 'Q':
        
            ch = kb_input.getch()
            
            if   ch == '1':
                msg.data = "lift_up";    print(msg)
                node.publish_lift_ctrl(msg)
            elif ch == '0':
                msg.data = "lift_down";  print(msg)
                node.publish_lift_ctrl(msg)
            else:   pass
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

