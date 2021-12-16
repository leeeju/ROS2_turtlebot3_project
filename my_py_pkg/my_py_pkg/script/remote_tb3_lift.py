import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from my_py_pkg.get_char import GetChar

MAX_LIN_SPD = 0.22
MAX_ANG_SPD = 2.84

LIN_STEP    = 0.01
ANG_STEP    = 0.10

msg_how2 = '''
-----------------------------------------------
             increase
             linear_x                lift up
               +---+                  +---+
               | w |                  | i |
 increase  +---+---+---+ decrease     +---+
 angular_z | a | s | d | agular_x     | k |
           +---+---+---+              +---+
              decrease               lift down
              linear_x 
-----------------------------------------------
 Type 'Ctrl-C' or 'Q' to quit program...
-----------------------------------------------
'''

class RemoteTb3Lift(Node):
    
    def __init__(self):
        super().__init__('remote_tb3_lift')
        qos_profile   = QoSProfile(depth=10)
        self.pub_tw   = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.pub_ctrl = self.create_publisher(String, '/lift_ctrl_msg', qos_profile)
        
    def publish_tw_msg(self, tw_msg):
        msg = Twist()
        msg = tw_msg
        self.pub_tw.publish(msg)
        
    def publish_lift_ctrl(self, ctrl_msg):
        msg = String()
        msg = ctrl_msg
        self.pub_ctrl.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node        = RemoteTb3Lift()
    kb_input    = GetChar()
    tw          = Twist()
    msg         = String()
    ch          = ' '
    count_keyin =  0
    count_reset = 15
        
    try:
        print(msg_how2)
    
        while ch != 'Q':
        
            ch = kb_input.getch()
            
            if ch == 'i' or ch == 'k':
            
                if   ch == 'i':
                    msg.data = "lift_up";    print(msg)
                    node.publish_lift_ctrl(msg)                    
                else:
                    msg.data = "lift_down";  print(msg)
                    node.publish_lift_ctrl(msg)
                    
                count_keyin = count_keyin + 1
                
            else:
            
                if   ch == 'w':
                    if   tw.linear.x + LIN_STEP <=  MAX_LIN_SPD:
                        tw.linear.x  =  tw.linear.x + LIN_STEP
                    else:
                        tw.linear.x  =  MAX_LIN_SPD
                    count_keyin = count_keyin + 1
                    
                elif ch == 's':
                    if   tw.linear.x - LIN_STEP >= -MAX_LIN_SPD:
                        tw.linear.x  =  tw.linear.x - LIN_STEP
                    else:
                        tw.linear.x  = -MAX_LIN_SPD
                    count_keyin = count_keyin + 1
                    
                elif ch == 'a':
                    if   tw.angular.z + ANG_STEP <=  MAX_ANG_SPD:
                        tw.angular.z = tw.angular.z + ANG_STEP
                    else:
                        tw.angular.z = MAX_ANG_SPD
                    count_keyin = count_keyin + 1
                    
                elif ch == 'd':
                    if   tw.angular.z - ANG_STEP >= -MAX_ANG_SPD:
                        tw.angular.z = tw.angular.z - ANG_STEP
                    else:
                        tw.angular.z = -MAX_ANG_SPD
                    count_keyin = count_keyin + 1
                    
                elif ch == ' ':
                    tw.linear.x = tw.angular.z = 0.0
                    
                elif ch == 'Q' or ch == 'q':    break
                    
                else:   pass
                
                node.publish_tw_msg(tw)
                print("linear speed = %s(m/s)\tangular speed = %s(rad/s)" %(tw.linear.x, tw.angular.z))
            
            count_keyin = count_keyin % count_reset
            
            if count_keyin == 0:
                print(msg_how2) 
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

