import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from my_py_pkg.get_char import GetChar

msg_how2 = '''
---------------------------------------
              (forward)
                 'w'

  (ccw)'a'      's'       'd'(cw)
              (backward)
---------------------------------------
type 'Ctrl-C' or 'Q' to quit program...
---------------------------------------
'''

class RemoteTurtle(Node):

    def __init__(self):
        super().__init__('remote_turtle')
        qos_profile = QoSProfile(depth=10)
        self.tw_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)
        
    def publish_tw_msg(self, tw_msg):
        msg2 = Twist()
        #msg = tw_msg
        msg2 = Joy()

        self.tw_pub.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node        = RemoteTurtle()
    kb_input    = GetChar()
    tw          = Twist()
    ch          = ' '
    count_keyin =  0
    count_reset = 15

    try:
        print(msg_how2)

        while ch != 'Q':

            ch = kb_input.getch()

            if   ch == 'w':
                tw.linear.x  =  2.0;    tw.angular.z =   .0
                print(": forward"   );  count_keyin = count_keyin + 1
            elif ch == 's':
                tw.linear.x  = -2.0;    tw.angular.z =   .0
                print(": backward"  );  count_keyin = count_keyin + 1
            elif ch == 'a':
                tw.linear.x  =   .0;    tw.angular.z =  2.0
                print(": turn left" );  count_keyin = count_keyin + 1
            elif ch == 'd':
                tw.linear.x  =   .0;    tw.angular.z = -2.0
                print(": turn right");  count_keyin = count_keyin + 1
            elif ch == 'Q' or ch == 'q':
                break
            else:
                pass

            node.publish_tw_msg(tw)

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

