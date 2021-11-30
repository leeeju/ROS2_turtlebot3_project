import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ar_track.move_tb3 import MoveTB3
from math import radians, degrees, sqrt, atan2
#움직임을 테스츠 할때 사용된 코드 입니다 

def main(args=None):
    rclpy.init(args=args)
    node = MoveTB3()
    
    try:
        a = radians(float(input("input rotation (deg): ")))
        #d = float(input("input distance (m)  : "))
        
        #angle = atan2(y, x)
        #dist  = sqrt(pow(x, 2) + pow(y, 2))
        
        #print("rotate %s(deg), and than straight %s(m)" %(degrees(angle), dist))
        
        node.rotate(a)
        #node.straight(d)
        #node.rotate(-angle)
        
        rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()
    
