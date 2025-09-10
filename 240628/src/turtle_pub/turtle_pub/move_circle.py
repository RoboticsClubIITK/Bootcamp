import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveCircle(Node):
    def __init__(self):
        super().__init__('move_circle')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_cmd)

    def publish_cmd(self):
        vel = Twist()
        vel.linear.x = 0.2
        vel.angular.z = 0.2
        self.pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = MoveCircle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
