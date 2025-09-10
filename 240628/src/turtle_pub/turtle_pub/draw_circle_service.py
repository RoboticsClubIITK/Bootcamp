import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from turtle_pub.srv import DrawCircle
import math
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info('Turtle Controller Node started.')

        # Create a publisher for the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose = Pose()
        
        # Subscribe to the pose topic to get the turtle's current position
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Create the service server for drawing the circle
        self.srv = self.create_service(DrawCircle, 'draw_circle', self.draw_circle_callback)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

    def pose_callback(self, msg):
        self.pose = msg

    def draw_circle_callback(self, request, response):
        self.get_logger().info(f'Received request to draw a circle at x={request.x}, y={request.y} with radius={request.radius}')

        # Phase 1: Teleport to the specified coordinates
        self.teleport_turtle(request.x, request.y, 0.0)

        # Phase 2: Move along the positive x-axis
        self.move_straight(request.radius)
        
        # Phase 3: Move in a circular path
        self.move_in_circle(request.radius)

        response.success = True
        response.message = "Cool success! The TurtleBot completed its movements."
        self.get_logger().info('TurtleBot completed movements.')
        return response

    def teleport_turtle(self, x, y, theta):
        self.get_logger().info(f'Teleporting to x={x}, y={y}, theta={theta}')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting...')

        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.teleport_client.call_async(req)
        time.sleep(1) # Give time for the teleport to complete

    def move_straight(self, distance):
        self.get_logger().info(f'Moving straight for distance: {distance}')
        
        start_x = self.pose.x
        start_y = self.pose.y
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 1.0  # Move forward at a speed of 1.0 m/s
        
        while self.pose.x < (start_x + distance):
            self.publisher_.publish(cmd_vel_msg)
            rclpy.spin_once(self)
        
        # Stop the turtle
        cmd_vel_msg.linear.x = 0.0
        self.publisher_.publish(cmd_vel_msg)

    def move_in_circle(self, radius):
        self.get_logger().info(f'Moving in a circular path with radius: {radius}')
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 1.0
        cmd_vel_msg.angular.z = 1.0 / radius
        
        # Move for one full circle. Time = 2*pi*r / v = 2*pi / w
        duration = (2 * math.pi) / cmd_vel_msg.angular.z
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.publisher_.publish(cmd_vel_msg)
            rclpy.spin_once(self)

        # Stop the turtle
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.publisher_.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()