import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from turtlesim.srv import TeleportAbsolute
import math
import time

class CircleSubscriberNode(Node):
    def __init__(self):
        super().__init__('circle_subscriber_node')
        self.get_logger().info("Circle drawing subscriber is now active.")

        # Subscriber to get the circle parameters from cmd line when the user sends it
        self.subscription = self.create_subscription(
            Vector3,
            'draw_circle_params',
            self.draw_circle_callback,
            10)

        # To control the turtle
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # teleport the turtle
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting again...')

    def draw_circle_callback(self, msg):
        center_x = msg.x
        center_y = msg.y
        radius = msg.z

        self.get_logger().info(f"Received request to draw a circle at ({center_x:.2f}, {center_y:.2f}) with radius {radius:.2f}")

        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = float(center_x)
        teleport_request.y = float(center_y)
        teleport_request.theta = float(math.pi / 2.0)
        self.teleport_client.call_async(teleport_request)

        time.sleep(1)

        twist_msg = Twist()
        twist_msg.linear.y = -radius
        start_time = self.get_clock().now()
        duration = 1
        while rclpy.ok() and (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(0.05)

        twist_msg = Twist()
        twist_msg.linear.x = radius
        twist_msg.angular.z = 1.0    # (1 rad/s)

        # 2*pi / omega
        duration = 2 * math.pi / twist_msg.angular.z
        start_time = self.get_clock().now()

        # Publish velocity commands
        while rclpy.ok() and (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(0.05)

        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.get_logger().info("Circle drawing complete.")


def main(args=None):
    rclpy.init(args=args)
    node = CircleSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
