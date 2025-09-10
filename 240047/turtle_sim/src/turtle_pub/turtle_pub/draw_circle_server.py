import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtle_pub.srv import DrawCircle
import math


class DrawCircleServer(Node):
    def __init__(self):
        super().__init__('draw_circle_server')

        # Create service
        self.srv = self.create_service(DrawCircle, 'draw_circle', self.draw_circle_callback)

        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Client for teleportation
        self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')

        # Wait until the teleport service is available
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for /turtle1/teleport_absolute service...')

        self.get_logger().info('✅ DrawCircle service ready!')

    def draw_circle_callback(self, request, response):
        self.get_logger().info(f'Received: x={request.x}, y={request.y}, r={request.radius}')

        # ---- Step 1: Teleport the turtle ----
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = float(request.x)
        teleport_req.y = float(request.y)
        teleport_req.theta = 0.0  # start facing right

        future = self.teleport_client.call_async(teleport_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('❌ Teleport service call failed!')
            response.success = False
            return response

        self.get_logger().info('🐢 Teleported turtle successfully!')

        # ---- Step 2: Draw a circle ----
        twist = Twist()

        # Move forward briefly (to offset from teleport point)
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < request.radius:
            self.publisher.publish(twist)

        # Rotate with forward motion (circle)
        twist.linear.x = 1.0
        twist.angular.z = 1.0
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        duration = 2 * math.pi * request.radius
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.publisher.publish(twist)

        # Stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

        # Response
        response.success = True
        self.get_logger().info('✅ Finished drawing circle!')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
