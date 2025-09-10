import rclpy
from rclpy.node import Node
from turtle_pub.srv import DrawCircle  # Custom service


class DrawCircleClient(Node):
    def __init__(self):
        super().__init__('draw_circle_client')
        # Create client for DrawCircle service
        self.client = self.create_client(DrawCircle, 'draw_circle')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for service...')

    def send_request(self, x, y, radius):
        # Prepare request with x, y, radius
        req = DrawCircle.Request()
        req.x = x
        req.y = y
        req.radius = radius

        # Call service asynchronously
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    client = DrawCircleClient()

    # Example request: teleport to (2, 3) and draw a circle of radius 1.0
    resp = client.send_request(2.0, 3.0, 1.0)

    if resp.success:
        client.get_logger().info('✅ Circle drawn successfully!')
    else:
        client.get_logger().info('❌ Failed to draw circle')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
