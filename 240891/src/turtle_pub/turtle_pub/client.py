import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # Service client for teleportation
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')

        # Publisher for circular motion
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def teleport(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)

        future = self.teleport_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Turtle teleported to x={x}, y={y}, theta={theta}")
        else:
            self.get_logger().error("Teleportation failed")

    def move_in_circle(self, r, v):
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(v) / float(r)

        self.get_logger().info(f"Starting circular motion: radius={r}, velocity={v}, angular={twist.angular.z}")

        while rclpy.ok():  # Keep publishing until shutdown
            self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    # Step 1: Teleport the turtle
    x = input("Enter x : ")
    y = input("Enter y : ")
    theta = input("Enter theta : ")
    node.teleport(x, y, theta)

    # Step 2: Move in a circle
    r = float(input("Enter radius: "))
    v = float(input("Enter velocity: "))
    node.move_in_circle(r, v)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
