import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class ObstacleAvoidController(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_controller')

        # Publisher and Subscribers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # State variables
        self.min_distance = float('inf')
        self.yaw = 0.0
        self.turning = False
        self.yaw_start = None

    def scan_callback(self, msg):
        # Focus on a narrow forward-facing laser scan window
        mid = 0 # len(msg.ranges) // 2 180
        self.get_logger().info(f"mid: {mid}")
        self.get_logger().info(f"total: {len(msg.ranges)}")

        window = 5
        front_ranges_1 = msg.ranges[0 : 5]
        front_ranges_2 = msg.ranges[355 : 360]
        front_ranges = front_ranges_1 + front_ranges_2
        self.get_logger().info(f"total: {front_ranges}")
        self.min_distance = min(front_ranges)

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

    def control_loop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'Min distance ahead: {self.min_distance:.2f} m')

        if self.turning:
            angle_diff = abs(self.normalize_angle(self.yaw - self.yaw_start))
            self.get_logger().info(f'Turning: {math.degrees(angle_diff):.2f}°')

            if angle_diff >= math.pi / 2:
                self.get_logger().info("Completed 90° turn")
                self.turning = False
            else:
                msg.twist.angular.z = 0.5  # Turning in place

        else:
            if self.min_distance < 0.5:
                self.get_logger().info("Obstacle too close! Starting turn.")
                self.turning = True
                self.yaw_start = self.yaw
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.5
            elif self.min_distance < 1.0:
                self.get_logger().info("Obstacle nearby: slowing down")
                msg.twist.linear.x = 0.1
                msg.twist.angular.z = 0.0
            else:
                self.get_logger().info("Moving normally ")
                msg.twist.linear.x = 0.5 # Cruise speed
                msg.twist.angular.z = 0.0
                self.get_logger().info(f'x_velocity: {msg.twist.linear.x:.2f} m')

        self.cmd_pub.publish(msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
