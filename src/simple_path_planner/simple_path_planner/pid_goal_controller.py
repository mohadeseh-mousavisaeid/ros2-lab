# Refactored version of the previous SimplePathPlanner into a PID-based velocity controller with full PID (P, I, D).

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class PIDGoalController(Node):
    def __init__(self):
        super().__init__('pid_goal_controller')

        # PID Gains
        self.kp_rho = 0.8       # linear distance gain
        self.ki_rho = 0.0       # integral gain (linear)
        self.kd_rho = 0.1       # derivative gain (linear)

        self.kp_alpha = 1.5     # angular error gain
        self.ki_alpha = 0.0     # integral gain (angular)
        self.kd_alpha = 0.1     # derivative gain (angular)

        # State
        self.current_pose = None
        self.goal_pose = None

        # PID memory
        self.prev_rho = 0.0
        self.sum_rho = 0.0

        self.prev_alpha = 0.0
        self.sum_alpha = 0.0

        self.dt = 0.1  # timer interval

        # Subscriptions and Publishers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info('New goal received')

    def control_loop(self):
        if self.current_pose is None or self.goal_pose is None:
            return

        # Current position and orientation
        pos = self.current_pose.position
        q = self.current_pose.orientation

        robot_theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y**2 + q.z**2))

        # Goal position
        goal_pos = self.goal_pose.position

        dx = goal_pos.x - pos.x
        dy = goal_pos.y - pos.y
        rho = math.hypot(dx, dy)

        desired_theta = math.atan2(dy, dx)
        alpha = normalize_angle(desired_theta - robot_theta)

        # PID calculations
        self.sum_rho += rho * self.dt
        d_rho = (rho - self.prev_rho) / self.dt

        self.sum_alpha += alpha * self.dt
        d_alpha = (alpha - self.prev_alpha) / self.dt

        self.prev_rho = rho
        self.prev_alpha = alpha

        # Apply PID control law
        cmd = Twist()
        cmd.linear.x = (
            self.kp_rho * rho +
            self.ki_rho * self.sum_rho +
            self.kd_rho * d_rho
        )

        cmd.angular.z = (
            self.kp_alpha * alpha +
            self.ki_alpha * self.sum_alpha +
            self.kd_alpha * d_alpha
        )

        # Clip for safety
        cmd.linear.x = min(cmd.linear.x, 0.5)
        cmd.angular.z = max(min(cmd.angular.z, 1.5), -1.5)

        self.cmd_pub.publish(cmd)

        if rho < 0.1:
            self.get_logger().info('Goal reached')
            self.goal_pose = None
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.sum_rho = 0.0
            self.sum_alpha = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = PIDGoalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
# "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"

# ros2 run simple_path_planner pid_goal_controller
