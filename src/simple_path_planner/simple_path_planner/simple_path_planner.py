#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from nav_msgs.msg import Path
import os


# -----------------------------------------------------------------------------
# 1) Define the “delta” rotation quaternion and 4×4 transform matrix T
# -----------------------------------------------------------------------------
# This quaternion undoes the original orientation (0,0,-0.1088,0.99406)
Q_DELTA = (0.0, 0.0, +0.10880735604155682, 0.9940628547889947)

# 4×4 homogeneous transform: rotation = R(Q_DELTA), translation = [-1.17, +0.06, 0]
TF_MATRIX = np.array([
    [ 0.9763219185, -0.2163227019, 0.0, -1.17],
    [ 0.2163227019,  0.9763219185, 0.0,  0.06],
    [ 0.0,           0.0,          1.0,  0.0],
    [ 0.0,           0.0,          0.0,  1.0]
])


def quaternion_multiply(q1, q2):
    """Multiply two quaternions q1 ⊗ q2. Quaternions are (x,y,z,w)."""
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )


def euler_to_quaternion(yaw: float):
    """Convert a yaw angle (radians) into quaternion (x,y,z,w)."""
    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )


def quaternion_to_euler(q):
    """Extract yaw angle (radians) from quaternion (x,y,z,w)."""
    x,y,z,w = q
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def transform_goal(x: float, y: float, yaw: float):
    """
    Applies TF_MATRIX to (x,y,0,1) and rotates the yaw by Q_DELTA.
    Returns transformed (x', y', yaw').
    """
    # 1) position transform
    v = np.array([x, y, 0.0, 1.0])
    v_t =  v
    x_t, y_t = float(v_t[0]), float(v_t[1])

    # 2) orientation transform
    q_goal = euler_to_quaternion(yaw)
    q_new = quaternion_multiply(Q_DELTA, q_goal)
    yaw_t = quaternion_to_euler(q_new)

    return x_t, y_t, yaw_t


# -----------------------------------------------------------------------------
# 2) ROS 2 Node
# -----------------------------------------------------------------------------
class SimplePathPlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner')

        # — use_sim_time parameter —
        try:
            use_sim = self.declare_parameter('use_sim_time', True).value
        except ParameterAlreadyDeclaredException:
            use_sim = self.get_parameter('use_sim_time').value
        if use_sim:
            self.get_logger().info('Using simulation time (/clock)')

        # — declare other parameters —
        self.declare_parameter('goal_x', 3.2085)
        self.declare_parameter('goal_y', -0.24024)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('verbose', True)

        self.verbose = self.get_parameter('verbose').value

        # placeholders for goal; will set in try_send()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'

        # subscribe to AMCL (fallback to ODOM)
        self.current_pose = None
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_cb,
            10
        )
        # subscribe to Nav2 planner output (/plan)
        self.waypoints = []
        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_cb,
            10
        )

        self.get_logger().info('Subscribed to /amcl_pose (fallback to /odom in 5s)')
        self._fallback_timer = self.create_timer(5.0, self.switch_to_odom)

        # publisher for initial pose
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # --- NEW: publish a startup initial pose immediately ---
        self.publish_initial_pose()

        # NavigateToPose action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for NavigateToPose action server...')
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose server unavailable—exiting')
            rclpy.shutdown()
            return
        self.get_logger().info('NavigateToPose server available.')

        # timer to send goal once we have a pose
        self._sent = False
        self.create_timer(1.0, self.try_send)

    # --- NEW helper method ---
    def publish_initial_pose(self):
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = "map"
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.pose.position.x = -1.17
        init_pose.pose.pose.position.y = 0.06
        init_pose.pose.pose.orientation.z = 0.1088
        init_pose.pose.pose.orientation.w = 0.9940

        # covariance values
        init_pose.pose.covariance = [0.0] * 36
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] = 0.0685

        self.init_pub.publish(init_pose)
        self.get_logger().info("✅ Published startup /initialpose")

    def plan_cb(self, msg: Path):
        self.waypoints.clear()

        for pose_stamped in msg.poses:
            pos = pose_stamped.pose.position
            self.waypoints.append((pos.x, pos.y))

        self.get_logger().info(f'Received path with {len(self.waypoints)} waypoints.')

        # Optional: Save to file
        self.save_waypoints('/tmp/nav2_path.csv')


    def save_waypoints(self, filepath):
        try:
            with open(filepath, 'w') as f:
                f.write("index,x,y\n")
                for i, (x, y) in enumerate(self.waypoints):
                    f.write(f"{i},{x},{y}\n")
            self.get_logger().info(f"Saved waypoints to {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoints: {e}")


    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg
        if self.verbose:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            yaw = quaternion_to_euler((o.x, o.y, o.z, o.w))
            self.get_logger().info(f'[AMCL] x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f}')

    def switch_to_odom(self):
        # if no AMCL pose, switch to /odom after 5s
        if self.current_pose is None:
            self.get_logger().warn('No /amcl_pose—switching to /odom')
            self.destroy_subscription(self.sub)
            self.sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_cb,
                10
            )
            self._fallback_timer.cancel()

    def odom_cb(self, msg: Odometry):
        pwc = PoseWithCovarianceStamped()
        pwc.header = msg.header
        pwc.pose = msg.pose
        self.current_pose = pwc
        if self.verbose:
            p = msg.pose.pose.position
            self.get_logger().info(f'[ODOM] x={p.x:.2f}, y={p.y:.2f}')

    def try_send(self):
        if self._sent:
            return
        self.get_logger().info('Timer tick: checking for pose…')
        if self.current_pose is None:
            return

        # publish initial pose (runtime, from current pose)
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.init_pub.publish(self.current_pose)
        self.get_logger().info('Published initial pose to /initialpose')

        # read original goal parameters
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        gyaw = self.get_parameter('goal_yaw').value

        # transform goal position & yaw
        gx_t, gy_t, gyaw_t = transform_goal(gx, gy, gyaw)
        self.get_logger().info(
            f'Transformed goal → x={gx_t:.2f}, y={gy_t:.2f}, yaw={gyaw_t:.2f}'
        )

        # set up goal PoseStamped
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = gx_t
        self.goal_pose.pose.position.y = gy_t
        self.goal_pose.pose.position.z = 0.0
        qx, qy, qz, qw = euler_to_quaternion(gyaw_t)
        self.goal_pose.pose.orientation.x = qx
        self.goal_pose.pose.orientation.y = qy
        self.goal_pose.pose.orientation.z = qz
        self.goal_pose.pose.orientation.w = qw

        # send navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.goal_pose
        self.get_logger().info('Sending NavigateToPose goal...')
        send_fut = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.on_feedback
        )
        send_fut.add_done_callback(self.on_response)
        self._sent = True

    def on_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Navigation goal rejected.')
            rclpy.shutdown()
            return
        self.get_logger().info('Navigation goal accepted; waiting for result…')
        handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, feedback):
        p = feedback.feedback.current_pose.pose.position
        self.get_logger().info(f'Navigating… x={p.x:.2f}, y={p.y:.2f}')

    def on_result(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().error(f'Navigation failed with error code: {result.error_code}')
        rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    node = SimplePathPlanner()
    try:
        rclpy.spin(node)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
