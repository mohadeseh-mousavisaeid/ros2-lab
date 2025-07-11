import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry, Path
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

import math
import yaml
import cv2
import numpy as np
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time

class StandaloneNavigator(Node):
    def __init__(self):
        super().__init__('standalone_navigator')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Planner action client
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Control loop timer
        self.create_timer(0.1, self.control_loop)

        # State
        self.current_pose = None
        self.yaw = 0.0
        self.global_plan = []
        self.min_distance_ahead = float('inf')

        # Start-up tasks
        self.publish_map('/home/projectlab3_ss25/our_ws/my_map.yaml')  # Provide full path or assume CWD
        self.set_goal_pose(2.0, 0.0)  # Arbitrary goal on the map

    def publish_map(self, yaml_file):
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        img = cv2.imread(data['image'], cv2.IMREAD_GRAYSCALE)
        occupancy = 100 - (img / 255.0 * 100).astype(np.int8)
        occupancy[img == 205] = -1  # Unknown

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info = MapMetaData()
        msg.info.width = img.shape[1]
        msg.info.height = img.shape[0]
        msg.info.resolution = data['resolution']
        msg.info.origin.position.x = data['origin'][0]
        msg.info.origin.position.y = data['origin'][1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy.flatten().tolist()

        # Publish it a few times so everyone receives it
        for _ in range(10):
            self.map_pub.publish(msg)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        self.get_logger().info("✅ Map published.")

    def set_goal_pose(self, x, y):
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.orientation.w = 1.0
        self.send_path_request()

    def send_path_request(self):
        if not self.planner_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Planner server not available!")
            return

        goal = ComputePathToPose.Goal()
        goal.goal = self.goal_pose
        goal.start = PoseStamped()
        goal.start.header.frame_id = 'map'
        goal.start.pose.position.x = self.current_pose.x if self.current_pose else 0.0
        goal.start.pose.position.y = self.current_pose.y if self.current_pose else 0.0
        goal.start.pose.orientation.w = 1.0

        self.planner_client.send_goal_async(goal).add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        result = future.result().result
        self.global_plan = result.path.poses
        self.get_logger().info(f"🧭 Received path with {len(self.global_plan)} poses.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def scan_callback(self, msg):
        front = msg.ranges[0:5] + msg.ranges[-5:]
        valid = [r for r in front if not math.isinf(r) and not math.isnan(r)]
        self.min_distance_ahead = min(valid) if valid else float('inf')

    def control_loop(self):
        if not self.current_pose or not self.global_plan:
            return

        cmd = Twist()

        if self.min_distance_ahead < 0.5:
            self.get_logger().warn("🛑 Obstacle ahead!")
            cmd.angular.z = 0.5
            self.cmd_pub.publish(cmd)
            return

        # Follow plan
        goal = self.global_plan[min(5, len(self.global_plan)-1)].pose.position
        dx = goal.x - self.current_pose.x
        dy = goal.y - self.current_pose.y
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.yaw)

        if abs(angle_diff) > 0.2:
            cmd.angular.z = 0.5 * math.copysign(1.0, angle_diff)
        else:
            cmd.linear.x = 0.3

        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = StandaloneNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
