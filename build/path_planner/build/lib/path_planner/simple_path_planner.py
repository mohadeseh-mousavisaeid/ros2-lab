#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

class SimplePathPlanner(Node):

    def __init__(self):
        super().__init__('simple_path_planner')

        # 1) Subscriber to current pose (from AMCL)
        self.current_pose: PoseWithCovarianceStamped = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_cb,
            10
        )

        # 2) Publisher to /initialpose (only needed if you want to reset AMCL)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 3) Action client for ComputePathToPose
        self._action_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose'
        )

        # wait for action server
        self.get_logger().info('Waiting for ComputePathToPose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available.')

        # Timer to check when we have a pose and then send goal
        self.create_timer(1.0, self.send_goal_if_ready)

        # Example goal: modify these values as needed
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = 1.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.orientation.w = 1.0

        self._goal_sent = False

    def amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        # store latest AMCL pose
        self.current_pose = msg

    def send_goal_if_ready(self):
        if self._goal_sent:
            return
        if self.current_pose is None:
            return

        # (Optional) republish as initial pose to AMCL
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.init_pub.publish(self.current_pose)
        self.get_logger().info('Published initialpose.')

        # build goal request
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = PoseStamped(
            header=self.current_pose.header,
            pose=self.current_pose.pose.pose
        )
        goal_msg.goal = self.goal_pose
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()

        # send it off
        self.get_logger().info(f'Sending path‐planning goal: ({self.goal_pose.pose.position.x}, '
                               f'{self.goal_pose.pose.position.y})')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        send_goal_future.add_done_callback(self.goal_response_cb)
        self._goal_sent = True

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_cb)

    def feedback_cb(self, feedback_msg):
        # ComputePathToPose actually doesn’t send feedback, but stub in case
        self.get_logger().info(f'Feedback: {feedback_msg}')

    def get_result_cb(self, future):
        result = future.result().result
        path = result.path
        self.get_logger().info(f'Received path with {len(path.poses)} waypoints.')
        for idx, p in enumerate(path.poses):
            x = p.pose.position.x
            y = p.pose.position.y
            self.get_logger().info(f'  {idx}: {x:.2f}, {y:.2f}')

        # once done, you can shutdown or send another goal
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimplePathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
