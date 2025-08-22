#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def euler_to_quaternion(yaw: float):
    """Convert yaw (rad) to quaternion (x,y,z,w)."""
    return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))


class SendGoal(Node):
    def __init__(self):
        super().__init__('send_goal_node')

        # Action client for Nav2
        self._ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for Nav2...")
        self._ac.wait_for_server()

        # Define goal (map frame, (0,0))
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        qx, qy, qz, qw = euler_to_quaternion(0.0)  # facing forward
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        self.get_logger().info("Sending goal (0,0)...")
        send_future = self._ac.send_goal_async(nav_goal, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.response_cb)

    def response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted!")
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def feedback_cb(self, feedback):
        pos = feedback.feedback.current_pose.pose.position
        self.get_logger().info(f"→ moving… x={pos.x:.2f}, y={pos.y:.2f}")

    def result_cb(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("✅ Goal reached!")
        else:
            self.get_logger().error(f"❌ Failed, error code={result.error_code}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SendGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():   # ✅ only shutdown if still running
            rclpy.shutdown()


if __name__ == '__main__':
    main()
