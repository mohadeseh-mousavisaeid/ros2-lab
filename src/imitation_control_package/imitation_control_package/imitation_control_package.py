# nn_controller_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
import torch
import torch.nn as nn

import numpy as np

# Define the same model structure
class ControllerNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(7, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 2)
        )

    def forward(self, x):
        return self.net(x)

class NNControllerNode(Node):
    def __init__(self):
        super().__init__('nn_controller_node')
        print("⚙️ Initializing controller node...")

        # Load model
        self.model = ControllerNN()
        try:
            self.model.load_state_dict(torch.load('20_08__nn_controller_2.pt'))
            self.model.eval()
            print("✅ Loaded model successfully.")
        except Exception as e:
            print(f"❌ Failed to load model: {e}")

        self.initial_pose = None
        self.odom = None

        print("📡 Creating subscriptions...")
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        print("⏲️ Starting control loop timer...")
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def initialpose_callback(self, msg):
        print("📍 Received /initialpose")
        self.initial_pose = msg.pose.pose

    def odom_callback(self, msg):
        print("🚗 Received /odom")
        self.odom = msg

    def control_loop(self):
        self.initial_pose = Pose()
        self.initial_pose.position.x = 3.0
        self.initial_pose.position.y = 2.0
        self.initial_pose.orientation.w = 1.0  # valid quaternion
        print("🎯 Manually set initial pose.")

        if self.odom is None :
            print("⏳ Waiting for /odom...")
            return

        print("🧠 Running control loop...")

        odom_pose = self.odom.pose.pose
        odom_twist = self.odom.twist.twist
        input_tensor = torch.tensor([[
            odom_pose.position.x,
            odom_pose.position.y,
            odom_twist.linear.x,
            odom_twist.linear.y,
            odom_twist.angular.z,
            self.initial_pose.position.x,
            self.initial_pose.position.y
        ]], dtype=torch.float32)

        with torch.no_grad():
            output = self.model(input_tensor).numpy()[0]

        print(f"📤 Publishing cmd_vel: linear_x = {output[0]:.2f}, angular_z = {output[1]:.2f}")
        cmd = TwistStamped()
        cmd.twist.linear.x = float(output[0])
        cmd.twist.angular.z = float(output[1])
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    print("🚀 Starting NN controller node...")
    node = NNControllerNode()
    rclpy.spin(node)
    print("🛑 Node shutdown.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
# import torch
# import torch.nn as nn
# import numpy as np
# import math
# import os
# from ament_index_python.packages import get_package_share_directory

# class ControllerNN(nn.Module):
#     def __init__(self):
#         super().__init__()
#         self.net = nn.Sequential(
#             nn.Linear(3, 64),
#             nn.ReLU(),
#             nn.Linear(64, 64),
#             nn.ReLU(),
#             nn.Linear(64, 2)
#         )

#     def forward(self, x):
#         return self.net(x)

# class NNControllerNode(Node):
#     def __init__(self):
#         super().__init__('nn_controller')
#         self.model = ControllerNN()

#         # Get model path from package
#         pkg_path = get_package_share_directory('imitation_control_package')
#         model_path = os.path.join(pkg_path, 'models', 'new_nn_controller.pt')

#         self.get_logger().info(f"Loading model from: {model_path}")
#         self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
#         self.model.eval()

#         self.goal = np.array([1.0, 3.0])  # static goal
        
#         self.get_logger().info("Node initialized. Subscribing to /amcl_pose...")
#         self.sub = self.create_subscription(
#             PoseWithCovarianceStamped,
#             '/amcl_pose',
#             self.pose_callback,
#             10
#         )

#         self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

#     def pose_callback(self, msg):
#         self.get_logger().info("Received pose update")

#         p = msg.pose.pose.position
#         q = msg.pose.pose.orientation
#         yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

#         input_tensor = torch.tensor([[p.x, p.y, yaw]], dtype=torch.float32)
#         output = self.model(input_tensor).detach().numpy()[0]

#         twist = Twist()
#         twist.linear.x = float(output[0])
#         twist.angular.z = float(output[1])

#         self.get_logger().info(f"Publishing Twist: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")

#         self.pub.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = NNControllerNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
