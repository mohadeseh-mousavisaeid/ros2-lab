==============================
Obstacle Avoidance Controller
ROS 2 Jazzy + Gazebo (GZ Sim)
==============================

🧩 Requirements:
----------------
- ROS 2 Jazzy
- GZ Sim (Ignition/Gazebo)
- TurtleBot3 packages:

sudo apt install ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-description \
ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim

======================
🛠️ Setup Instructions:
======================

cd ~/test_project
colcon build
source install/setup.bash

# Optional: Auto-source in terminal
echo "source ~/test_project/install/setup.bash" >> ~/.bashrc

========================
🚀 Run Simulation (GZ):
========================

export TURTLEBOT3_MODEL=burger
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf

# Spawn TurtleBot3 in the empty world:
ros2 run ros_gz_sim create -name turtlebot3 \
-string "$(gz sdf -p /opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf)" \
-x 0 -y 0 -z 0.1

ros2 topic echo /scan  # Check LiDAR data

=============================
🧠 Run Obstacle Controller:
=============================

ros2 run obstacle_avoidance_controller obstacle_avoidance

# Log output will show robot decisions based on scan:
# [INFO] [obstacle_avoid_controller]: Obstacle nearby: slowing down

=======================
🧪 Test & Visualize:
=======================

ros2 topic echo /cmd_vel        # Check velocity output
ros2 run rviz2 rviz2            # Visualize environment

=====================
📁 Project Structure:
=====================

test_project/
└── src/
    └── obstacle_avoidance_controller/
        ├── controller_node.py
        ├── package.xml
        └── setup.py

=================
👤 Author: Mohadeseh
=================
