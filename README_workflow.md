# TurtleBot3 Navigation & Imitation Control – Workflow

## **1. Environment Setup**
```bash
# Activate virtual environment
source ~/torch_env/bin/activate

# Install required Python packages
pip install torch torchvision torchaudio pandas PyYAML mcap mcap-ros2 rosbags
```

---

## **2. Start Simulation**
```bash
# Launch TurtleBot3 in Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Launch Navigation2 with map
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=our_ws/my_map.yaml

# (Optional) Launch Cartographer for mapping
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

---

## **3. Monitor Topics**
```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /amcl_pose
ros2 topic echo /initialpose
ros2 topic info /cmd_vel
```

---

## **4. Record Navigation Data**
```bash
mkdir -p our_ws/dataset
cd our_ws/dataset

# Record key topics
ros2 bag record /amcl_pose /cmd_vel -o nav_data

# Record full navigation session
ros2 bag record /initialpose /amcl_pose /odom /cmd_vel --output all_data_bag
```

---

## **5. Process Dataset**
```bash
# Convert recorded bag to CSV
python3 convert_rosbag_to_csv.py /path/to/rosbag_directory
```

---

## **6. Train Neural Controller**
```bash
python3 train_controller.py
```

---

## **7. Build & Run Custom ROS 2 Packages**
```bash
# Simple Path Planner
cd our_ws
colcon build
source install/setup.bash
ros2 run simple_path_planner simple_path_planner

# Imitation Control Package
colcon build --packages-select imitation_control_package
source install/setup.bash
export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH
ros2 run imitation_control_package imitation_control_package
```

---

## **8. Set Initial & Goal Poses**
```bash
# Initial pose
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 0.5, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [0.0]*36
"

# Goal pose
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'map'
pose:
  position: {x: 3.0, y: 2.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

---

## **Tips**
- Always run:
  ```bash
  source install/setup.bash
  ```
  after building.
- Use:
  ```bash
  colcon build --symlink-install
  ```
  for faster development builds.
- Keep your Python environment activated when working with ML scripts.
