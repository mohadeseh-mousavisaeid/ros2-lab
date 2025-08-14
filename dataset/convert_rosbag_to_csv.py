import os
import csv
import rclpy
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from datetime import datetime

# --- Config ---
BAG_PATH = 'all_data_bag'  # Folder name (not .mcap file)
RESAMPLE_MS = 100  # Resample to 10 Hz

# Topics to extract
TOPICS = {
    "/initialpose": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/amcl_pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/odom": "nav_msgs/msg/Odometry",
    "/cmd_vel": "geometry_msgs/msg/TwistStamped",
}

# Init ROS
rclpy.init()

# Open bag with rosbag2_py
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id="mcap")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

# Setup deserialization
type_map = {topic: get_message(msg_type) for topic, msg_type in TOPICS.items()}
data_by_topic = {topic: [] for topic in TOPICS}

# Read all messages
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic in TOPICS:
        msg_type = type_map[topic]
        msg = deserialize_message(data, msg_type)
        timestamp = pd.to_datetime(t / 1e9, unit='s')

        entry = {"timestamp": timestamp}

        if topic in ["/initialpose", "/amcl_pose"]:
            pose = msg.pose.pose
            entry.update({
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "qx": pose.orientation.x,
                "qy": pose.orientation.y,
                "qz": pose.orientation.z,
                "qw": pose.orientation.w,
            })
        elif topic == "/odom":
            entry.update({
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "vx": msg.twist.twist.linear.x,
                "vy": msg.twist.twist.linear.y,
                "angular_z": msg.twist.twist.angular.z,
            })
        elif topic == "/cmd_vel":
            entry.update({
                "vx": msg.twist.linear.x,
                "vy": msg.twist.linear.y,
                "angular_z": msg.twist.angular.z,
            })

        data_by_topic[topic].append(entry)

# Convert and resample DataFrames
dfs = []
for topic, records in data_by_topic.items():
    if not records:
        continue
    df = pd.DataFrame(records)
    df = df.set_index("timestamp").sort_index()
    df = df.resample(f"{RESAMPLE_MS}ms").nearest(limit=1)
    df = df.add_prefix(f"{topic.strip('/').replace('/', '_')}_")
    dfs.append(df)

# Merge all DataFrames on timestamp
merged_df = pd.concat(dfs, axis=1).reset_index()

# ✅ 1. Add constant goal pose columns
merged_df["goal_x"] = 0.0
merged_df["goal_y"] = 0.0
merged_df["goal_z"] = 0.0
merged_df["goal_qx"] = 0.0
merged_df["goal_qy"] = 0.0
merged_df["goal_qz"] = 0.0
merged_df["goal_qw"] = 1.0

# ✅ 2. Fill initialpose_* columns with the single value
initial_cols = [col for col in merged_df.columns if col.startswith("initialpose_")]
for col in initial_cols:
    first_value = merged_df[col].dropna().iloc[0] if not merged_df[col].dropna().empty else 0.0
    merged_df[col] = first_value

# ✅ 3. Save with timestamp
timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_filename = f"all_data_merged_{timestamp_str}.csv"
merged_df.to_csv(output_filename, index=False)
print(f"✅ Saved {output_filename} with shape: {merged_df.shape}")




# import os
# import csv
# import rclpy
# import pandas as pd
# from rclpy.serialization import deserialize_message
# from rosidl_runtime_py.utilities import get_message
# import rosbag2_py
# from datetime import datetime

# # --- Config ---
# BAG_PATH = 'all_data_bag'  # Folder name (not .mcap file)
# OUTPUT_CSV = 'all_data_merged.csv'
# RESAMPLE_MS = 100  # Resample to 10 Hz

# # Topics to extract
# TOPICS = {
#     "/initialpose": "geometry_msgs/msg/PoseWithCovarianceStamped",
#     "/amcl_pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
#     "/odom": "nav_msgs/msg/Odometry",
#     "/cmd_vel": "geometry_msgs/msg/TwistStamped",
# }

# # Init ROS
# rclpy.init()

# # Open bag with rosbag2_py
# reader = rosbag2_py.SequentialReader()
# storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id="mcap")
# converter_options = rosbag2_py.ConverterOptions("", "")
# reader.open(storage_options, converter_options)

# # Setup deserialization
# type_map = {topic: get_message(msg_type) for topic, msg_type in TOPICS.items()}
# data_by_topic = {topic: [] for topic in TOPICS}

# # Read all messages
# while reader.has_next():
#     topic, data, t = reader.read_next()
#     if topic in TOPICS:
#         msg_type = type_map[topic]
#         msg = deserialize_message(data, msg_type)
#         timestamp = pd.to_datetime(t / 1e9, unit='s')

#         entry = {"timestamp": timestamp}

#         if topic in ["/initialpose", "/amcl_pose"]:
#             pose = msg.pose.pose
#             entry.update({
#                 "x": pose.position.x,
#                 "y": pose.position.y,
#                 "z": pose.position.z,
#                 "qx": pose.orientation.x,
#                 "qy": pose.orientation.y,
#                 "qz": pose.orientation.z,
#                 "qw": pose.orientation.w,
#             })
#         elif topic == "/odom":
#             entry.update({
#                 "x": msg.pose.pose.position.x,
#                 "y": msg.pose.pose.position.y,
#                 "vx": msg.twist.twist.linear.x,
#                 "vy": msg.twist.twist.linear.y,
#                 "angular_z": msg.twist.twist.angular.z,
#             })
#         elif topic == "/cmd_vel":
#             entry.update({
#                 "vx": msg.twist.linear.x,
#                 "vy": msg.twist.linear.y,
#                 "angular_z": msg.twist.angular.z,
#             })

#         data_by_topic[topic].append(entry)

# # Convert and resample DataFrames
# dfs = []
# for topic, records in data_by_topic.items():
#     if not records:
#         continue
#     df = pd.DataFrame(records)
#     df = df.set_index("timestamp").sort_index()
#     df = df.resample(f"{RESAMPLE_MS}ms").nearest(limit=1)
#     df = df.add_prefix(f"{topic.strip('/').replace('/', '_')}_")
#     dfs.append(df)

# # Merge all DataFrames on timestamp
# merged_df = pd.concat(dfs, axis=1).reset_index()

# # Save to CSV
# merged_df.to_csv(OUTPUT_CSV, index=False)
# print(f"✅ Saved {OUTPUT_CSV} with shape: {merged_df.shape}")
