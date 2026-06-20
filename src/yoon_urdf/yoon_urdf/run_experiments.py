#!/usr/bin/env python3
import os
import sys
import time
import json
import subprocess
import signal
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

class ExperimentCollector(Node):
    def __init__(self):
        super().__init__('experiment_collector')
        self.latest_pointcloud = None
        self.latest_grid = None
        
        self.pc_sub = self.create_subscription(
            PointCloud2, '/semantic_pointcloud', self.pointcloud_callback, 10)
        self.grid_sub = self.create_subscription(
            OccupancyGrid, '/semantic_risk_map', self.grid_callback, 10)
            
    def pointcloud_callback(self, msg):
        self.latest_pointcloud = msg
        
    def grid_callback(self, msg):
        self.latest_grid = msg

def run_experiment(name, params):
    print(f"\n==================================================")
    print(f"Starting Experiment: {name}")
    print(f"Parameters: {params}")
    print(f"==================================================")
    
    # 1. Start simulator (mujoco_cam_publisher)
    fovy_val = params.get('fovy', 90.0)
    print(f"Launching mujoco_cam_publisher with fovy: {fovy_val}...")
    sim_proc = subprocess.Popen(
        ['ros2', 'run', 'yoon_urdf', 'mujoco_cam_publisher', '--ros-args', '-p', f'fovy:={fovy_val}'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    
    # Wait for simulator to warm up
    time.sleep(3.0)
    
    # 2. Build feature_mapper arguments (exclude fovy from mapper node params)
    args = ['ros2', 'run', 'yoon_urdf', 'feature_mapper', '--ros-args']
    for k, v in params.items():
        if k != 'fovy':
            args.extend(['-p', f"{k}:={v}"])
        
    print(f"Launching feature_mapper with params: {args}")
    mapper_proc = subprocess.Popen(
        args,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    
    # 3. Start risk_map_generator
    print("Launching risk_map_generator...")
    grid_proc = subprocess.Popen(
        ['ros2', 'run', 'yoon_urdf', 'risk_map_generator'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    
    # 4. Spin ROS2 subscriber to collect data for 25 seconds
    collector = ExperimentCollector()
    start_time = time.time()
    duration = 25.0  # seconds (robot completes slightly more than 1 circle)
    
    print(f"Running simulation and collecting data for {duration} seconds...")
    while time.time() - start_time < duration:
        rclpy.spin_once(collector, timeout_sec=0.1)
        
    # 5. Extract data
    collected_data = {
        "name": name,
        "parameters": params,
        "points": [],
        "grid": None
    }
    
    if collector.latest_pointcloud is not None:
        pc_msg = collector.latest_pointcloud
        num_points = len(pc_msg.data) // 12
        for i in range(num_points):
            x, y, z = struct.unpack_from('fff', pc_msg.data, i * 12)
            # Round for file size and readability
            collected_data["points"].append([
                round(float(x), 4),
                round(float(y), 4),
                round(float(z), 4)
            ])
        print(f"Successfully collected {len(collected_data['points'])} 3D points.")
    else:
        print("Warning: No PointCloud2 message received!")
        
    if collector.latest_grid is not None:
        grid_msg = collector.latest_grid
        collected_data["grid"] = {
            "resolution": float(grid_msg.info.resolution),
            "width": int(grid_msg.info.width),
            "height": int(grid_msg.info.height),
            "origin_x": float(grid_msg.info.origin.position.x),
            "origin_y": float(grid_msg.info.origin.position.y),
            "data": [int(val) for val in grid_msg.data]
        }
        print(f"Successfully collected 2D Grid map of size {grid_msg.info.width}x{grid_msg.info.height}.")
    else:
        print("Warning: No OccupancyGrid message received!")
        
    # 6. Terminate ROS2 nodes
    print("Terminating ROS2 nodes...")
    for proc, label in [(grid_proc, "risk_map_generator"), (mapper_proc, "feature_mapper"), (sim_proc, "mujoco_cam_publisher")]:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3.0)
        except Exception as e:
            print(f"Error terminating {label}: {e}")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except:
                pass
                
    collector.destroy_node()
    time.sleep(2.0)  # cooldown to clear ROS2 DDS discovery
    
    return collected_data

def main():
    # Setup directories
    output_dir = "/home/yoon/yoon_urdf/src/yoon_urdf/resource/viewer/data"
    os.makedirs(output_dir, exist_ok=True)
    
    # Define experiment configurations
    experiments = {
        "Default": {
            "algorithm_mode": "advanced_filter",
            "nfeatures": 3000,
            "fast_threshold": 7,
            "min_baseline": 0.12,
            "min_rotation": 0.20,
            "voxel_size": 0.05,
            "match_threshold": 60,
            "parallax_threshold": 0.044,
            "margin_ratio": 0.15
        },
        "Dense": {
            "algorithm_mode": "advanced_filter",
            "nfeatures": 5000,
            "fast_threshold": 5,
            "min_baseline": 0.06,
            "min_rotation": 0.10,
            "voxel_size": 0.03,
            "match_threshold": 70,
            "parallax_threshold": 0.026,  # ~1.5 deg
            "margin_ratio": 0.05
        },
        "Strict_Filter": {
            "algorithm_mode": "advanced_filter",
            "nfeatures": 3000,
            "fast_threshold": 10,
            "min_baseline": 0.20,
            "min_rotation": 0.26,         # ~15 deg
            "voxel_size": 0.05,
            "match_threshold": 50,
            "parallax_threshold": 0.061,  # ~3.5 deg
            "margin_ratio": 0.20
        },
        "Lenient_Filter": {
            "algorithm_mode": "basic",
            "nfeatures": 3000,
            "fast_threshold": 7,
            "min_baseline": 0.05,
            "min_rotation": 0.08,         # ~5 deg
            "voxel_size": 0.05,
            "match_threshold": 70,
            "parallax_threshold": 0.026,  # ~1.5 deg
            "margin_ratio": 0.10
        },
        "High_Resolution": {
            "algorithm_mode": "sliding_window",
            "nfeatures": 3000,
            "fast_threshold": 7,
            "min_baseline": 0.12,
            "min_rotation": 0.20,
            "voxel_size": 0.02,           # 2cm voxel
            "match_threshold": 60,
            "parallax_threshold": 0.044,
            "margin_ratio": 0.15
        },
        "Fast_Speed": {
            "algorithm_mode": "sliding_window",
            "nfeatures": 1000,
            "fast_threshold": 12,
            "min_baseline": 0.15,
            "min_rotation": 0.20,
            "voxel_size": 0.08,
            "match_threshold": 55,
            "parallax_threshold": 0.044,
            "margin_ratio": 0.15
        },
        "FOV_60": {
            "algorithm_mode": "sliding_window",
            "fovy": 60.0,
            "nfeatures": 3000,
            "fast_threshold": 7,
            "min_baseline": 0.12,
            "min_rotation": 0.20,
            "voxel_size": 0.05,
            "match_threshold": 60,
            "parallax_threshold": 0.044,
            "margin_ratio": 0.15
        },
        "FOV_75": {
            "algorithm_mode": "sliding_window",
            "fovy": 75.0,
            "nfeatures": 3000,
            "fast_threshold": 7,
            "min_baseline": 0.12,
            "min_rotation": 0.20,
            "voxel_size": 0.05,
            "match_threshold": 60,
            "parallax_threshold": 0.044,
            "margin_ratio": 0.15
        },
        "FOV_90": {
            "algorithm_mode": "sliding_window",
            "fovy": 90.0,
            "nfeatures": 3000,
            "fast_threshold": 7,
            "min_baseline": 0.12,
            "min_rotation": 0.20,
            "voxel_size": 0.05,
            "match_threshold": 60,
            "parallax_threshold": 0.044,
            "margin_ratio": 0.15
        },
        "FOV_105": {
            "algorithm_mode": "sliding_window",
            "fovy": 105.0,
            "nfeatures": 3000,
            "fast_threshold": 7,
            "min_baseline": 0.12,
            "min_rotation": 0.20,
            "voxel_size": 0.05,
            "match_threshold": 60,
            "parallax_threshold": 0.044,
            "margin_ratio": 0.15
        }
    }
    
    # Initialize rclpy once
    rclpy.init()
    
    results = {}
    for name, params in experiments.items():
        try:
            data = run_experiment(name, params)
            output_file = os.path.join(output_dir, f"{name}.json")
            with open(output_file, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"Saved results to {output_file}")
            results[name] = len(data["points"])
        except Exception as e:
            print(f"Failed run {name}: {e}")
            
    rclpy.shutdown()
    
    print("\n==================================================")
    print("All Experiments Completed Successfully!")
    for name, count in results.items():
        print(f" - {name}: {count} points")
    print("==================================================")

if __name__ == '__main__':
    main()
