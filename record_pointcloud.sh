#!/bin/bash

# Create saved_bags directory
mkdir -p /home/yoon/yoon_urdf/saved_bags

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="/home/yoon/yoon_urdf/saved_bags/bag_$TIMESTAMP"

echo "=============================================="
echo "Starting ROS 2 Bag Recording..."
echo "Topic: /semantic_pointcloud"
echo "Output Directory: $BAG_NAME"
echo "To STOP recording, press Ctrl+C"
echo "=============================================="

# Run ros2 bag record
ros2 bag record -o "$BAG_NAME" /semantic_pointcloud
