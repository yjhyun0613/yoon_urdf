#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

class PedestrianMockNode(Node):
    def __init__(self):
        super().__init__('pedestrian_mock_node')
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseArray, '/pedestrian_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pedestrian_markers', 10)
        
        # Parameters
        self.num_pedestrians = 10
        self.dt = 0.1  # 10 Hz
        
        # Initialize pedestrian states: [x, y, yaw, speed]
        # Spawning them in a 10m x 10m area
        self.pedestrians = []
        for i in range(self.num_pedestrians):
            x = np.random.uniform(-8.0, 8.0)
            y = np.random.uniform(-8.0, 8.0)
            yaw = np.random.uniform(-math.pi, math.pi)
            speed = np.random.uniform(0.5, 1.2)  # typical human walking speed in m/s
            self.pedestrians.append([x, y, yaw, speed])
            
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Pedestrian Mock Node started. Simulating 10 pedestrians.")

    def timer_callback(self):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = 'map'  # World coordinate system
        
        marker_array_msg = MarkerArray()
        
        for idx, ped in enumerate(self.pedestrians):
            x, y, yaw, speed = ped
            
            # 1. Update state (Random walk with heading change)
            yaw += np.random.normal(0, 0.2)  # minor direction change
            x += speed * math.cos(yaw) * self.dt
            y += speed * math.sin(yaw) * self.dt
            
            # Boundary collision: bounce back towards center
            if abs(x) > 10.0 or abs(y) > 10.0:
                yaw = math.atan2(-y, -x) + np.random.uniform(-0.5, 0.5)
                # pull back slightly inside boundaries
                x = np.clip(x, -9.5, 9.5)
                y = np.clip(y, -9.5, 9.5)
                
            # Update internal list
            self.pedestrians[idx] = [x, y, yaw, speed]
            
            # 2. Populate PoseArray message
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            
            # Convert yaw to quaternion
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            
            pose_array_msg.poses.append(pose)
            
            # 3. Populate Marker (Cylinder representation for RViz2)
            marker = Marker()
            marker.header = pose_array_msg.header
            marker.ns = "pedestrians"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.85  # Z is center of cylinder (height is 1.7m)
            marker.pose.orientation = pose.orientation
            
            # Dimensions: diameter 0.5m, height 1.7m
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 1.7
            
            # Color: Cyan (semi-transparent)
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 0.8
            marker.color.a = 0.8
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = int(0.2 * 1e9)
            
            marker_array_msg.markers.append(marker)
            
        self.pose_pub.publish(pose_array_msg)
        self.marker_pub.publish(marker_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PedestrianMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
