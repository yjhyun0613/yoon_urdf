#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
import struct

class RiskMapGenerator(Node):
    def __init__(self):
        super().__init__('risk_map_generator')
        
        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2, '/semantic_pointcloud', self.pointcloud_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_global_pose', self.robot_pose_callback, 10)
            
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/semantic_risk_map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/risk_markers', 10)
        
        # Map Parameters (Static 20m x 20m map centered at origin (0,0))
        self.resolution = 0.1  # 10cm per cell
        self.width_m = 20.0
        self.height_m = 20.0
        
        self.width_cells = int(self.width_m / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)
        
        # Persistent Global Grid Map
        self.global_grid = np.zeros((self.height_cells, self.width_cells))
        
        # Robot ego state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_received = False
        
        # Timer to publish map and decay costs (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(
            f"Risk Map Generator initialized. Map size: {self.width_cells}x{self.height_cells} ({self.resolution}m res)"
        )

    def robot_pose_callback(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose_received = True

    def pointcloud_callback(self, msg: PointCloud2):
        # Extract points from the PointCloud2 message
        num_points = msg.width
        for i in range(num_points):
            offset = i * msg.point_step
            try:
                x, y, z = struct.unpack_from('fff', msg.data, offset)
                
                # Calculate distance to robot
                dx_rob = x - self.robot_x
                dy_rob = y - self.robot_y
                dist = math.sqrt(dx_rob**2 + dy_rob**2)
                
                # Distance-adaptive footprint offsets
                if dist < 2.0:
                    offsets = [0]
                elif dist < 4.0:
                    offsets = [-1, 0, 1]
                else:
                    offsets = [-2, -1, 0, 1, 2]
                
                # Convert global coordinates to map grid indices
                idx_x = int((x - (-self.width_m / 2.0)) / self.resolution)
                idx_y = int((y - (-self.height_m / 2.0)) / self.resolution)
                
                # Check bounds and write to global grid
                max_offset = max(abs(o) for o in offsets)
                if max_offset <= idx_x < self.width_cells - max_offset and max_offset <= idx_y < self.height_cells - max_offset:
                    for dx in offsets:
                        for dy in offsets:
                            self.global_grid[idx_y + dy, idx_x + dx] = 100.0
            except Exception as e:
                self.get_logger().error(f"Error unpacking point: {e}")

    def timer_callback(self):
        # 1. Decay the grid map slowly so that dynamic noise/outliers disappear
        # but persistent features remain solid
        self.global_grid = self.global_grid * 0.95
        self.global_grid[self.global_grid < 10.0] = 0.0
        
        # 2. Publish OccupancyGrid Map
        grid_data_int8 = self.global_grid.astype(np.int8)
        
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width_cells
        map_msg.info.height = self.height_cells
        map_msg.info.origin.position.x = -self.width_m / 2.0
        map_msg.info.origin.position.y = -self.height_m / 2.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.data = grid_data_int8.flatten().tolist()
        self.map_pub.publish(map_msg)
        
        # 3. Publish MarkerArray for RViz safety viz (optional, can be empty or show active obstacles)
        marker_array = MarkerArray()
        # Find active high-cost cells to visualize as markers
        active_indices = np.argwhere(self.global_grid > 50.0)
        
        # Throttling marker output to prevent RViz overhead
        if len(active_indices) > 0:
            # Downsample to maximum 50 markers
            step = max(1, len(active_indices) // 50)
            for m_idx, cell in enumerate(active_indices[::step]):
                y_idx, x_idx = cell
                gx = -self.width_m / 2.0 + x_idx * self.resolution + self.resolution / 2.0
                gy = -self.height_m / 2.0 + y_idx * self.resolution + self.resolution / 2.0
                
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "triangulated_features"
                marker.id = m_idx
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = gx
                marker.pose.position.y = gy
                marker.pose.position.z = 0.4
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.8
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = int(0.15 * 1e9)
                marker_array.markers.append(marker)
                
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RiskMapGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
