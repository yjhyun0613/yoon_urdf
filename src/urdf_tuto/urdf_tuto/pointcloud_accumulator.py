#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import os
import time
import numpy as np

class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('pointcloud_accumulator')
        
        # Parameters
        self.declare_parameter('save_dir', '/home/yoon/yoon_urdf/saved_maps')
        self.declare_parameter('save_interval_sec', 15.0)
        self.declare_parameter('file_format', 'ply')      # 'ply', 'pcd', or 'both'
        self.declare_parameter('voxel_size', 0.02)        # 2cm grid resolution
        
        self.save_dir = self.get_parameter('save_dir').value
        self.save_interval = self.get_parameter('save_interval_sec').value
        self.file_format = self.get_parameter('file_format').value
        self.voxel_size = self.get_parameter('voxel_size').value
        
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Persistent global voxel grid mapping: Key: (ix, iy, iz) -> Value: (x, y, z, r, g, b)
        self.voxel_grid = {}
        self.last_save_time = time.time()
        self.frame_id = 'odom' # Dynamically updated from incoming message headers
        
        # Subscribers
        self.subscription = self.create_subscription(
            PointCloud2,
            '/semantic_pointcloud',
            self.listener_callback,
            10
        )
        
        # Publishers
        self.accumulated_pc_pub = self.create_publisher(PointCloud2, '/accumulated_pointcloud', 10)
        
        # Timers:
        # 1. Auto-save every N seconds
        self.save_timer = self.create_timer(1.0, self.save_timer_callback)
        # 2. Publish accumulated visualization at 2 Hz
        self.pub_timer = self.create_timer(0.5, self.pub_timer_callback)
        
        self.get_logger().info(
            f"PointCloud Accumulator Node initialized.\n"
            f" - Save Directory: {self.save_dir}\n"
            f" - Voxel Resolution: {self.voxel_size}m\n"
            f" - Format: {self.file_format}\n"
            f" - Accumulating points in incoming message frames."
        )

    def listener_callback(self, msg: PointCloud2):
        num_points = msg.width * msg.height
        if num_points == 0:
            return
            
        # Update frame_id dynamically
        self.frame_id = msg.header.frame_id
        
        point_step = msg.point_step
        
        # Check fields
        has_rgb = False
        offset_rgb = 12
        offset_x, offset_y, offset_z = 0, 4, 8
        for field in msg.fields:
            if field.name == 'rgb':
                has_rgb = True
                offset_rgb = field.offset
            elif field.name == 'x':
                offset_x = field.offset
            elif field.name == 'y':
                offset_y = field.offset
            elif field.name == 'z':
                offset_z = field.offset
                
        # Unpack and add to voxel grid
        # We can extract all points using numpy for fast processing
        data_uint8 = np.frombuffer(msg.data, dtype=np.uint8)
        data_reshaped = data_uint8.reshape((num_points, point_step))
        
        x_bytes = data_reshaped[:, offset_x:offset_x+4].copy()
        y_bytes = data_reshaped[:, offset_y:offset_y+4].copy()
        z_bytes = data_reshaped[:, offset_z:offset_z+4].copy()
        
        x = np.frombuffer(x_bytes, dtype=np.float32)
        y = np.frombuffer(y_bytes, dtype=np.float32)
        z = np.frombuffer(z_bytes, dtype=np.float32)
        
        if has_rgb:
            rgb_bytes = data_reshaped[:, offset_rgb:offset_rgb+4].copy()
            rgb_val = np.frombuffer(rgb_bytes, dtype=np.uint32)
            r = ((rgb_val >> 16) & 0xFF).astype(np.uint8)
            g = ((rgb_val >> 8) & 0xFF).astype(np.uint8)
            b = (rgb_val & 0xFF).astype(np.uint8)
        else:
            r = np.full(num_points, 255, dtype=np.uint8)
            g = np.full(num_points, 255, dtype=np.uint8)
            b = np.full(num_points, 255, dtype=np.uint8)
            
        # Add to voxel grid
        for i in range(num_points):
            ix = int(x[i] / self.voxel_size)
            iy = int(y[i] / self.voxel_size)
            iz = int(z[i] / self.voxel_size)
            key = (ix, iy, iz)
            self.voxel_grid[key] = (float(x[i]), float(y[i]), float(z[i]), int(r[i]), int(g[i]), int(b[i]))

    def pub_timer_callback(self):
        # Publish current accumulated voxel grid as PointCloud2 message
        points = list(self.voxel_grid.values())
        if len(points) == 0:
            return
            
        header = self.get_clock().now().to_msg()
        # Create structured array for speed
        N = len(points)
        dtype = np.dtype([
            ('x', '<f4'),
            ('y', '<f4'),
            ('z', '<f4'),
            ('rgb', '<u4')
        ])
        
        data = np.zeros(N, dtype=dtype)
        for i, p in enumerate(points):
            rgb_val = (p[3] << 16) | (p[4] << 8) | p[5]
            data[i] = (p[0], p[1], p[2], rgb_val)
            
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = header
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = N
        msg.is_dense = True
        msg.is_bigendian = False
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = data.tobytes()
        
        self.accumulated_pc_pub.publish(msg)

    def save_timer_callback(self):
        # Auto-save periodically
        current_time = time.time()
        if current_time - self.last_save_time >= self.save_interval:
            points = list(self.voxel_grid.values())
            if len(points) > 0:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename_prefix = f"accumulated_map_{timestamp}"
                
                if self.file_format in ['ply', 'both']:
                    self.write_ply(filename_prefix, points)
                if self.file_format in ['pcd', 'both']:
                    self.write_pcd(filename_prefix, points)
                self.last_save_time = current_time

    def write_ply(self, prefix, points):
        filename = os.path.join(self.save_dir, f"{prefix}.ply")
        try:
            with open(filename, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("property uchar red\n")
                f.write("property uchar green\n")
                f.write("property uchar blue\n")
                f.write("end_header\n")
                
                for p in points:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[3]} {p[4]} {p[5]}\n")
            self.get_logger().info(f"Saved accumulated PLY: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to write PLY file: {e}")

    def write_pcd(self, prefix, points):
        filename = os.path.join(self.save_dir, f"{prefix}.pcd")
        try:
            with open(filename, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z rgb\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F U\n")
                f.write("COUNT 1 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                
                for p in points:
                    rgb_val = (p[3] << 16) | (p[4] << 8) | p[5]
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {rgb_val}\n")
            self.get_logger().info(f"Saved accumulated PCD: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to write PCD file: {e}")

    def destroy_node(self):
        # Save final complete map on shutdown
        points = list(self.voxel_grid.values())
        if len(points) > 0:
            self.get_logger().info(f"Shutting down accumulator node. Writing final map of {len(points)} points...")
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename_prefix = f"final_accumulated_map_{timestamp}"
            
            if self.file_format in ['ply', 'both']:
                self.write_ply(filename_prefix, points)
            if self.file_format in ['pcd', 'both']:
                self.write_pcd(filename_prefix, points)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
