#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import struct
import message_filters

class LidarCameraFusion(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion')
        
        # Parameters
        self.declare_parameter('target_frame', 'odom') # frame to publish colored pointcloud in
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('keep_only_visible', True) # only keep points visible in the camera FOV
        
        self.target_frame = self.get_parameter('target_frame').value
        self.sync_slop = self.get_parameter('sync_slop').value
        self.sync_queue_size = self.get_parameter('sync_queue_size').value
        self.keep_only_visible = self.get_parameter('keep_only_visible').value
        
        self.bridge = CvBridge()
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera Intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.camera_info_received = False
        
        # Camera Info Subscriber (to dynamically get intrinsics once)
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/my_robot/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Message synchronizer will be initialized once camera_info is received
        self.sync_initialized = False
        
        self.get_logger().info("Lidar-Camera Fusion Node initialized. Waiting for camera_info...")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info_received:
            return
            
        if msg.k[0] > 0.0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.distortion_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info(f"Received Camera Info: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
            
            # Setup synchronizers now that we have camera parameters
            self.setup_synchronizers()
 
    def setup_synchronizers(self):
        if self.sync_initialized:
            return
            
        # Synchronized subscribers
        self.image_sub = message_filters.Subscriber(self, Image, '/my_robot/camera/image_raw')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/points')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Publisher
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/semantic_pointcloud', 10)
        
        self.sync_initialized = True
        self.get_logger().info("Synchronizers initialized successfully.")

    def quaternion_to_matrix(self, q):
        # q is [x, y, z, w]
        x, y, z, w = q
        return np.array([
            [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
            [2*x*y + 2*z*w,           1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,           2*y*z + 2*x*w,       1 - 2*x**2 - 2*y**2]
        ])

    def transform_to_matrix(self, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        q = [rotation.x, rotation.y, rotation.z, rotation.w]
        R_mat = self.quaternion_to_matrix(q)
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[0, 3] = translation.x
        T[1, 3] = translation.y
        T[2, 3] = translation.z
        return T

    def unpack_pointcloud2(self, msg):
        # Find field offsets
        offset_x, offset_y, offset_z = 0, 4, 8
        for f in msg.fields:
            if f.name == 'x':
                offset_x = f.offset
            elif f.name == 'y':
                offset_y = f.offset
            elif f.name == 'z':
                offset_z = f.offset
                
        num_points = msg.width * msg.height
        if num_points == 0:
            return np.empty((0, 3), dtype=np.float32)
            
        # Convert data buffer to numpy array
        data_uint8 = np.frombuffer(msg.data, dtype=np.uint8)
        data_reshaped = data_uint8.reshape((num_points, msg.point_step))
        
        # Extract x, y, z columns
        x_bytes = data_reshaped[:, offset_x:offset_x+4].copy()
        y_bytes = data_reshaped[:, offset_y:offset_y+4].copy()
        z_bytes = data_reshaped[:, offset_z:offset_z+4].copy()
        
        x = np.frombuffer(x_bytes, dtype=np.float32)
        y = np.frombuffer(y_bytes, dtype=np.float32)
        z = np.frombuffer(z_bytes, dtype=np.float32)
        
        return np.column_stack((x, y, z))

    def array_to_pointcloud2(self, header, points, colors_rgb):
        N = len(points)
        dtype = np.dtype([
            ('x', '<f4'),
            ('y', '<f4'),
            ('z', '<f4'),
            ('rgb', '<u4')
        ])
        
        data = np.zeros(N, dtype=dtype)
        data['x'] = points[:, 0]
        data['y'] = points[:, 1]
        data['z'] = points[:, 2]
        data['rgb'] = colors_rgb
        
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = self.target_frame
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
        
        return msg

    def sync_callback(self, img_msg, pc_msg):
        # 1. Decode Camera Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Decode Error: {e}")
            return
            
        h, w = cv_image.shape[:2]
        
        # Apply lens undistortion if calibration data exists
        if self.distortion_coeffs is not None and len(self.distortion_coeffs) > 0 and np.any(self.distortion_coeffs != 0):
            cv_image = cv2.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
            
        # Update target_frame in case it changes
        self.target_frame = self.get_parameter('target_frame').value
        self.keep_only_visible = self.get_parameter('keep_only_visible').value
        
        # 2. Get TF transforms
        try:
            # Transform from Lidar to Camera Optical frame
            t_cam = self.tf_buffer.lookup_transform(
                'camera_link_optical',
                pc_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            # Transform from Lidar to target_frame (e.g. odom or map)
            t_target = self.tf_buffer.lookup_transform(
                self.target_frame,
                pc_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=3.0)
            return
            
        T_cam_lidar = self.transform_to_matrix(t_cam)
        T_target_lidar = self.transform_to_matrix(t_target)
        
        # 3. Unpack Lidar points
        pts_lidar = self.unpack_pointcloud2(pc_msg)
        if len(pts_lidar) == 0:
            return
            
        # 4. Project points to Camera frame
        pts_hom = np.hstack((pts_lidar, np.ones((len(pts_lidar), 1))))
        pts_cam_hom = pts_hom @ T_cam_lidar.T
        pts_cam = pts_cam_hom[:, :3]
        
        # Select points in front of the camera
        front_mask = pts_cam[:, 2] > 0.1
        
        if not np.any(front_mask):
            if self.keep_only_visible:
                return
            else:
                # If we don't filter, transform everything to target frame with default color
                pts_target_hom = pts_hom @ T_target_lidar.T
                pts_target = pts_target_hom[:, :3]
                default_colors = np.full(len(pts_target), (128 << 16) | (128 << 8) | 128, dtype=np.uint32)
                fused_msg = self.array_to_pointcloud2(pc_msg.header, pts_target, default_colors)
                self.pointcloud_pub.publish(fused_msg)
                return

        # Indices of points that are in front
        idx_front = np.where(front_mask)[0]
        pts_cam_front = pts_cam[front_mask]
        
        # Project to 2D pixels
        Z = pts_cam_front[:, 2]
        X = pts_cam_front[:, 0]
        Y = pts_cam_front[:, 1]
        
        u = (self.fx * X / Z) + self.cx
        v = (self.fy * Y / Z) + self.cy
        
        u_int = np.round(u).astype(int)
        v_int = np.round(v).astype(int)
        
        # Filter pixels within image dimensions
        inside_mask = (u_int >= 0) & (u_int < w) & (v_int >= 0) & (v_int < h)
        
        # Index map mapping back to original pts_lidar indices
        idx_visible = idx_front[inside_mask]
        
        # 5. Extract colors and build colored pointcloud
        if len(idx_visible) > 0:
            # Extract colors (BGR)
            colors_bgr = cv_image[v_int[inside_mask], u_int[inside_mask]]
            R = colors_bgr[:, 2].astype(np.uint32)
            G = colors_bgr[:, 1].astype(np.uint32)
            B = colors_bgr[:, 0].astype(np.uint32)
            rgb_packed_visible = (R << 16) | (G << 8) | B
            
            if self.keep_only_visible:
                # Keep only visible points
                pts_visible_hom = pts_hom[idx_visible]
                pts_target_hom = pts_visible_hom @ T_target_lidar.T
                pts_target = pts_target_hom[:, :3]
                colors_packed = rgb_packed_visible
            else:
                # Keep all points, color the visible ones and set the rest to gray
                pts_target_hom = pts_hom @ T_target_lidar.T
                pts_target = pts_target_hom[:, :3]
                colors_packed = np.full(len(pts_target), (128 << 16) | (128 << 8) | 128, dtype=np.uint32)
                colors_packed[idx_visible] = rgb_packed_visible
                
            # Publish
            fused_msg = self.array_to_pointcloud2(pc_msg.header, pts_target, colors_packed)
            self.pointcloud_pub.publish(fused_msg)
            
        elif not self.keep_only_visible:
            # None are visible but we keep all as gray
            pts_target_hom = pts_hom @ T_target_lidar.T
            pts_target = pts_target_hom[:, :3]
            default_colors = np.full(len(pts_target), (128 << 16) | (128 << 8) | 128, dtype=np.uint32)
            fused_msg = self.array_to_pointcloud2(pc_msg.header, pts_target, default_colors)
            self.pointcloud_pub.publish(fused_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
