#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import struct
import message_filters
from message_filters import Subscriber, ApproximateTimeSynchronizer

class FeatureMapper(Node):
    def __init__(self):
        super().__init__('feature_mapper')
        
        # Parameters
        self.declare_parameter('camera_offset_x', 0.25)
        self.declare_parameter('camera_height', 0.8)
        self.declare_parameter('nfeatures', 3000)
        self.declare_parameter('fast_threshold', 7)
        self.declare_parameter('min_baseline', 0.12)
        self.declare_parameter('min_rotation', 0.20)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('min_keypoint_dist', 5.0)
        self.declare_parameter('match_threshold', 60)
        self.declare_parameter('parallax_threshold', 0.044)
        self.declare_parameter('margin_ratio', 0.15)
        self.declare_parameter('algorithm_mode', 'advanced_filter') # basic, advanced_filter, sliding_window
        
        self.camera_offset_x = self.get_parameter('camera_offset_x').value
        self.camera_height = self.get_parameter('camera_height').value
        self.nfeatures = self.get_parameter('nfeatures').value
        self.fast_threshold = self.get_parameter('fast_threshold').value
        self.min_baseline = self.get_parameter('min_baseline').value
        self.min_rotation = self.get_parameter('min_rotation').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.min_keypoint_dist = self.get_parameter('min_keypoint_dist').value
        self.match_threshold = self.get_parameter('match_threshold').value
        self.parallax_threshold = self.get_parameter('parallax_threshold').value
        self.margin_ratio = self.get_parameter('margin_ratio').value
        self.algorithm_mode = self.get_parameter('algorithm_mode').value
        
        # Sliding Window Queue (max size 3)
        self.sliding_window = []
        
        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=self.nfeatures, fastThreshold=self.fast_threshold)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Camera Intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.K = None
        
        # Robot States
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_received = False
        
        # Previous Frame Cache for Triangulation
        self.prev_gray = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_P = None
        
        # Keyframe tracking for baseline control
        self.prev_cam_x = None
        self.prev_cam_y = None
        self.prev_cam_yaw = None
        
        # Persistent Point Cloud Database (Voxel Grid Filter)
        self.accumulated_points = []
        self.mapped_voxels = set()
        
        # message_filters Subscribers
        self.img_filter_sub = Subscriber(self, Image, '/camera/image_raw')
        self.pose_filter_sub = Subscriber(self, PoseStamped, '/robot_global_pose')
        
        # ApproximateTimeSynchronizer (slop=0.05s matching 20Hz publishing rate)
        self.ts = ApproximateTimeSynchronizer(
            [self.img_filter_sub, self.pose_filter_sub],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Standard subscription for CameraInfo (static intrinsics)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
            
        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/semantic_pointcloud', 10)
        
        self.get_logger().info("Feature Mapper Node initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        if msg.k[0] > 0.0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.K = np.array([
                [self.fx, 0.0,     self.cx],
                [0.0,     self.fy, self.cy],
                [0.0,     0.0,     1.0]
            ])

    def get_projection_matrix(self):
        """
        Computes the camera projection matrix P = K * [R | T] in the global frame
        """
        if self.K is None or not self.robot_pose_received:
            return None
            
        # 1. Camera center in global map frame
        cx_w = self.robot_x + self.camera_offset_x * math.cos(self.robot_yaw)
        cy_w = self.robot_y + self.camera_offset_x * math.sin(self.robot_yaw)
        cz_w = self.camera_height
        C = np.array([cx_w, cy_w, cz_w])
        
        # 2. Rotation matrix from map frame to camera frame (R = R_wc^T)
        # Camera Z-axis = robot X-axis (forward) -> [cos(yaw), sin(yaw), 0]
        # Camera X-axis = robot -Y-axis (right)   -> [sin(yaw), -cos(yaw), 0]
        # Camera Y-axis = robot -Z-axis (down)    -> [0, 0, -1]
        R = np.array([
            [math.sin(self.robot_yaw), -math.cos(self.robot_yaw), 0.0],
            [0.0,                      0.0,                      -1.0],
            [math.cos(self.robot_yaw), math.sin(self.robot_yaw),  0.0]
        ])
        
        # 3. Translation T = -R * C
        T = -R.dot(C)
        
        # 4. Extrinsics [R | T]
        extrinsic = np.hstack((R, T.reshape(3, 1)))
        
        # 5. Projection matrix P = K * [R | T]
        P = self.K.dot(extrinsic)
        return P

    def filter_keypoints(self, kp, des, width=640, height=480):
        if len(kp) == 0:
            return kp, des
            
        # Define border margins
        margin_x = self.margin_ratio * width
        margin_y = self.margin_ratio * height
        
        # Sort keypoints by response descending (best quality first)
        indices = np.argsort([-k.response for k in kp])
        kp_sorted = [kp[i] for i in indices]
        des_sorted = des[indices] if des is not None else None
        
        keep_indices = []
        kept_pts = []
        
        for idx, k in enumerate(kp_sorted):
            pt = k.pt
            
            # Border check
            if pt[0] < margin_x or pt[0] > (width - margin_x) or pt[1] < margin_y or pt[1] > (height - margin_y):
                continue
                
            too_close = False
            for kept_pt in kept_pts:
                dist = math.hypot(pt[0] - kept_pt[0], pt[1] - kept_pt[1])
                if dist < self.min_keypoint_dist:
                    too_close = True
                    break
            if not too_close:
                keep_indices.append(idx)
                kept_pts.append(pt)
                
        kp_filtered = [kp_sorted[i] for i in keep_indices]
        des_filtered = des_sorted[keep_indices] if des_sorted is not None else None
        return kp_filtered, des_filtered

    def sync_callback(self, img_msg: Image, pose_msg: PoseStamped):
        if self.K is None:
            return
            
        # Update robot pose from the synchronized pose_msg
        self.robot_x = pose_msg.pose.position.x
        self.robot_y = pose_msg.pose.position.y
        
        # Convert quaternion to yaw
        q = pose_msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose_received = True

        # Calculate current camera center in global map frame
        cx_w = self.robot_x + self.camera_offset_x * math.cos(self.robot_yaw)
        cy_w = self.robot_y + self.camera_offset_x * math.sin(self.robot_yaw)
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return
            
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect ORB keypoints
        kp, des = self.orb.detectAndCompute(gray, None)
        h, w = gray.shape[:2]
        kp, des = self.filter_keypoints(kp, des, w, h)
        P_curr = self.get_projection_matrix()
        
        if P_curr is None or des is None:
            return

        # Delegate to the chosen algorithm mode
        if self.algorithm_mode == 'sliding_window':
            self.run_sliding_window_triangulation(cv_image, gray, kp, des, P_curr, cx_w, cy_w, img_msg.header)
        elif self.algorithm_mode == 'basic':
            # Initialize keyframe if needed
            if self.prev_cam_x is None:
                self.prev_cam_x = cx_w
                self.prev_cam_y = cy_w
                self.prev_cam_yaw = self.robot_yaw
                self.prev_gray = gray
                self.prev_kp = kp
                self.prev_des = des
                self.prev_P = P_curr
                return
                
            dist = math.sqrt((cx_w - self.prev_cam_x)**2 + (cy_w - self.prev_cam_y)**2)
            diff_yaw = abs(math.atan2(math.sin(self.robot_yaw - self.prev_cam_yaw), math.cos(self.robot_yaw - self.prev_cam_yaw)))
            
            if dist < self.min_baseline and diff_yaw < self.min_rotation:
                if self.accumulated_points:
                    pc_msg = self.create_pointcloud_msg(img_msg.header, self.accumulated_points)
                    self.pointcloud_pub.publish(pc_msg)
                return
                
            self.run_basic_triangulation(cv_image, gray, kp, des, P_curr, cx_w, cy_w, img_msg.header)
        else: # advanced_filter (default)
            # Initialize keyframe if needed
            if self.prev_cam_x is None:
                self.prev_cam_x = cx_w
                self.prev_cam_y = cy_w
                self.prev_cam_yaw = self.robot_yaw
                self.prev_gray = gray
                self.prev_kp = kp
                self.prev_des = des
                self.prev_P = P_curr
                return
                
            dist = math.sqrt((cx_w - self.prev_cam_x)**2 + (cy_w - self.prev_cam_y)**2)
            diff_yaw = abs(math.atan2(math.sin(self.robot_yaw - self.prev_cam_yaw), math.cos(self.robot_yaw - self.prev_cam_yaw)))
            
            if dist < self.min_baseline and diff_yaw < self.min_rotation:
                if self.accumulated_points:
                    pc_msg = self.create_pointcloud_msg(img_msg.header, self.accumulated_points)
                    self.pointcloud_pub.publish(pc_msg)
                return
                
            self.run_advanced_filter_triangulation(cv_image, gray, kp, des, P_curr, cx_w, cy_w, img_msg.header, dist, diff_yaw)
            
        # Show output stream with keypoints
        cv2.imshow("AMR Feature Tracker Stream", cv_image)
        cv2.waitKey(1)

    def run_basic_triangulation(self, cv_image, gray, kp, des, P_curr, cx_w, cy_w, header):
        bf_basic = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        valid_triangulated_points = []
        
        if self.prev_des is not None and des is not None:
            matches = bf_basic.match(self.prev_des, des)
            matches = sorted(matches, key=lambda x: x.distance)[:500]
            
            if len(matches) >= 8:
                pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                pts2 = np.float32([kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
                
                pts1_2d = pts1.reshape(-1, 2)
                pts2_2d = pts2.reshape(-1, 2)
                points_4d = cv2.triangulatePoints(self.prev_P, P_curr, pts1_2d.T, pts2_2d.T)
                points_3d = (points_4d[:3, :] / points_4d[3, :]).T
                
                for p in points_3d:
                    dist_to_robot = math.sqrt((p[0] - self.robot_x)**2 + (p[1] - self.robot_y)**2)
                    if 0.5 < dist_to_robot < 6.0 and -0.01 <= p[2] <= 1.8:
                        if np.isfinite(p).all():
                            ix = int(p[0] / self.voxel_size)
                            iy = int(p[1] / self.voxel_size)
                            iz = int(p[2] / self.voxel_size)
                            voxel_coord = (ix, iy, iz)
                            
                            if voxel_coord not in self.mapped_voxels:
                                self.mapped_voxels.add(voxel_coord)
                                self.accumulated_points.append(p)
                                valid_triangulated_points.append(p)
                                
                for pt in pts2_2d:
                    cv2.circle(cv_image, (int(pt[0]), int(pt[1])), 4, (0, 0, 255), -1)
                    
        self.get_logger().info(f"[Basic] Reconstructed {len(valid_triangulated_points)} points. Total: {len(self.accumulated_points)}", throttle_duration_sec=1.0)
        
        if self.accumulated_points:
            pc_msg = self.create_pointcloud_msg(header, self.accumulated_points)
            self.pointcloud_pub.publish(pc_msg)
            
        # Update keyframe cache every step (Frame-to-Frame)
        self.prev_cam_x = cx_w
        self.prev_cam_y = cy_w
        self.prev_cam_yaw = self.robot_yaw
        self.prev_gray = gray
        self.prev_kp = kp
        self.prev_des = des
        self.prev_P = P_curr

    def run_advanced_filter_triangulation(self, cv_image, gray, kp, des, P_curr, cx_w, cy_w, header, dist, diff_yaw):
        valid_triangulated_points = []
        
        # Match using KNN and Lowe's ratio test
        good_matches = []
        if self.prev_des is not None and des is not None and len(self.prev_des) >= 2 and len(des) >= 2:
            try:
                matches = self.bf.knnMatch(self.prev_des, des, k=2)
                for m, n in matches:
                    if m.distance < self.match_threshold and m.distance < 0.75 * n.distance:
                        good_matches.append(m)
            except cv2.error as e:
                self.get_logger().warn(f"KNN Match failed: {e}")
                
        good_matches = sorted(good_matches, key=lambda x: x.distance)[:500]
        
        if len(good_matches) >= 8:
            pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            pts2 = np.float32([kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            
            try:
                method = getattr(cv2, 'USAC_MAGSAC', cv2.FM_RANSAC)
                F, mask_inliers = cv2.findFundamentalMat(pts1, pts2, method, 3.0)
            except cv2.error as e:
                self.get_logger().warn(f"OpenCV findFundamentalMat failed: {e}")
                return
                
            if F is not None and mask_inliers is not None and np.sum(mask_inliers) >= 8:
                mask_inliers = mask_inliers.ravel()
                pts1 = pts1[mask_inliers == 1]
                pts2 = pts2[mask_inliers == 1]
                
                pts1_2d = pts1.reshape(-1, 2)
                pts2_2d = pts2.reshape(-1, 2)
                points_4d = cv2.triangulatePoints(self.prev_P, P_curr, pts1_2d.T, pts2_2d.T)
                points_3d = (points_4d[:3, :] / points_4d[3, :]).T
                
                C1 = np.array([self.prev_cam_x, self.prev_cam_y, self.camera_height])
                C2 = np.array([cx_w, cy_w, self.camera_height])
                
                for i, p in enumerate(points_3d):
                    dist_to_robot = math.sqrt((p[0] - self.robot_x)**2 + (p[1] - self.robot_y)**2)
                    if 0.5 < dist_to_robot < 6.0 and -0.01 <= p[2] <= 1.8:
                        if np.isfinite(p).all():
                            # Reprojection error filter
                            p_homo = np.array([p[0], p[1], p[2], 1.0])
                            
                            pt1_proj_h = self.prev_P.dot(p_homo)
                            if abs(pt1_proj_h[2]) > 1e-5:
                                u1_proj = pt1_proj_h[0] / pt1_proj_h[2]
                                v1_proj = pt1_proj_h[1] / pt1_proj_h[2]
                                err1 = math.hypot(u1_proj - pts1_2d[i][0], v1_proj - pts1_2d[i][1])
                            else:
                                err1 = 999.0
                                
                            pt2_proj_h = P_curr.dot(p_homo)
                            if abs(pt2_proj_h[2]) > 1e-5:
                                u2_proj = pt2_proj_h[0] / pt2_proj_h[2]
                                v2_proj = pt2_proj_h[1] / pt2_proj_h[2]
                                err2 = math.hypot(u2_proj - pts2_2d[i][0], v2_proj - pts2_2d[i][1])
                            else:
                                err2 = 999.0
                                
                            if err1 > 3.0 or err2 > 3.0:
                                continue
                                
                            # Parallax filter
                            v1 = p - C1
                            v2 = p - C2
                            norm_v1 = np.linalg.norm(v1)
                            norm_v2 = np.linalg.norm(v2)
                            if norm_v1 > 1e-5 and norm_v2 > 1e-5:
                                cos_theta = np.dot(v1, v2) / (norm_v1 * norm_v2)
                                cos_theta = np.clip(cos_theta, -1.0, 1.0)
                                parallax_angle = math.acos(cos_theta)
                                if parallax_angle < self.parallax_threshold:
                                    continue
                                    
                            ix = int(p[0] / self.voxel_size)
                            iy = int(p[1] / self.voxel_size)
                            iz = int(p[2] / self.voxel_size)
                            voxel_coord = (ix, iy, iz)
                            
                            if voxel_coord not in self.mapped_voxels:
                                self.mapped_voxels.add(voxel_coord)
                                self.accumulated_points.append(p)
                                valid_triangulated_points.append(p)
                                
                for pt in pts2_2d:
                    cv2.circle(cv_image, (int(pt[0]), int(pt[1])), 4, (0, 255, 0), -1)
                    
        self.filter_outliers_ror()
        self.get_logger().info(f"[Advanced] Reconstructed {len(valid_triangulated_points)} points. Total: {len(self.accumulated_points)}", throttle_duration_sec=1.0)
        
        if self.accumulated_points:
            pc_msg = self.create_pointcloud_msg(header, self.accumulated_points)
            self.pointcloud_pub.publish(pc_msg)
            
        # Update keyframe cache only when significant baseline/rotation is reached or tracking degrades
        if dist > 0.40 or diff_yaw > 0.40 or len(good_matches) < 30:
            self.prev_cam_x = cx_w
            self.prev_cam_y = cy_w
            self.prev_cam_yaw = self.robot_yaw
            self.prev_gray = gray
            self.prev_kp = kp
            self.prev_des = des
            self.prev_P = P_curr

    def run_sliding_window_triangulation(self, cv_image, gray, kp, des, P_curr, cx_w, cy_w, header):
        # We maintain a queue of keyframes in self.sliding_window: max size 2.
        # Each keyframe: {'x': cx_w, 'y': cy_w, 'yaw': yaw, 'kp': kp, 'des': des, 'P': P, 'matches_prev': good_matches_with_prev_keyframe}
        
        # 1. Initialize queue if empty
        if len(self.sliding_window) == 0:
            self.sliding_window.append({
                'x': cx_w, 'y': cy_w, 'yaw': self.robot_yaw,
                'gray': gray, 'kp': kp, 'des': des, 'P': P_curr,
                'matches_prev': None
            })
            return
            
        # 2. If queue has 1 keyframe, check if displacement is large enough to make the 2nd keyframe
        if len(self.sliding_window) == 1:
            kf1 = self.sliding_window[0]
            dist = math.sqrt((cx_w - kf1['x'])**2 + (cy_w - kf1['y'])**2)
            diff_yaw = abs(math.atan2(math.sin(self.robot_yaw - kf1['yaw']), math.cos(self.robot_yaw - kf1['yaw'])))
            
            if dist < 0.15 and diff_yaw < 0.20:
                return
                
            # Perform RANSAC match between kf1 and current frame to store as matches_prev
            good_matches = []
            if kf1['des'] is not None and des is not None and len(kf1['des']) >= 2 and len(des) >= 2:
                try:
                    matches = self.bf.knnMatch(kf1['des'], des, k=2)
                    for m, n in matches:
                        if m.distance < self.match_threshold and m.distance < 0.75 * n.distance:
                            good_matches.append(m)
                except cv2.error:
                    pass
            
            if len(good_matches) >= 8:
                pts1 = np.float32([kf1['kp'][m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                pts2 = np.float32([kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                try:
                    method = getattr(cv2, 'USAC_MAGSAC', cv2.FM_RANSAC)
                    F, mask = cv2.findFundamentalMat(pts1, pts2, method, 3.0)
                    if mask is not None:
                        mask = mask.ravel()
                        good_matches = [good_matches[i] for i in range(len(good_matches)) if mask[i] == 1]
                except cv2.error:
                    pass
                    
            self.sliding_window.append({
                'x': cx_w, 'y': cy_w, 'yaw': self.robot_yaw,
                'gray': gray, 'kp': kp, 'des': des, 'P': P_curr,
                'matches_prev': good_matches
            })
            return
            
        # 3. If queue has 2 keyframes: kf1 (oldest), kf2 (latest)
        kf1 = self.sliding_window[0]
        kf2 = self.sliding_window[1]
        
        # Calculate displacement of current frame from kf2
        dist = math.sqrt((cx_w - kf2['x'])**2 + (cy_w - kf2['y'])**2)
        diff_yaw = abs(math.atan2(math.sin(self.robot_yaw - kf2['yaw']), math.cos(self.robot_yaw - kf2['yaw'])))
        
        # If displacement is too small, skip triangulation but publish accumulated points
        if dist < self.min_baseline and diff_yaw < self.min_rotation:
            if self.accumulated_points:
                pc_msg = self.create_pointcloud_msg(header, self.accumulated_points)
                self.pointcloud_pub.publish(pc_msg)
            return
            
        # Match kf2 -> current_frame using KNN + Lowe's + RANSAC
        matches_23 = []
        if kf2['des'] is not None and des is not None and len(kf2['des']) >= 2 and len(des) >= 2:
            try:
                m_raw = self.bf.knnMatch(kf2['des'], des, k=2)
                for m, n in m_raw:
                    if m.distance < self.match_threshold and m.distance < 0.75 * n.distance:
                        matches_23.append(m)
            except cv2.error:
                pass
                
        if len(matches_23) >= 8:
            pts2 = np.float32([kf2['kp'][m.queryIdx].pt for m in matches_23]).reshape(-1, 1, 2)
            pts3 = np.float32([kp[m.trainIdx].pt for m in matches_23]).reshape(-1, 1, 2)
            try:
                method = getattr(cv2, 'USAC_MAGSAC', cv2.FM_RANSAC)
                F, mask = cv2.findFundamentalMat(pts2, pts3, method, 3.0)
                if mask is not None:
                    mask = mask.ravel()
                    matches_23 = [matches_23[i] for i in range(len(matches_23)) if mask[i] == 1]
            except cv2.error:
                pass
                
        # Connect tracking chain: kf1 -> kf2 -> current
        matches_12 = kf2['matches_prev'] if kf2['matches_prev'] is not None else []
        
        map_12 = {m.queryIdx: m.trainIdx for m in matches_12}
        rev_map_12 = {v: k for k, v in map_12.items()}
        
        tracked_features = []
        for m23 in matches_23:
            idx2 = m23.queryIdx
            idx3 = m23.trainIdx
            if idx2 in rev_map_12:
                idx1 = rev_map_12[idx2]
                tracked_features.append((idx1, idx2, idx3))
                
        valid_triangulated_points = []
        P1 = kf1['P']
        P2 = kf2['P']
        P3 = P_curr
        
        reproj_rejected = 0
        parallax_rejected = 0
        bounds_rejected = 0
        
        for idx1, idx2, idx3 in tracked_features:
            pt1 = kf1['kp'][idx1].pt
            pt2 = kf2['kp'][idx2].pt
            pt3 = kp[idx3].pt
            
            # Construct system of equations AX = 0
            A = np.zeros((6, 4))
            A[0] = pt1[0] * P1[2] - P1[0]
            A[1] = pt1[1] * P1[2] - P1[1]
            
            A[2] = pt2[0] * P2[2] - P2[0]
            A[3] = pt2[1] * P2[2] - P2[1]
            
            A[4] = pt3[0] * P3[2] - P3[0]
            A[5] = pt3[1] * P3[2] - P3[1]
            
            # Solve using SVD
            _, _, Vh = np.linalg.svd(A)
            X = Vh[-1]
            
            if abs(X[3]) > 1e-5:
                p = X[:3] / X[3]
                
                # Check bounds
                dist_to_robot = math.sqrt((p[0] - self.robot_x)**2 + (p[1] - self.robot_y)**2)
                if 0.5 < dist_to_robot < 6.0 and -0.01 <= p[2] <= 1.8:
                    if np.isfinite(p).all():
                        # Reprojection filter on all 3 views (allow 5.0px for SVD compromise)
                        p_homo = np.array([p[0], p[1], p[2], 1.0])
                        errs = []
                        for P_view, pt_view in [(P1, pt1), (P2, pt2), (P3, pt3)]:
                            proj = P_view.dot(p_homo)
                            if abs(proj[2]) > 1e-5:
                                u = proj[0] / proj[2]
                                v = proj[1] / proj[2]
                                errs.append(math.hypot(u - pt_view[0], v - pt_view[1]))
                            else:
                                errs.append(999.0)
                                
                        if any(e > 5.0 for e in errs):
                            reproj_rejected += 1
                            continue
                            
                        # Parallax filter between oldest and newest view
                        C1 = np.array([kf1['x'], kf1['y'], self.camera_height])
                        C3 = np.array([cx_w, cy_w, self.camera_height])
                        v1 = p - C1
                        v3 = p - C3
                        norm_v1 = np.linalg.norm(v1)
                        norm_v3 = np.linalg.norm(v3)
                        if norm_v1 > 1e-5 and norm_v3 > 1e-5:
                            cos_theta = np.dot(v1, v3) / (norm_v1 * norm_v3)
                            cos_theta = np.clip(cos_theta, -1.0, 1.0)
                            parallax_angle = math.acos(cos_theta)
                            if parallax_angle < self.parallax_threshold:
                                parallax_rejected += 1
                                continue
                                
                        ix = int(p[0] / self.voxel_size)
                        iy = int(p[1] / self.voxel_size)
                        iz = int(p[2] / self.voxel_size)
                        voxel_coord = (ix, iy, iz)
                        
                        if voxel_coord not in self.mapped_voxels:
                            self.mapped_voxels.add(voxel_coord)
                            self.accumulated_points.append(p)
                            valid_triangulated_points.append(p)
                else:
                    bounds_rejected += 1
                            
        # Draw tracks on image (magenta circles)
        for idx3 in [idx3 for _, _, idx3 in tracked_features]:
            cv2.circle(cv_image, (int(kp[idx3].pt[0]), int(kp[idx3].pt[1])), 4, (255, 0, 255), -1)
            
        self.filter_outliers_ror()
        self.get_logger().info(
            f"[Sliding Window Debug] matches_12: {len(matches_12)}, matches_23: {len(matches_23)}, "
            f"tracked: {len(tracked_features)} | Reconstructed: {len(valid_triangulated_points)} (Total: {len(self.accumulated_points)}) | "
            f"Rejected: bounds={bounds_rejected}, reproj={reproj_rejected}, parallax={parallax_rejected}",
            throttle_duration_sec=1.0
        )
        
        if self.accumulated_points:
            pc_msg = self.create_pointcloud_msg(header, self.accumulated_points)
            self.pointcloud_pub.publish(pc_msg)
            
        # 4. Check if we should update the keyframe queue (dist > 0.35m or diff_yaw > 0.35rad)
        if dist > 0.35 or diff_yaw > 0.35 or len(matches_23) < 30:
            self.sliding_window.pop(0)
            self.sliding_window.append({
                'x': cx_w, 'y': cy_w, 'yaw': self.robot_yaw,
                'gray': gray, 'kp': kp, 'des': des, 'P': P_curr,
                'matches_prev': matches_23
            })

    def filter_outliers_ror(self):
        """
        Applies Radius Outlier Removal (ROR) to self.accumulated_points.
        Removes points that have fewer than 3 neighbors within 0.30m radius.
        """
        if len(self.accumulated_points) < 4:
            return
            
        pts = np.array(self.accumulated_points)
        
        # Compute pairwise distance matrix in vectorized numpy
        diff = pts[:, np.newaxis, :] - pts[np.newaxis, :, :]
        dists = np.linalg.norm(diff, axis=-1)
        
        # Count neighbors within 0.30m (count includes self, so must be >= 4)
        neighbor_count = np.sum(dists < 0.30, axis=1)
        valid_mask = neighbor_count >= 4
        
        new_points = []
        new_voxels = set()
        for i, p in enumerate(self.accumulated_points):
            if valid_mask[i]:
                new_points.append(p)
                ix = int(p[0] / self.voxel_size)
                iy = int(p[1] / self.voxel_size)
                iz = int(p[2] / self.voxel_size)
                new_voxels.add((ix, iy, iz))
                
        self.accumulated_points = new_points
        self.mapped_voxels = new_voxels

    def create_pointcloud_msg(self, header, points):
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(points)
        
        # Define fields: x, y, z
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack data using struct
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', p[0], p[1], p[2]))
        msg.data = b''.join(buffer)
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = FeatureMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
