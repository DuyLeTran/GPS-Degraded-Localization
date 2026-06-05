import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from vision_msgs.msg import Detection2DArray
import numpy as np
import math
import os

from ev_localization.landmark_db import LandmarkDB

def quat_to_mat(q):
    """Chuyển đổi Quaternion sang Rotation Matrix"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)

def euler_to_mat(roll, pitch, yaw):
    """Chuyển đổi Euler angles sang Rotation Matrix"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    
    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    
    return R_z @ R_y @ R_x

class LandmarkGhostNode(Node):
    def __init__(self):
        super().__init__('landmark_ghost')
        
        # 1. Đọc Parameters
        self.declare_parameter('db_path', 'config/landmarks_sample.json')
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        
        # Offset Camera Frame so với Robot Frame
        self.declare_parameter('cam_x', 0.0)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 0.0)
        # Hệ ROS: Base_link X-forward, Y-left, Z-up
        # Hệ Optical: Z-forward, X-right, Y-down (ví dụ: yaw = -90, roll = -90)
        self.declare_parameter('cam_roll', -1.5708)
        self.declare_parameter('cam_pitch', 0.0)
        self.declare_parameter('cam_yaw', -1.5708)
        
        self.declare_parameter('match_dist_thresh', 100.0) # khoảng cách pixel max để match
        
        db_path = self.get_parameter('db_path').value
        print("")
        # Resolve relative path to absolute path based on package directory
        if not os.path.isabs(db_path):
            pkg_dir = '/home/tranleduy/GPS-Degraded-Localization/ev_localization'
            db_path = os.path.join(pkg_dir, db_path)
        
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        
        cam_x = self.get_parameter('cam_x').value
        cam_y = self.get_parameter('cam_y').value
        cam_z = self.get_parameter('cam_z').value
        cam_roll = self.get_parameter('cam_roll').value
        cam_pitch = self.get_parameter('cam_pitch').value
        cam_yaw = self.get_parameter('cam_yaw').value
        
        self.match_dist_thresh = self.get_parameter('match_dist_thresh').value
        
        # 2. Khởi tạo Landmark Database
        self.db = LandmarkDB()
        try:
            self.db.load(db_path)
            self.get_logger().info(f"Loaded {len(self.db.landmarks)} landmarks từ {db_path}.")
        except Exception as e:
            self.get_logger().error(f"Lỗi khi nạp DB từ {db_path}: {e}")
            
        # Ma trận transform từ Base Link sang Camera Frame (T_bc)
        self.T_bc = np.eye(4)
        self.T_bc[:3, :3] = euler_to_mat(cam_roll, cam_pitch, cam_yaw)
        self.T_bc[:3, 3] = [cam_x, cam_y, cam_z]
        
        self.latest_pose = None
        
        # Landmark Recall Stats
        self.total_visible = 0
        self.total_matched = 0
        self.unique_visible = set()
        self.unique_matched = set()
        self.stats_timer = self.create_timer(10.0, self.print_stats_cb)
        
        # 3. Subscribers
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/ekf/pose', self.pose_cb, 10)
        self.det_sub = self.create_subscription(Detection2DArray, '/detection/bboxes', self.detection_cb, 10)
        
        # 4. Publisher Reprojection Error (Δu, Δv)
        self.err_pub = self.create_publisher(PoseWithCovarianceStamped, '/landmark/reprojection_error', 10)
        
        self.get_logger().info("Landmark Ghost Node started.")

    def pose_cb(self, msg: PoseStamped):
        """Lưu state Pose hiện tại từ EKF Predict"""
        self.latest_pose = msg.pose

    def image_cb(self, msg: Image):
        pass # Dùng cho visualization khi cần thiết

    def get_camera_transform(self, pose):
        """Tính toán ma trận T_cw (World to Camera) dựa trên Robot Pose"""
        T_wb = np.eye(4)
        T_wb[:3, :3] = quat_to_mat(pose.orientation)
        T_wb[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        
        # T_wc (World -> Camera) = T_wb (World -> Base) @ T_bc (Base -> Camera)
        T_wc = T_wb @ self.T_bc
        # T_cw là nghịch đảo của T_wc
        T_cw = np.linalg.inv(T_wc)
        
        return T_cw

    def detection_cb(self, msg: Detection2DArray):
        """Khi có bbox detection mởi, tính projection & matching"""
        if self.latest_pose is None:
            return
            
        pose = self.latest_pose
        x_w, y_w = pose.position.x, pose.position.y
        
        # (1) Lọc landmark gần EKF Predict trong bán kính 40m
        nearby_lms = self.db.query_nearby(x_w, y_w, radius=40.0)
        if not nearby_lms:
            return
            
        T_cw = self.get_camera_transform(pose)
        
        # (2 & 3) Chuyển frame World -> Camera & Pinhole Model
        ghosts = []
        for lm in nearby_lms:
            p_w = np.append(lm.p3d, 1.0)
            p_c = T_cw @ p_w
            X, Y, Z = p_c[:3]
            
            # Kiểm tra object ở phía trước camera và nằm trong tầm nhìn hiệu dụng (35m)
            if 0.1 < Z < 35.0:
                u = self.fx * X / Z + self.cx
                v = self.fy * Y / Z + self.cy
                # Giới hạn landmark nằm trong khung hình thực tế
                if 0.0 <= u <= 2.0 * self.cx and 0.0 <= v <= 2.0 * self.cy:
                    ghosts.append((lm, lm.cls, u, v))
                
        if not ghosts:
            return
            
        # Tăng biến đếm landmarks khả dụng
        self.total_visible += len(ghosts)
        matched_ghosts_this_frame = set()
        
        # Track unique visible landmarks
        for lm, _, _, _ in ghosts:
            self.unique_visible.add(lm.id)
            
        # (4) Match ghost projection với detection thực tế
        for det in msg.detections:
            if not det.results:
                continue
                
            det_cls = det.results[0].hypothesis.class_id
            u_det = det.bbox.center.position.x
            v_det = det.bbox.center.position.y
            
            best_dist = float('inf')
            best_match = None
            best_lm = None
            
            for g_lm, g_cls, g_u, g_v in ghosts:
                # Semantic class matching
                if g_cls == det_cls:
                    dist = math.hypot(g_u - u_det, g_v - v_det)
                    if dist < best_dist:
                        best_dist = dist
                        best_match = (g_u, g_v)
                        best_lm = g_lm
                        
            # (5) Tính reprojection error (Δu, Δv) và Publish
            if best_match is not None and best_dist < self.match_dist_thresh:
                du = u_det - best_match[0]
                dv = v_det - best_match[1]
                
                err_msg = PoseWithCovarianceStamped()
                err_msg.header = msg.header
                
                # Assign error (Δu, Δv) to position.x and position.y
                err_msg.pose.pose.position.x = float(du)
                err_msg.pose.pose.position.y = float(dv)
                err_msg.pose.pose.position.z = 0.0
                
                # Pack landmark 3D position vào orientation fields (convention)
                err_msg.pose.pose.orientation.x = float(best_lm.p3d[0])  # Lx
                err_msg.pose.pose.orientation.y = float(best_lm.p3d[1])  # Ly
                err_msg.pose.pose.orientation.z = float(best_lm.p3d[2])  # Lz
                err_msg.pose.pose.orientation.w = float(det.results[0].hypothesis.score)
                
                self.err_pub.publish(err_msg)
                matched_ghosts_this_frame.add(best_lm.id)
                self.unique_matched.add(best_lm.id)
                
        # Tăng biến đếm landmarks match thành công
        self.total_matched += len(matched_ghosts_this_frame)

    def print_stats_cb(self):
        if self.total_visible > 0:
            recall = (self.total_matched / self.total_visible) * 100.0
            
            num_unique_visible = len(self.unique_visible)
            num_unique_matched = len(self.unique_matched)
            unique_recall = (num_unique_matched / num_unique_visible * 100.0) if num_unique_visible > 0 else 0.0
            
            self.get_logger().info(f"--- LANDMARK RE-ID STATS ---")
            self.get_logger().info(f"Total Visible (in FOV): {self.total_visible}")
            self.get_logger().info(f"Total Matched: {self.total_matched}")
            self.get_logger().info(f"Recall Rate (cumulative): {recall:.2f}%")
            self.get_logger().info(f"Unique Landmarks Visible: {num_unique_visible}")
            self.get_logger().info(f"Unique Landmarks Matched: {num_unique_matched}")
            self.get_logger().info(f"Unique Landmark Recall Rate: {unique_recall:.2f}%")
            self.get_logger().info(f"-----------------------------")
        else:
            self.get_logger().info("--- LANDMARK RE-ID STATS: No visible landmarks in FOV yet. ---")

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkGhostNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
