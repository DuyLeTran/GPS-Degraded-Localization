import numpy as np
import math
from typing import List, Tuple, Dict, Optional

def quat_to_mat(q) -> np.ndarray:
    """Chuyển đổi Quaternion sang Rotation Matrix (3x3)"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)

def euler_to_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Chuyển đổi Euler angles (Roll, Pitch, Yaw) sang Rotation Matrix (3x3)"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    
    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    
    return R_z @ R_y @ R_x

def get_camera_transform(pose, T_bc: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Tính toán R_wc (World to Camera Rotation) và t_wc (World to Camera Translation)
    pose: geometry_msgs/Pose (Base link in World frame)
    T_bc: 4x4 Homogeneous transform matrix (Base link to Camera optical link)
    """
    T_wb = np.eye(4)
    T_wb[:3, :3] = quat_to_mat(pose.orientation)
    T_wb[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    
    T_wc = T_wb @ T_bc
    
    R_wc = T_wc[:3, :3]
    t_wc = T_wc[:3, 3]
    
    return R_wc, t_wc

def select_best_keyframe_pair(history: List[Tuple]) -> Optional[Tuple[Tuple, Tuple]]:
    """
    Chọn cặp frame trong lịch sử quan sát có khoảng cách di chuyển (baseline) lớn nhất.
    history: List of (timestamp, bbox, pose)
    """
    if len(history) < 2:
        return None
        
    best_pair = None
    max_dist = -1.0
    
    # Duyệt qua các cặp để tìm baseline lớn nhất
    for i in range(len(history)):
        pos_i = np.array([history[i][2].position.x, history[i][2].position.y, history[i][2].position.z])
        for j in range(i + 1, len(history)):
            pos_j = np.array([history[j][2].position.x, history[j][2].position.y, history[j][2].position.z])
            dist = np.linalg.norm(pos_i - pos_j)
            if dist > max_dist:
                max_dist = dist
                best_pair = (history[i], history[j])
                
    return best_pair

def triangulate_two_rays(ray_a: np.ndarray, origin_a: np.ndarray,
                         ray_b: np.ndarray, origin_b: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    """
    Giải giao hội 3D giữa 2 tia nhìn bằng cách tìm trung điểm của đoạn thẳng ngắn nhất nối 2 tia.
    ray_a, ray_b: Unit bearing vectors in world frame (length 1)
    origin_a, origin_b: Ray starting positions (camera centers in world frame)
    Returns: Tuple[Point3D, parallax_angle_deg] hoặc None nếu 2 tia song song
    """
    # Vector nối 2 điểm gốc
    dp = origin_a - origin_b
    
    # Cosine góc giữa 2 tia
    c = np.dot(ray_a, ray_b)
    
    # Kiểm tra song song
    if 1.0 - c**2 < 1e-4:
        return None
        
    # Tính tham số t_a và t_b của 2 đường thẳng r(t) = o + t*v
    t_a = (c * np.dot(dp, ray_b) - np.dot(dp, ray_a)) / (1.0 - c**2)
    t_b = c * t_a + np.dot(dp, ray_b)
    
    # Không giao hội các điểm nằm sau lưng camera
    if t_a < 0.1 or t_b < 0.1:
        return None
        
    # Tìm 2 điểm gần nhất trên 2 tia
    p_a = origin_a + t_a * ray_a
    p_b = origin_b + t_b * ray_b
    
    # Trung điểm
    p_3d = (p_a + p_b) / 2.0
    
    # Tính góc Parallax
    parallax_rad = math.acos(np.clip(c, -1.0, 1.0))
    parallax_deg = math.degrees(parallax_rad)
    
    return p_3d, parallax_deg

def depth_from_known_size(cls: str, bbox: List[float], fy: float, fx: float, cx: float, cy: float,
                          R_wc: np.ndarray, t_wc: np.ndarray, known_heights: Dict[str, float]) -> np.ndarray:
    """
    Ước lượng toạ độ 3D ENU bằng giả thuyết kích thước thực tế của đối tượng.
    bbox: [cx_pixel, cy_pixel, w_px, h_px]
    known_heights: Dict các chiều cao trung bình của vật thể
    """
    h_px = bbox[3]
    h_real = known_heights.get(cls.lower(), 1.5) # Default 1.5m (Car)
    
    # Pinhole model: depth = (height_real * focal_y) / height_pixel
    depth = (h_real * fy) / max(h_px, 1e-3)
    
    # Tính toạ độ camera optical frame
    x_cam = (bbox[0] - cx) * depth / fx
    y_cam = (bbox[1] - cy) * depth / fy
    z_cam = depth
    
    # Chuyển sang World ENU
    p_3d = R_wc @ np.array([x_cam, y_cam, z_cam]) + t_wc
    return p_3d

def flat_ground_projection(bbox: List[float], fx: float, fy: float, cx: float, cy: float,
                           R_wc: np.ndarray, t_wc: np.ndarray) -> Optional[np.ndarray]:
    """
    Ước lượng toạ độ 3D ENU bằng phương pháp chiếu điểm chân Bounding Box xuống mặt phẳng đất Z_world = 0.
    bbox: [cx_pixel, cy_pixel, w_px, h_px]
    """
    # Lấy điểm đáy của Bounding Box (cx, cy + h/2) - Điểm tiếp xúc đất
    u_pixel = bbox[0]
    v_pixel = bbox[1] + bbox[3] / 2.0
    
    # Hướng tia trong camera optical frame
    ray_cam = np.array([
        (u_pixel - cx) / fx,
        (v_pixel - cy) / fy,
        1.0
    ])
    
    # Xoay tia sang World ENU frame
    ray_world = R_wc @ ray_cam
    
    # Nếu tia hướng lên trên (v_world_z >= 0), không thể giao với mặt đất Z=0
    if ray_world[2] >= -1e-4:
        return None
        
    # Điểm gốc là camera center
    origin_world = t_wc
    
    # Giải phương trình: origin_z + t * ray_z = 0 -> t = -origin_z / ray_z
    # Giả định mặt đường ở cao độ Z = 0
    t = -origin_world[2] / ray_world[2]
    
    if t < 0.1:
        return None
        
    p_3d = origin_world + t * ray_world
    return p_3d

def sanity_check(p_3d: np.ndarray, vehicle_pos: np.ndarray, heading_yaw: float) -> bool:
    """
    Kiểm tra tính hợp lệ hình học của toạ độ 3D thu được
    """
    # 1. Kiểm tra cự ly (phải từ 2m đến 60m)
    dist = np.linalg.norm(p_3d - vehicle_pos)
    if dist < 2.0 or dist > 60.0:
        return False
        
    # 2. Kiểm tra độ cao (Z phải hợp lý cho biển báo / xe cộ bên đường: từ -1.5m đến 6.0m)
    if p_3d[2] < -1.5 or p_3d[2] > 6.0:
        return False
        
    # 3. Kiểm tra hướng (Vật thể phải nằm phía trước xe)
    # Vector từ xe đến vật thể
    v_to_p = p_3d[:2] - vehicle_pos[:2]
    # Unit heading vector
    heading_vec = np.array([math.cos(heading_yaw), math.sin(heading_yaw)])
    
    # Dot product phải > 0 (góc lệch < 90 độ)
    dot = np.dot(v_to_p, heading_vec)
    if dot <= 0:
        return False
        
    return True
