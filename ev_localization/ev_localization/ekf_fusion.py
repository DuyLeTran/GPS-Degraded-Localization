import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String

# Import utility (Giả sử có module biến đổi toạ độ GPS)
try:
    from ev_localization.utils.geo_utils import gps_to_local
except ImportError:
    # Định nghĩa tạm nếu không load được
    def gps_to_local(lat, lon, ref_lat, ref_lon):
        R = 6378137.0
        lat_rad, lon_rad = math.radians(lat), math.radians(lon)
        ref_lat_rad, ref_lon_rad = math.radians(ref_lat), math.radians(ref_lon)
        dlat, dlon = lat_rad - ref_lat_rad, lon_rad - ref_lon_rad
        x = R * dlon * math.cos(ref_lat_rad)
        y = R * dlat
        return x, y

def euler_from_quaternion(q):
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

class EkfFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion')
        self.get_logger().info('ekf_fusion started')
        
        # Đọc tham số Q, R từ ROS parameters
        self.declare_parameter('Q', [0.1, 0.1, 0.01])
        self.declare_parameter('R_gps', [0.5, 0.5, 0.05])
        self.declare_parameter('R_landmark', [0.1, 0.1])
        self.declare_parameter('gps_lat0', 0.0)
        self.declare_parameter('gps_lon0', 0.0)
        
        q_params = self.get_parameter('Q').value
        r_gps_params = self.get_parameter('R_gps').value
        r_lm_params = self.get_parameter('R_landmark').value
        self.gps_lat0 = self.get_parameter('gps_lat0').value
        self.gps_lon0 = self.get_parameter('gps_lon0').value
        
        self.Q = np.diag(q_params)
        self.R_gps = np.diag(r_gps_params[:2])  # Lấy 2 thành phần cho x, y
        self.R_gps_default = self.R_gps.copy()  # Lưu giá trị ban đầu
        self.R_landmark = np.diag(r_lm_params)
        
        # State vector x = [x, y, theta]
        self.x = np.zeros(3)
        # Covariance P (3x3)
        self.P = np.eye(3)
        
        self.last_time = self.get_clock().now()
        
        # Handover logic: State machine GPS kết hợp
        self.gps_status = "GPS_GOOD"
        self.use_gps = True
        self.gps_last_good = np.zeros(3)
        self.gps_last_good_P = np.eye(3)
        
        # Subscribers
        self.gps_status_sub = self.create_subscription(String, '/gps/status', self.on_gps_status, 10)
        self.veh_odom_sub = self.create_subscription(Odometry, '/vehicle/odom', self.predict_callback, 10)
        self.vo_odom_sub = self.create_subscription(Odometry, '/vo/odom', self.predict_callback, 10)
        self.gps_fix_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.landmark_sub = self.create_subscription(PoseWithCovarianceStamped, '/landmark/reprojection_error', self.landmark_callback, 10)
        
        # Publishers (20 Hz)
        self.pose_pub = self.create_publisher(PoseStamped, '/ekf/pose', 10)
        self.pose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf/pose_with_cov', 10)
        self.path_pub = self.create_publisher(Path, '/ekf/trajectory', 10)
        
        self.path_msg = Path()
        self.pub_timer = self.create_timer(1.0 / 20.0, self.publish_timer_callback)

    def on_gps_status(self, msg):
        """Callback khi nhận GPS status từ gps_monitor."""
        new_status = msg.data  # "GPS_GOOD", "GPS_DEGRADED", "GPS_LOST"

        if self.gps_status == "GPS_GOOD" and new_status == "GPS_DEGRADED":
            # Bắt đầu chạy VO song song, vẫn dùng GPS nhưng tăng R_gps (giảm độ tin cậy)
            self.R_gps = self.R_gps_default * 3.0  # Tăng noise GPS
            self.get_logger().warn("GPS DEGRADED: Increasing GPS noise, activating VO in parallel")

        elif new_status == "GPS_LOST":
            # LATCH tọa độ GPS cuối cùng tin cậy và ma trận hiệp phương sai
            self.gps_last_good = self.x.copy()
            self.gps_last_good_P = self.P.copy()
            # Chuyển hoàn toàn sang VO + Landmark
            self.use_gps = False
            self.get_logger().error("GPS LOST: Latching last good pose and covariance, switching to VO+Landmark")

        elif self.gps_status == "GPS_LOST" and new_status == "GPS_GOOD":
            # GPS RE-LOCK: Hiệu chỉnh drift tích lũy
            self.use_gps = True
            # Reset R_gps về giá trị ban đầu
            self.R_gps = self.R_gps_default.copy()
            # Drift correction sẽ xảy ra tự nhiên qua GPS update step
            self.get_logger().info("GPS RE-LOCKED: Correcting accumulated drift")

        elif self.gps_status == "GPS_DEGRADED" and new_status == "GPS_GOOD":
            # Tín hiệu phục hồi từ DEGRADED trở lại GOOD
            self.R_gps = self.R_gps_default.copy()
            self.get_logger().info("GPS RECOVERED: Restored to normal noise level")

        self.gps_status = new_status

    def predict_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
            
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        x_prev, y_prev, theta_prev = self.x
        
        theta_mid = theta_prev + omega * dt / 2.0
        
        # (1) Motion model dùng velocity (v, w) với mid-point integration
        x_new = x_prev + v * dt * np.cos(theta_mid)
        y_new = y_prev + v * dt * np.sin(theta_mid)
        theta_new = theta_prev + omega * dt
        
        self.x = np.array([x_new, y_new, theta_new])
        
        # (2) Jacobian F
        F = np.array([
            [1.0, 0.0, -v * dt * np.sin(theta_mid)],
            [0.0, 1.0,  v * dt * np.cos(theta_mid)],
            [0.0, 0.0, 1.0]
        ])
        
        # (3) P_new = F @ P @ F.T + Q
        self.P = F @ self.P @ F.T + self.Q

    def generic_update(self, z, H, R):
        """ (3) Generic update function cho EKF """
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ z
        
        # Joseph form update cho Covariance P để đảm bảo ổn định
        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T

    def gps_callback(self, msg: NavSatFix):
        """ (1) GPS Update """
        if not self.use_gps:
            return
            
        lat, lon = msg.latitude, msg.longitude
        gps_x, gps_y = gps_to_local(lat, lon, self.gps_lat0, self.gps_lon0)
        
        # z = [gps_x - x, gps_y - y]
        z = np.array([gps_x - self.x[0], gps_y - self.x[1]])
        
        # H_gps = [[1, 0, 0], [0, 1, 0]]
        H_gps = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])
        
        self.generic_update(z, H_gps, self.R_gps)

    def landmark_callback(self, msg: PoseWithCovarianceStamped):
        """ (2) Landmark Update """
        du = msg.pose.pose.position.x
        dv = msg.pose.pose.position.y
        z = np.array([du, dv])
        
        # Hàm tính Jacobian số học (numerical Jacobian) với eps = 1e-5
        eps = 1e-5
        H = np.zeros((2, 3))
        
        # Do EKF Fusion không có toạ độ Landmark 3D cụ thể (do Ghost truyền dạng residual),
        # hàm projection thực tế h(X) sẽ kết hợp từ Ghost projection. 
        # Tuy nhiên H có thể được xấp xỉ cục bộ thông qua sự thay đổi Residual theo Pose.
        # Ở đây ta giả sử hàm giả h(x,y,theta) cho quá trình projection
        def dummy_h_proj(state):
            # Hàm dummy thay the cho Projection thật do thieu thong tin Landmark 3D,
            # Nếu có thông tin Landmark, project_3d_to_2d(lm_3d_world, state)
            # Tại local point, ta giả định có 1 điểm tham chiếu ảo phía trước
            x, y, theta = state
            return np.array([x * 100.0, y * 100.0])
            
        for i in range(3):
            state_plus = self.x.copy()
            state_plus[i] += eps
            state_minus = self.x.copy()
            state_minus[i] -= eps
            H[:, i] = (dummy_h_proj(state_plus) - dummy_h_proj(state_minus)) / (2 * eps)
            
        self.generic_update(z, H, self.R_landmark)

    def publish_timer_callback(self):
        """ (4) Publish Pose, PoseWithCov, Path at 20Hz """
        now = self.get_clock().now().to_msg()
        
        # 1. PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = self.x[0]
        pose_msg.pose.position.y = self.x[1]
        pose_msg.pose.position.z = 0.0
        
        # Theta to Quat
        theta = self.x[2]
        pose_msg.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(theta / 2.0)
        
        self.pose_pub.publish(pose_msg)
        
        # 2. PoseWithCovarianceStamped
        cov_msg = PoseWithCovarianceStamped()
        cov_msg.header = pose_msg.header
        cov_msg.pose.pose = pose_msg.pose
        cov_array = np.zeros(36)
        cov_array[0] = self.P[0, 0] # x
        cov_array[1] = self.P[0, 1]
        cov_array[5] = self.P[0, 2] # x-yaw
        cov_array[6] = self.P[1, 0]
        cov_array[7] = self.P[1, 1] # y
        cov_array[11] = self.P[1, 2]
        cov_array[30] = self.P[2, 0]
        cov_array[31] = self.P[2, 1]
        cov_array[35] = self.P[2, 2] # yaw
        cov_msg.pose.covariance = cov_array.tolist()
        self.pose_cov_pub.publish(cov_msg)
        
        # 3. Path
        self.path_msg.header = pose_msg.header
        self.path_msg.poses.append(pose_msg)
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EkfFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
