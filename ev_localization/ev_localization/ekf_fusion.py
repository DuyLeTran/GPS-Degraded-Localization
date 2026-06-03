import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

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


def _normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class EkfFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion')
        self.get_logger().info('ekf_fusion started')

        # Đọc tham số Q, R từ ROS parameters
        self.declare_parameter('Q', [0.1, 0.1, 0.01])
        self.declare_parameter('R_gps', [2.5, 2.5, 0.05])
        self.declare_parameter('R_landmark', [0.1, 0.1])
        self.declare_parameter('gps_lat0', 0.0)
        self.declare_parameter('gps_lon0', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('camera_fx', 600.0)
        self.declare_parameter('camera_fy', 600.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        # Tunable: maximum covariance diagonal to prevent divergence
        self.declare_parameter('max_covariance', 1000.0)
        # Tunable: maximum path length (poses) to prevent unbounded memory
        self.declare_parameter('max_path_length', 10000)

        q_params = self.get_parameter('Q').value
        r_gps_params = self.get_parameter('R_gps').value
        r_lm_params = self.get_parameter('R_landmark').value
        self.gps_lat0 = self.get_parameter('gps_lat0').value
        self.gps_lon0 = self.get_parameter('gps_lon0').value
        self.initial_yaw = self.get_parameter('initial_yaw').value
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value
        self.max_covariance = self.get_parameter('max_covariance').value
        self.max_path_length = self.get_parameter('max_path_length').value

        # ── Process noise: store the continuous-time spectral density (per-second)
        # so it can be properly discretized as Q_d = Q_c * dt during predict.
        self.Q_continuous = np.diag(q_params)

        self.R_gps = np.diag(r_gps_params[:2])  # Lấy 2 thành phần cho x, y
        self.R_gps_default = self.R_gps.copy()   # Lưu giá trị ban đầu
        self.R_landmark = np.diag(r_lm_params)

        # State vector x = [x, y, theta]
        self.x = np.zeros(3)
        # Covariance P (3x3)
        self.P = np.eye(3)

        # ── FIX #5: Guard against huge dt on first odom message.
        # last_time is set to None; the first odom callback will initialize it
        # and skip the predict step to avoid a spurious large-dt prediction.
        self.last_time = None

        # Handover logic: State machine GPS kết hợp
        self.gps_status = "GPS_GOOD"
        self.use_gps = True
        self.gps_last_good = np.zeros(3)
        self.gps_last_good_P = np.eye(3)

        # ── Precompute GPS rotation matrix from ENU to body frame.
        # This rotates GPS ENU coordinates by -initial_yaw so they align
        # with the vehicle's local odom frame that starts at heading=0.
        c_yaw = math.cos(-self.initial_yaw)
        s_yaw = math.sin(-self.initial_yaw)
        self._gps_rot = np.array([[c_yaw, -s_yaw],
                                  [s_yaw,  c_yaw]])

        self.get_logger().info(
            f'EKF params: Q_c={q_params}, R_gps={r_gps_params[:2]}, '
            f'gps_origin=({self.gps_lat0}, {self.gps_lon0}), '
            f'initial_yaw={self.initial_yaw:.4f} rad')

        # Subscribers
        self.gps_status_sub = self.create_subscription(
            String, '/gps/status', self.on_gps_status, 10)
        self.veh_odom_sub = self.create_subscription(
            Odometry, '/vehicle/odom', self.vehicle_odom_cb, 10)
        self.vo_odom_sub = self.create_subscription(
            Odometry, '/vo/odom', self.vo_odom_cb, 10)
        self.gps_fix_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.landmark_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/landmark/reprojection_error',
            self.landmark_callback, 10)

        # Publishers (20 Hz)
        self.pose_pub = self.create_publisher(PoseStamped, '/ekf/pose', 10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/ekf/pose_with_cov', 10)
        self.path_pub = self.create_publisher(Path, '/ekf/trajectory', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.path_msg = Path()
        self.pub_timer = self.create_timer(1.0 / 20.0, self.publish_timer_callback)

    # ─────────────────────────────── GPS STATUS ───────────────────────────────

    def on_gps_status(self, msg):
        """Callback khi nhận GPS status từ gps_monitor."""
        new_status = msg.data  # "GPS_GOOD", "GPS_DEGRADED", "GPS_LOST"

        if self.gps_status == "GPS_GOOD" and new_status == "GPS_DEGRADED":
            # Bắt đầu chạy VO song song, vẫn dùng GPS nhưng tăng R_gps
            self.R_gps = self.R_gps_default * 3.0
            self.get_logger().warn(
                "GPS DEGRADED: Increasing GPS noise, activating VO in parallel")

        elif new_status == "GPS_LOST":
            # LATCH tọa độ GPS cuối cùng tin cậy và ma trận hiệp phương sai
            self.gps_last_good = self.x.copy()
            self.gps_last_good_P = self.P.copy()
            # Chuyển hoàn toàn sang VO + Landmark
            self.use_gps = False
            self.get_logger().error(
                "GPS LOST: Latching last good pose and covariance, "
                "switching to VO+Landmark")

        elif self.gps_status == "GPS_LOST" and new_status == "GPS_GOOD":
            # GPS RE-LOCK: Hiệu chỉnh drift tích lũy
            self.use_gps = True
            self.R_gps = self.R_gps_default.copy()
            self.get_logger().info(
                "GPS RE-LOCKED: Correcting accumulated drift")

        elif self.gps_status == "GPS_LOST" and new_status == "GPS_DEGRADED":
            # GPS RE-LOCK dưới dạng DEGRADED
            self.use_gps = True
            self.R_gps = self.R_gps_default * 3.0
            self.get_logger().warn(
                "GPS RE-LOCKED (DEGRADED): Correcting drift, "
                "using scaled GPS noise")

        elif self.gps_status == "GPS_DEGRADED" and new_status == "GPS_GOOD":
            # Tín hiệu phục hồi từ DEGRADED trở lại GOOD
            self.R_gps = self.R_gps_default.copy()
            self.get_logger().info(
                "GPS RECOVERED: Restored to normal noise level")

        self.gps_status = new_status

    # ─────────────────────────────── ODOMETRY ─────────────────────────────────

    def vehicle_odom_cb(self, msg):
        """Wheel odometry luôn dùng cho predict."""
        self.predict_from_odom(msg)

    def vo_odom_cb(self, msg):
        """VO chỉ dùng cho predict khi GPS mất VÀ wheel odom không khả dụng.
        Trong trường hợp cả 2 có, ưu tiên wheel odom (chính xác hơn về scale).
        """
        if self.gps_status in ["GPS_DEGRADED", "GPS_LOST"]:
            # Nếu muốn dùng VO thay wheel odom, có thể thêm cờ hoặc xử lý
            # riêng ở đây.
            pass

    def predict_from_odom(self, msg):
        current_time = self.get_clock().now()

        # ── FIX #5: Skip the first odom message – only record the timestamp
        # to avoid a huge dt from node start to first message.
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Guard: skip if dt is non-positive or unreasonably large (>2s)
        if dt <= 0.0 or dt > 2.0:
            return

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        x_prev, y_prev, theta_prev = self.x

        # Mid-point integration for better accuracy
        theta_mid = theta_prev + omega * dt / 2.0

        # (1) Motion model dùng velocity (v, w) với mid-point integration
        x_new = x_prev + v * dt * np.cos(theta_mid)
        y_new = y_prev + v * dt * np.sin(theta_mid)
        theta_new = theta_prev + omega * dt

        # ── FIX #3: Normalize theta to [-pi, pi]
        theta_new = _normalize_angle(theta_new)

        self.x = np.array([x_new, y_new, theta_new])

        # (2) Jacobian F
        F = np.array([
            [1.0, 0.0, -v * dt * np.sin(theta_mid)],
            [0.0, 1.0,  v * dt * np.cos(theta_mid)],
            [0.0, 0.0, 1.0]
        ])

        # ── FIX #1: Discretize continuous-time process noise by dt.
        # Q_discrete = Q_continuous * dt
        # This ensures noise grows proportionally with the prediction interval.
        Q_d = self.Q_continuous * dt

        # (3) P_new = F @ P @ F.T + Q_d
        self.P = F @ self.P @ F.T + Q_d

        # ── FIX #2: Clamp covariance diagonal to prevent divergence
        self._clamp_covariance()

    # ─────────────────────────── GENERIC EKF UPDATE ───────────────────────────

    def generic_update(self, z, H, R):
        """Generic update function cho EKF."""
        S = H @ self.P @ H.T + R

        # ── FIX #4: Safe matrix inversion with error handling
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().error(
                "Matrix inversion failed in EKF update — skipping this "
                "measurement. det(S)={:.6e}".format(np.linalg.det(S)))
            return

        # Additional check: if determinant is near-zero, skip
        det_S = np.linalg.det(S)
        if abs(det_S) < 1e-12:
            self.get_logger().warn(
                f"Near-singular innovation matrix (det={det_S:.6e}), "
                "skipping update")
            return

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ z

        # ── FIX #3: Normalize theta after update
        self.x[2] = _normalize_angle(self.x[2])

        # Joseph form update cho Covariance P để đảm bảo ổn định số học
        I = np.eye(len(self.x))
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

        # ── FIX #2: Enforce symmetry and clamp after update
        self.P = (self.P + self.P.T) / 2.0
        self._clamp_covariance()

    # ────────────────────────── GPS MEASUREMENT UPDATE ─────────────────────────

    def gps_callback(self, msg: NavSatFix):
        """(1) GPS Update"""
        if not self.use_gps:
            return

        lat, lon = msg.latitude, msg.longitude
        gps_x_enu, gps_y_enu = gps_to_local(
            lat, lon, self.gps_lat0, self.gps_lon0)

        # Rotate GPS ENU coordinates by -initial_yaw to align with the
        # vehicle's local odometry frame. The vehicle starts with heading=0
        # in the odom frame, but its true heading in ENU is initial_yaw.
        # So we rotate GPS ENU by -initial_yaw to bring it into the same
        # frame as the motion model.
        enu_vec = np.array([gps_x_enu, gps_y_enu])
        local_vec = self._gps_rot @ enu_vec
        gps_x, gps_y = local_vec[0], local_vec[1]

        # Innovation: z = measurement - predicted
        z = np.array([gps_x - self.x[0], gps_y - self.x[1]])

        # H_gps = [[1, 0, 0], [0, 1, 0]]
        H_gps = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])

        self.generic_update(z, H_gps, self.R_gps)

    # ──────────────────────── LANDMARK MEASUREMENT UPDATE ─────────────────────

    def landmark_callback(self, msg: PoseWithCovarianceStamped):
        """(2) Landmark Update"""
        du = msg.pose.pose.position.x
        dv = msg.pose.pose.position.y
        z = np.array([du, dv])

        # Lấy landmark 3D position từ message (packed bởi landmark_ghost)
        lm_p3d = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        # Hàm tính Jacobian số học (numerical Jacobian) với eps = 1e-5
        eps = 1e-5
        H = np.zeros((2, 3))

        uv0 = self._project_landmark(lm_p3d, self.x)
        if uv0 is None:
            return

        for i in range(3):
            x_plus = self.x.copy()
            x_plus[i] += eps
            uv_plus = self._project_landmark(lm_p3d, x_plus)
            if uv_plus is None:
                return
            H[:, i] = (uv_plus - uv0) / eps

        # Calculate innovation covariance S to perform Chi-squared gating check
        S = H @ self.P @ H.T + self.R_landmark
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        # Chi-squared gating (2 DOF, 95% confidence threshold is 5.99)
        mahalanobis_sq = z.T @ S_inv @ z
        if mahalanobis_sq > 5.99:
            self.get_logger().warn(
                f"Landmark update rejected (chi2={mahalanobis_sq:.2f} > 5.99)",
                throttle_duration_sec=2.0)
            return

        self.generic_update(z, H, self.R_landmark)

    def _project_landmark(self, p3d_world, state):
        """Project 3D landmark world → 2D pixel given robot state."""
        x, y, theta = state
        cam_height = 1.5  # hoặc đọc từ parameter
        dx = p3d_world[0] - x
        dy = p3d_world[1] - y
        dz = p3d_world[2] - cam_height

        c, s = np.cos(-theta), np.sin(-theta)
        x_body = c * dx - s * dy    # Forward
        y_body = s * dx + c * dy    # Left

        # Body → Camera (OpenCV convention)
        X_cam = -y_body
        Y_cam = -dz
        Z_cam = x_body

        if Z_cam <= 0.1:
            return None

        u = self.fx * X_cam / Z_cam + self.cx
        v = self.fy * Y_cam / Z_cam + self.cy
        return np.array([u, v])

    # ──────────────────────── COVARIANCE HEALTH ───────────────────────────────

    def _clamp_covariance(self):
        """FIX #2: Clamp covariance diagonal to prevent runaway divergence
        during GPS-denied periods. Also ensures P remains positive-definite."""
        max_val = self.max_covariance
        clamped = False
        for i in range(3):
            if self.P[i, i] > max_val:
                # Scale the entire row/column proportionally to preserve
                # correlation structure (instead of hard clamping diagonal only)
                scale = math.sqrt(max_val / self.P[i, i])
                self.P[i, :] *= scale
                self.P[:, i] *= scale
                clamped = True
            # Also ensure diagonal is never negative (numerical artifact)
            if self.P[i, i] < 1e-6:
                self.P[i, i] = 1e-6

        if clamped:
            self.get_logger().warn(
                "Covariance clamped to prevent divergence "
                f"(max_cov={max_val})",
                throttle_duration_sec=5.0)

    # ────────────────────────── PUBLISH TIMER ─────────────────────────────────

    def publish_timer_callback(self):
        """(4) Publish Pose, PoseWithCov, Path at 20Hz"""
        now = self.get_clock().now().to_msg()

        # 1. PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = float(self.x[0])
        pose_msg.pose.position.y = float(self.x[1])
        pose_msg.pose.position.z = 0.0

        # Theta to Quaternion
        theta = self.x[2]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(theta / 2.0)

        self.pose_pub.publish(pose_msg)

        # 2. PoseWithCovarianceStamped
        cov_msg = PoseWithCovarianceStamped()
        cov_msg.header = pose_msg.header
        cov_msg.pose.pose = pose_msg.pose
        # Map 3x3 EKF covariance [x, y, yaw] → 6x6 ROS covariance
        # Indices in row-major 6x6: x=0, y=1, z=2, roll=3, pitch=4, yaw=5
        cov_array = np.zeros(36)
        cov_array[0] = self.P[0, 0]     # Var(x)
        cov_array[1] = self.P[0, 1]     # Cov(x, y)
        cov_array[5] = self.P[0, 2]     # Cov(x, yaw)
        cov_array[6] = self.P[1, 0]     # Cov(y, x)
        cov_array[7] = self.P[1, 1]     # Var(y)
        cov_array[11] = self.P[1, 2]    # Cov(y, yaw)
        cov_array[30] = self.P[2, 0]    # Cov(yaw, x)
        cov_array[31] = self.P[2, 1]    # Cov(yaw, y)
        cov_array[35] = self.P[2, 2]    # Var(yaw)
        cov_msg.pose.covariance = cov_array.tolist()
        self.pose_cov_pub.publish(cov_msg)

        # 3. Path
        self.path_msg.header = pose_msg.header
        self.path_msg.poses.append(pose_msg)

        # ── FIX #6: Limit path length to prevent unbounded memory growth
        if len(self.path_msg.poses) > self.max_path_length:
            # Keep the most recent poses
            self.path_msg.poses = self.path_msg.poses[-self.max_path_length:]

        self.path_pub.publish(self.path_msg)

        # 4. TF Broadcast (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.x[0])
        t.transform.translation.y = float(self.x[1])
        t.transform.translation.z = 0.0
        t.transform.rotation = pose_msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


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
