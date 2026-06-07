import rclpy
from rclpy.node import Node
import math
import collections
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class UTurnDetectorNode(Node):
    def __init__(self):
        super().__init__('uturn_detector')
        self.get_logger().info('uturn_detector started')

        # Khai báo và nạp tham số
        self.declare_parameter('angle_threshold_deg', 150.0)
        self.declare_parameter('time_window_sec', 2.0)
        self.declare_parameter('cooldown_sec', 5.0)

        self.angle_threshold_deg = self.get_parameter('angle_threshold_deg').value
        self.time_window_sec = self.get_parameter('time_window_sec').value
        self.cooldown_sec = self.get_parameter('cooldown_sec').value

        # Chuyển đổi ngưỡng góc sang radian
        self.angle_threshold_rad = math.radians(self.angle_threshold_deg)

        # Subscriber nhận odometry từ xe
        self.odom_sub = self.create_subscription(
            Odometry,
            '/vehicle/odom',
            self.odom_callback,
            10
        )

        # Publisher phát sự kiện U-turn
        self.uturn_pub = self.create_publisher(String, '/vehicle/u_turn_event', 10)

        # Biến trạng thái
        # Lưu lịch sử heading dưới dạng (timestamp_sec, accumulated_theta)
        self.heading_history = collections.deque()
        # Tích phân vận tốc góc tích lũy
        self.accumulated_theta = 0.0
        # Thời điểm odom cuối cùng (dùng để tính dt)
        self.last_time = None
        # Thời điểm trigger U-turn cuối cùng (để kiểm tra cooldown)
        self.last_trigger_time = None

        self.get_logger().info(
            f'UTurn params: angle_threshold={self.angle_threshold_deg}°, '
            f'time_window={self.time_window_sec}s, '
            f'cooldown={self.cooldown_sec}s')

    def odom_callback(self, msg):
        """Callback xử lý odometry: tích phân vận tốc góc và phát hiện U-turn."""
        current_time = self.get_clock().now()

        # Bỏ qua message đầu tiên, chỉ ghi nhận thời gian
        if self.last_time is None:
            self.last_time = current_time
            return

        # Tính khoảng thời gian dt giữa 2 message
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Bỏ qua nếu dt không hợp lệ hoặc quá lớn (> 1s)
        if dt <= 0.0 or dt > 1.0:
            return

        # Trích xuất vận tốc góc quanh trục z
        omega_z = msg.twist.twist.angular.z

        # Tích phân vận tốc góc để tính heading tích lũy
        self.accumulated_theta += omega_z * dt

        # Chuyển thời gian hiện tại sang giây (float)
        now_sec = current_time.nanoseconds / 1e9

        # Thêm vào lịch sử sliding window
        self.heading_history.append((now_sec, self.accumulated_theta))

        # Loại bỏ các entry cũ ngoài cửa sổ thời gian
        while (self.heading_history
               and now_sec - self.heading_history[0][0] > self.time_window_sec):
            self.heading_history.popleft()

        # Cần ít nhất 2 entry để tính delta
        if len(self.heading_history) < 2:
            return

        # Tính thay đổi heading trong cửa sổ trượt
        delta_theta = abs(self.heading_history[-1][1] - self.heading_history[0][1])

        # Kiểm tra điều kiện kích hoạt U-turn
        if delta_theta >= self.angle_threshold_rad:
            # Kiểm tra cooldown: tránh phát trigger liên tục
            if (self.last_trigger_time is not None
                    and now_sec - self.last_trigger_time < self.cooldown_sec):
                return

            # Phát thông báo U-turn
            event_msg = String()
            event_msg.data = 'U_TURN_DETECTED'
            self.uturn_pub.publish(event_msg)

            self.get_logger().warn(
                f'\033[1;36mU-TURN DETECTED! Δθ = {math.degrees(delta_theta):.1f}° '
                f'in {now_sec - self.heading_history[0][0]:.2f}s\033[0m')

            # Cập nhật thời điểm trigger cuối cùng
            self.last_trigger_time = now_sec

            # Reset lịch sử và tích phân sau khi phát hiện
            self.heading_history.clear()
            self.accumulated_theta = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = UTurnDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
