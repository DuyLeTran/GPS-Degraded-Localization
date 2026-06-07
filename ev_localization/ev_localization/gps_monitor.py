import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from enum import Enum

class GpsState(Enum):
    GPS_GOOD = "GPS_GOOD"
    GPS_DEGRADED = "GPS_DEGRADED"
    GPS_LOST = "GPS_LOST"

class GpsMonitorNode(Node):
    def __init__(self):
        super().__init__('gps_monitor')
        
        # Khai báo và nạp tham số
        self.declare_parameter('hdop_degraded_threshold', 5.0)
        self.declare_parameter('hdop_lost_threshold', 20.0)
        self.declare_parameter('hdop_good_threshold', 3.0)
        self.declare_parameter('sats_degraded_threshold', 4)
        self.declare_parameter('sats_good_threshold', 6)
        self.declare_parameter('hysteresis_duration_sec', 2.0)
        self.declare_parameter('timeout_sec', 10.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('hysteresis_recover_sec', 3.0)
        self.declare_parameter('hysteresis_relock_sec', 5.0)
        
        self.hdop_deg_th = self.get_parameter('hdop_degraded_threshold').value
        self.hdop_lost_th = self.get_parameter('hdop_lost_threshold').value
        self.hdop_good_th = self.get_parameter('hdop_good_threshold').value
        self.sats_deg_th = self.get_parameter('sats_degraded_threshold').value
        self.sats_good_th = self.get_parameter('sats_good_threshold').value
        self.hysteresis_dur = self.get_parameter('hysteresis_duration_sec').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.hyst_recover = self.get_parameter('hysteresis_recover_sec').value
        self.hyst_relock = self.get_parameter('hysteresis_relock_sec').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        
        # Biến trạng thái State Machine
        self.current_state = GpsState.GPS_GOOD
        self.target_state = GpsState.GPS_GOOD
        self.state_change_start_time = None
        
        # Biến lưu trữ dữ liệu GPS
        self.last_fix_time = self.get_clock().now()
        self.current_hdop = float('inf')
        self.current_sats = 0
        
        # Publisher và Subscriber
        self.status_pub = self.create_publisher(String, '/gps/status', 10)
        self.fix_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.fix_callback,
            10
        )
        
        # Timer (10 Hz)
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('GPS Monitor Node started.')

    def fix_callback(self, msg: NavSatFix):
        self.last_fix_time = self.get_clock().now()
        
        # Trích xuất HDOP từ position_covariance[0] theo yêu cầu
        if len(msg.position_covariance) >= 1:
            self.current_hdop = msg.position_covariance[0]
        else:
            self.current_hdop = float('inf')
            
        # Đếm số vệ tinh (NavSatFix không có field sats chuẩn, 
        # thông thường dev hay mượn trường status.service (uint16) để truyền sats)
        self.current_sats = msg.status.service

    def evaluate_target_state(self):
        """Đánh giá điều kiện chuyển trạng thái dựa trên ngưỡng"""
        # Quay về GOOD
        if self.current_hdop < self.hdop_good_th and self.current_sats >= self.sats_good_th:
            return GpsState.GPS_GOOD
        # DEGRADED -> LOST (do HDOP quá cao)
        elif self.current_hdop > self.hdop_lost_th:
            return GpsState.GPS_LOST
        # GOOD -> DEGRADED
        elif self.current_hdop > self.hdop_deg_th or self.current_sats < self.sats_deg_th:
            return GpsState.GPS_DEGRADED
            
        # Nếu nằm giữa các khoảng (VD: HDOP=4.0, Sats=5), giữ nguyên trạng thái cũ
        return self.current_state

    def timer_callback(self):
        now = self.get_clock().now()
        time_since_last_fix = (now - self.last_fix_time).nanoseconds / 1e9
        
        # 1. Kiểm tra timeout (10s không có fix) -> Chuyển sang LOST ngay lập tức
        if time_since_last_fix > self.timeout_sec:
            if self.current_state != GpsState.GPS_LOST:
                self.get_logger().warn('\033[1;31mGPS timeout (>10s). Transitioning to GPS_LOST!\033[0m')
                self.current_state = GpsState.GPS_LOST
                self.target_state = GpsState.GPS_LOST
                self.state_change_start_time = None
        else:
            # 2. State Machine với Hysteresis
            new_target = self.evaluate_target_state()
            
            # Nếu mục tiêu thay đổi so với lần trước, bắt đầu đếm thời gian
            if new_target != self.target_state:
                self.target_state = new_target
                self.state_change_start_time = now
            
            # Kiểm tra thời gian duy trì (hysteresis)
            if self.target_state != self.current_state and self.state_change_start_time is not None:
                time_in_target = (now - self.state_change_start_time).nanoseconds / 1e9
                
                # Tính required_dur dựa trên hướng chuyển:
                if self.current_state == GpsState.GPS_LOST and self.target_state == GpsState.GPS_GOOD:
                    required_dur = self.hyst_relock        # 5s: LOST→GOOD
                elif self.current_state == GpsState.GPS_DEGRADED and self.target_state == GpsState.GPS_GOOD:
                    required_dur = self.hyst_recover       # 3s: DEGRADED→GOOD
                else:
                    required_dur = self.hysteresis_dur     # 2s: default (GOOD→DEGRADED)
                    
                if time_in_target >= required_dur:
                    color = "\033[1;32m"  # default bold green (for GOOD state recovery)
                    if self.target_state == GpsState.GPS_LOST:
                        color = "\033[1;31m"  # bold red
                    elif self.target_state == GpsState.GPS_DEGRADED:
                        color = "\033[1;33m"  # bold yellow
                    
                    self.get_logger().info(
                        f"{color}State change: {self.current_state.value} -> {self.target_state.value} "
                        f"(HDOP={self.current_hdop:.2f}, Sats={self.current_sats})\033[0m"
                    )
                    self.current_state = self.target_state
                    self.state_change_start_time = None
                    
        # 3. Publish trạng thái ở 10 Hz
        msg = String()
        msg.data = self.current_state.value
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
