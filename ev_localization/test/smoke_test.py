#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class SmokeTestNode(Node):
    def __init__(self):
        super().__init__('smoke_test')
        
        # Publishers
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/vehicle/odom', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/ekf/pose', self.pose_cb, 10)
        self.status_sub = self.create_subscription(String, '/gps/status', self.status_cb, 10)
        
        # Timers
        self.timer_odom = self.create_timer(0.1, self.publish_odom) # 10Hz
        self.timer_gps = self.create_timer(1.0, self.publish_gps)   # 1Hz
        
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('Smoke Test started: 10s GPS GOOD -> 10s GPS LOST -> 10s GPS GOOD')

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        # Xe di chuyển 5 m/s thẳng
        msg.twist.twist.linear.x = 5.0
        msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(msg)

    def publish_gps(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        # 10s GOOD, 10s LOST, 10s GOOD
        if 10.0 < elapsed <= 20.0:
            # GPS Lost: Không publish gì cả
            pass
        elif elapsed > 30.0:
            self.get_logger().info('Smoke test finished. Bấm Ctrl+C để thoát.')
            # Shutdown tự động thì phải cẩn thận với thread con. Ở đây in ra console để thoát thủ công hoặc shutdown node
            rclpy.shutdown()
        else:
            # GPS Good
            msg = NavSatFix()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'gps_link'
            msg.latitude = 21.0
            msg.longitude = 105.0
            msg.altitude = 0.0
            # Giả lập mảng covariance dài 9, vị trí index 0 để encode HDOP
            msg.position_covariance = [0.0] * 9
            msg.position_covariance[0] = 1.0 # HDOP < 3.0 -> GOOD
            msg.status.service = 8 # 8 satellites -> GOOD
            self.gps_pub.publish(msg)

    def pose_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'[EKF Pose] x: {x:.2f}, y: {y:.2f}')

    def status_cb(self, msg: String):
        self.get_logger().info(f'[GPS Status] {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SmokeTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()

if __name__ == '__main__':
    main()