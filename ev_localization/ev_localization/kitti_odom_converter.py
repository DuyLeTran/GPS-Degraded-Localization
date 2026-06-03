import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
import math

# Try importing the common utility
try:
    from ev_localization.utils.geo_utils import gps_to_local
except ImportError:
    def gps_to_local(lat, lon, ref_lat, ref_lon):
        R = 6378137.0
        lat_rad, lon_rad = math.radians(lat), math.radians(lon)
        ref_lat_rad, ref_lon_rad = math.radians(ref_lat), math.radians(ref_lon)
        dlat, dlon = lat_rad - ref_lat_rad, lon_rad - ref_lon_rad
        x = R * dlon * math.cos(ref_lat_rad)
        y = R * dlat
        return x, y

class KittiOdomConverter(Node):
    def __init__(self):
        super().__init__('kitti_odom_converter')
        
        self.declare_parameter('gps_lat0', 49.011758)
        self.declare_parameter('gps_lon0', 8.422249)
        self.gps_lat0 = self.get_parameter('gps_lat0').value
        self.gps_lon0 = self.get_parameter('gps_lon0').value
        
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/vehicle/odom', 10)
        
        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        
        self.latest_yaw = 0.0
        self.latest_omega_z = 0.0
        
        self.get_logger().info("kitti_odom_converter started")

    def imu_cb(self, msg: Imu):
        # Extract yaw from quaternion
        q = msg.orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.latest_yaw = math.atan2(t3, t4)
        self.latest_omega_z = msg.angular_velocity.z

    def gps_cb(self, msg: NavSatFix):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        x, y = gps_to_local(msg.latitude, msg.longitude, self.gps_lat0, self.gps_lon0)
        
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_x = x
            self.prev_y = y
            return
            
        dt = current_time - self.prev_time
        if dt <= 0.001:  # Avoid division by zero
            return
            
        dx = x - self.prev_x
        dy = y - self.prev_y
        
        # Calculate velocity in ENU frame
        vx_enu = dx / dt
        vy_enu = dy / dt
        
        # Rotate into vehicle frame (assuming latest_yaw is valid)
        v_forward = vx_enu * math.cos(self.latest_yaw) + vy_enu * math.sin(self.latest_yaw)
        
        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Only set twist for EKF fusion
        odom_msg.twist.twist.linear.x = float(v_forward)
        odom_msg.twist.twist.angular.z = float(self.latest_omega_z)
        
        self.odom_pub.publish(odom_msg)
        
        self.prev_time = current_time
        self.prev_x = x
        self.prev_y = y

def main(args=None):
    rclpy.init(args=args)
    node = KittiOdomConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
