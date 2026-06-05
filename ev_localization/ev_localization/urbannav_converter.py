import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
import math

class UrbanNavConverter(Node):
    def __init__(self, **kwargs):
        super().__init__('urbannav_converter', **kwargs)
        
        self.declare_parameter('gt_path', '/home/tranleduy/GPS-Degraded-Localization/data/UrbanNav_dataset/UrbanNav_whampoa_raw.txt')
        self.declare_parameter('sim_gps_loss', True)
        self.declare_parameter('gps_loss_start_sec', 100.0)
        self.declare_parameter('gps_loss_duration_sec', 150.0)
        self.declare_parameter('gps_loss_starts', [0.0])
        self.declare_parameter('gps_loss_durations', [0.0])
        
        self.gt_path = self.get_parameter('gt_path').value
        self.sim_gps_loss = self.get_parameter('sim_gps_loss').value
        self.gps_loss_start = self.get_parameter('gps_loss_start_sec').value
        self.gps_loss_dur = self.get_parameter('gps_loss_duration_sec').value
        self.gps_loss_starts = self.get_parameter('gps_loss_starts').value
        self.gps_loss_durations = self.get_parameter('gps_loss_durations').value
        
        # Build list of intervals for backward compatibility
        if (not self.gps_loss_starts or not self.gps_loss_durations or 
            self.gps_loss_starts == [0.0] or self.gps_loss_durations == [0.0]):
            self.gps_loss_intervals = [(self.gps_loss_start, self.gps_loss_dur)]
        else:
            self.gps_loss_intervals = list(zip(self.gps_loss_starts, self.gps_loss_durations))
        
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/vehicle/odom', 10)
        
        # Subscribe to IMU to trigger publishing synchronized with simulation clock
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        self.gt_data = []
        self.start_utc_time = None
        self.load_ground_truth()
        
        self.last_gps_pub_time = 0.0
        self.last_odom_pub_time = 0.0
        
        self.get_logger().info(f"urbannav_converter started. Loaded {len(self.gt_data)} ground truth entries.")

    def dms_to_decimal(self, deg_str, min_str, sec_str):
        deg = float(deg_str)
        minute = float(min_str)
        sec = float(sec_str)
        decimal = abs(deg) + minute / 60.0 + sec / 3600.0
        if deg < 0 or deg_str.strip().startswith('-'):
            decimal = -decimal
        return decimal

    def load_ground_truth(self):
        try:
            with open(self.gt_path, 'r') as f:
                lines = f.readlines()
        except Exception as e:
            self.get_logger().error(f"Failed to open ground truth file {self.gt_path}: {e}")
            return
            
        for line in lines:
            line = line.strip()
            if not line:
                continue
            tokens = line.split()
            # Skip header lines
            if tokens[0] == "UTCTime" or tokens[0] == "(sec)":
                continue
            
            try:
                # tokens indices:
                # 0: UTCTime (sec)
                # 1: Week
                # 2: GPSTime (sec)
                # 3,4,5: Latitude (+/-D M S)
                # 6,7,8: Longitude (+/-D M S)
                # 9: H-Ell (m)
                # 10: VelBdyX (m/s)
                # 11: VelBdyY (m/s) -> Forward velocity
                # 12: VelBdyZ (m/s)
                # 13,14,15: AccBdyX,Y,Z
                # 16: Roll, 17: Pitch, 18: Heading
                
                utc_time = float(tokens[0])
                lat = self.dms_to_decimal(tokens[3], tokens[4], tokens[5])
                lon = self.dms_to_decimal(tokens[6], tokens[7], tokens[8])
                alt = float(tokens[9])
                vel_forward = float(tokens[11])
                heading = float(tokens[18])
                
                self.gt_data.append({
                    'utc_time': utc_time,
                    'lat': lat,
                    'lon': lon,
                    'alt': alt,
                    'vel_forward': vel_forward,
                    'heading': heading
                })
            except Exception as e:
                continue

        if self.gt_data:
            self.gt_data.sort(key=lambda x: x['utc_time'])
            self.start_utc_time = self.gt_data[0]['utc_time']

    def imu_callback(self, imu_msg: Imu):
        stamp = imu_msg.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9
        
        if not self.gt_data or self.start_utc_time is None:
            return
            
        # Find closest entry in O(1) assuming 1Hz step
        offset = current_time - self.start_utc_time
        idx = int(round(offset))
        
        if idx < 0:
            idx = 0
        elif idx >= len(self.gt_data):
            idx = len(self.gt_data) - 1
            
        entry = self.gt_data[idx]
        
        # Publish GPS at 1Hz
        if current_time - self.last_gps_pub_time >= 1.0:
            self.publish_gps(entry, stamp, offset)
            self.last_gps_pub_time = current_time
            
        # Publish Odometry at 20Hz (every 0.05s)
        if current_time - self.last_odom_pub_time >= 0.05:
            omega_z = imu_msg.angular_velocity.z
            self.publish_odom(entry, stamp, omega_z)
            self.last_odom_pub_time = current_time

    def publish_gps(self, entry, stamp, elapsed_time):
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gps_link'
        
        is_lost = False
        if self.sim_gps_loss:
            for start, duration in self.gps_loss_intervals:
                if start <= elapsed_time < (start + duration):
                    is_lost = True
                    break
                
        if is_lost:
            msg.status.status = -1  # STATUS_NO_FIX
            msg.status.service = 0   # 0 satellites
            msg.position_covariance[0] = 99.0  # High HDOP
        else:
            msg.status.status = 0   # STATUS_FIX
            msg.status.service = 10  # 10 satellites (GPS_GOOD)
            msg.position_covariance[0] = 1.0   # Low HDOP (GPS_GOOD)
            
        msg.latitude = entry['lat']
        msg.longitude = entry['lon']
        msg.altitude = entry['alt']
        
        self.gps_pub.publish(msg)

    def publish_odom(self, entry, stamp, omega_z):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        msg.twist.twist.linear.x = float(entry['vel_forward'])
        msg.twist.twist.angular.z = float(omega_z)
        
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UrbanNavConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
