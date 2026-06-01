import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class MonocularVioNode(Node):
    def __init__(self):
        super().__init__('monocular_vio')
        
        # Declare parameters
        self.declare_parameter('max_features', 500)
        self.declare_parameter('ransac_reproj_threshold', 1.0)
        self.declare_parameter('camera_fx', 600.0)
        self.declare_parameter('camera_fy', 600.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        
        self.max_features = self.get_parameter('max_features').value
        self.ransac_thresh = self.get_parameter('ransac_reproj_threshold').value
        fx = self.get_parameter('camera_fx').value
        fy = self.get_parameter('camera_fy').value
        cx = self.get_parameter('camera_cx').value
        cy = self.get_parameter('camera_cy').value
        
        # Camera Intrinsic Matrix
        self.K = np.array([[fx, 0, cx],
                           [0, fy, cy],
                           [0, 0, 1]], dtype=np.float64)
                           
        self.bridge = CvBridge()
        
        # Subscriptions
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/vehicle/odom', self.odom_cb, 10)
        
        # Publisher
        self.vo_pub = self.create_publisher(Odometry, '/vo/odom', 10)
        
        # States for VIO
        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_time = None
        
        # Sensors states
        self.current_speed = 0.0
        self.current_gyro_z = 0.0
        
        # Dead reckoning accumulation (World frame)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        
        # ORB Detector & Matcher
        self.orb = cv2.ORB_create(nfeatures=self.max_features)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        self.get_logger().info('Monocular VIO Node started.')

    def imu_cb(self, msg: Imu):
        # Lấy delta_theta rate từ IMU gyroscope z
        self.current_gyro_z = msg.angular_velocity.z

    def odom_cb(self, msg: Odometry):
        # Lấy speed từ wheel odometry (linear x)
        self.current_speed = msg.twist.twist.linear.x

    def image_cb(self, msg: Image):
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
        try:
            curr_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
            
        # 1. Trích xuất ORB features
        kp, des = self.orb.detectAndCompute(curr_img, None)
        
        if self.prev_img is not None and self.prev_des is not None and des is not None:
            # Tính dt
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt <= 0:
                dt = 0.033 # Fallback 30fps
                
            # Matching giữa frame hiện tại và frame trước
            raw_matches = self.bf.knnMatch(self.prev_des, des, k=2)
            matches = []
            for pair in raw_matches:
                if len(pair) == 2:
                    m, n = pair
                    if m.distance < 0.75 * n.distance:
                        matches.append(m)
            
            matches = sorted(matches, key=lambda x: x.distance)
            
            if len(matches) > 8:
                pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
                pts2 = np.float32([kp[m.trainIdx].pt for m in matches])
                
                # 2. Tính Essential Matrix và recover Pose
                E, mask = cv2.findEssentialMat(pts2, pts1, self.K, cv2.RANSAC, 0.999, self.ransac_thresh, None)
                
                if E is not None and E.shape == (3, 3):
                    _, R, t, mask_pose = cv2.recoverPose(E, pts2, pts1, self.K)
                    
                    # 3. Scale translation bằng wheel odometry speed * dt
                    scale = self.current_speed * dt
                    
                    # t là direction vector của camera. Giả định camera gắn hướng tới trước:
                    # Trục Z của camera = Trục X của xe (Forward)
                    # Trục X của camera = Trục -Y của xe (Right)
                    # Ánh xạ t từ hệ tọa độ camera sang hệ tọa độ xe (body frame)
                    norm_2d = math.hypot(t[2, 0], t[0, 0])
                    if norm_2d > 1e-6:
                        dx_body = (t[2, 0] / norm_2d) * scale
                        dy_body = (-t[0, 0] / norm_2d) * scale
                    else:
                        dx_body = scale
                        dy_body = 0.0
                    
                    # 4. Lấy delta_theta từ IMU gyroscope z
                    delta_theta = self.current_gyro_z * dt
                    
                    # 5. Tích lũy pose (x, y, theta) bằng dead reckoning (Body -> World frame)
                    self.pose_x += dx_body * math.cos(self.pose_theta) - dy_body * math.sin(self.pose_theta)
                    self.pose_y += dx_body * math.sin(self.pose_theta) + dy_body * math.cos(self.pose_theta)
                    self.pose_theta += delta_theta
                    # Normalize angle to [-pi, pi]
                    self.pose_theta = math.atan2(math.sin(self.pose_theta), math.cos(self.pose_theta))
                    
                    # 6. Publish nav_msgs/Odometry lên /vo/odom
                    self.publish_odom(msg.header.stamp)
            
        # Cập nhật trạng thái cho frame tiếp theo
        self.prev_img = curr_img
        self.prev_kp = kp
        self.prev_des = des
        self.prev_time = current_time

    def publish_odom(self, stamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Yaw to Quaternion
        q_w = math.cos(self.pose_theta / 2.0)
        q_z = math.sin(self.pose_theta / 2.0)
        odom_msg.pose.pose.orientation.w = q_w
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = q_z
        
        # Velocity (Linear x from wheel, Angular z from IMU)
        odom_msg.twist.twist.linear.x = self.current_speed
        odom_msg.twist.twist.angular.z = self.current_gyro_z
        
        self.vo_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MonocularVioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
