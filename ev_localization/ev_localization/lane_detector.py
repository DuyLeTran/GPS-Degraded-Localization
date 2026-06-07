import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.get_logger().info('lane_detector started')

        # Khai báo và nạp tham số
        self.declare_parameter('roi_top_ratio', 0.6)
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('hough_min_line_length', 50)
        self.declare_parameter('hough_max_line_gap', 150)
        self.declare_parameter('slope_min', 0.3)
        self.declare_parameter('slope_max', 3.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('deviation_threshold', 0.08)

        self.roi_top_ratio = self.get_parameter('roi_top_ratio').value
        self.canny_low = self.get_parameter('canny_low').value
        self.canny_high = self.get_parameter('canny_high').value
        self.hough_threshold = self.get_parameter('hough_threshold').value
        self.hough_min_line_length = self.get_parameter('hough_min_line_length').value
        self.hough_max_line_gap = self.get_parameter('hough_max_line_gap').value
        self.slope_min = self.get_parameter('slope_min').value
        self.slope_max = self.get_parameter('slope_max').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.deviation_threshold = self.get_parameter('deviation_threshold').value

        self.get_logger().info(
            f'Lane params: roi_top_ratio={self.roi_top_ratio}, '
            f'canny=({self.canny_low}, {self.canny_high}), '
            f'hough=(th={self.hough_threshold}, minLen={self.hough_min_line_length}, '
            f'maxGap={self.hough_max_line_gap}), '
            f'slope=({self.slope_min}, {self.slope_max}), '
            f'debug_image={self.publish_debug_image}, '
            f'deviation_threshold={self.deviation_threshold}')

        # Subscriber nhận ảnh từ camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher trạng thái làn đường
        self.status_pub = self.create_publisher(String, '/vehicle/lane_status', 10)

        # Publisher ảnh debug (chỉ dùng khi publish_debug_image = True)
        self.debug_pub = self.create_publisher(Image, '/lane/debug_image', 10)

        # Khởi tạo CvBridge để chuyển đổi giữa ROS Image và OpenCV
        self.bridge = CvBridge()

    def image_callback(self, msg):
        """Callback xử lý ảnh từ camera: phát hiện làn đường bằng Canny + HoughLinesP."""
        try:
            # Chuyển đổi ROS Image sang OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Chuyển sang ảnh xám nếu ảnh đầu vào là BGR (3 kênh)
            if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            else:
                gray = cv_image

            # Kích thước ảnh
            h, w = gray.shape[:2]

            # Cắt vùng quan tâm (ROI): chỉ lấy phần dưới ảnh chứa mặt đường
            roi = gray[int(h * self.roi_top_ratio):h, :]

            # Làm mờ Gaussian để giảm nhiễu trước khi phát hiện cạnh
            blurred = cv2.GaussianBlur(roi, (5, 5), 0)

            # Phát hiện cạnh bằng thuật toán Canny
            edges = cv2.Canny(blurred, self.canny_low, self.canny_high)

            # Phát hiện đoạn thẳng bằng Hough Transform (xác suất)
            lines = cv2.HoughLinesP(
                edges, 1, np.pi / 180,
                self.hough_threshold,
                minLineLength=self.hough_min_line_length,
                maxLineGap=self.hough_max_line_gap
            )

            # Phân loại trạng thái làn đường dựa trên các đoạn thẳng tìm được
            status = self._classify_lanes(lines, w, roi.shape[0])

            # Publish trạng thái làn đường
            status_msg = String()
            status_msg.data = status
            self.status_pub.publish(status_msg)

            self.get_logger().info(
                f'Lane status: {status}', throttle_duration_sec=2.0)

            # Publish ảnh debug nếu được bật
            if self.publish_debug_image:
                debug_msg = self._create_debug_image(roi, lines)
                self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Lane detection error: {e}')

    def _classify_lanes(self, lines, image_width, roi_height):
        """Phân loại vị trí xe so với làn đường dựa trên khoảng cách lệch của xe với trung tâm làn.

        Trả về:
            'CENTER'  — xe chạy chính giữa làn (độ lệch nhỏ hơn ngưỡng)
            'LEFT'    — xe đang lệch/nghiêng về bên trái làn đường
            'RIGHT'   — xe đang lệch/nghiêng về bên phải làn đường
            'UNKNOWN' — không phát hiện được làn đường
        """
        if lines is None or len(lines) == 0:
            return 'UNKNOWN'

        left_lines = []
        right_lines = []
        left_intercepts = []
        right_intercepts = []
        cx = image_width / 2.0  # Tâm ảnh theo chiều ngang

        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Tránh chia cho 0
            if x2 == x1:
                continue

            slope = (y2 - y1) / (x2 - x1)
            abs_slope = abs(slope)

            # Lọc bỏ đường quá ngang hoặc quá dọc
            if abs_slope < self.slope_min or abs_slope > self.slope_max:
                continue

            x_mid = (x1 + x2) / 2.0

            # Tính điểm cắt x_intercept tại đáy ROI (y = roi_height)
            # Phương trình đường thẳng: x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if y2 != y1:
                x_intercept = x1 + (roi_height - y1) * (x2 - x1) / (y2 - y1)
            else:
                x_intercept = x_mid

            # Trong hệ toạ độ ảnh (y tăng xuống dưới), nhìn từ camera phía trước:
            #   - Làn TRÁI: slope âm (đi từ dưới-trái lên trên-phải) VÀ x_mid < cx
            #   - Làn PHẢI: slope dương (đi từ dưới-phải lên trên-trái) VÀ x_mid > cx
            if slope < 0 and x_mid < cx:
                left_lines.append(line)
                left_intercepts.append(x_intercept)
            elif slope > 0 and x_mid > cx:
                right_lines.append(line)
                right_intercepts.append(x_intercept)

        # Tính toán độ lệch nếu phát hiện đầy đủ cả hai làn đường
        if len(left_lines) > 0 and len(right_lines) > 0:
            left_x = np.mean(left_intercepts)
            right_x = np.mean(right_intercepts)
            lane_width = right_x - left_x
            if lane_width > 0:
                lane_center = (left_x + right_x) / 2.0
                offset_ratio = (lane_center - cx) / lane_width
                
                # Nếu camera (xe) lệch trái, trung tâm làn trong ảnh sẽ dịch sang phải (offset_ratio dương)
                if offset_ratio > self.deviation_threshold:
                    return 'LEFT'   # Xe đang lệch về mép trái làn
                elif offset_ratio < -self.deviation_threshold:
                    return 'RIGHT'  # Xe đang lệch về mép phải làn
            return 'CENTER'
        elif len(left_lines) > 0 and len(right_lines) == 0:
            return 'RIGHT'   # Chỉ thấy làn trái → xe lệch sang phải
        elif len(right_lines) > 0 and len(left_lines) == 0:
            return 'LEFT'    # Chỉ thấy làn phải → xe lệch sang trái
        else:
            return 'UNKNOWN'

    def _create_debug_image(self, roi, lines):
        """Tạo ảnh debug với các đoạn thẳng được tô màu theo phân loại.

        Màu sắc:
            - XANH LÁ (0,255,0): làn trái
            - XANH DƯƠNG (255,0,0): làn phải
            - XÁM (128,128,128): đoạn thẳng bị lọc bỏ
            - ĐỎ (0,0,255): đường tâm ảnh
        """
        # Chuyển ảnh xám sang BGR để vẽ màu
        debug_img = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        h, w = debug_img.shape[:2]

        if lines is not None:
            cx = w / 2.0

            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Xác định màu dựa trên slope và vị trí
                if x2 == x1:
                    color = (128, 128, 128)  # Xám: đường thẳng đứng
                else:
                    slope = (y2 - y1) / (x2 - x1)
                    abs_slope = abs(slope)
                    x_mid = (x1 + x2) / 2.0

                    if abs_slope < self.slope_min or abs_slope > self.slope_max:
                        color = (128, 128, 128)  # Xám: bị lọc bỏ
                    elif slope < 0 and x_mid < cx:
                        color = (0, 255, 0)      # Xanh lá: làn trái
                    elif slope > 0 and x_mid > cx:
                        color = (255, 0, 0)      # Xanh dương: làn phải
                    else:
                        color = (128, 128, 128)  # Xám: không khớp tiêu chí

                cv2.line(debug_img, (x1, y1), (x2, y2), color, 2)

        # Vẽ đường tâm ảnh màu đỏ
        cv2.line(debug_img, (w // 2, 0), (w // 2, h), (0, 0, 255), 1)

        # Chuyển ngược về ROS Image message
        return self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
