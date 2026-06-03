import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        self.declare_parameter('weights_path', '/home/tranleduy/GPS-Degraded-Localization/YOLOv8n/best.pt')
        weights_path = self.get_parameter('weights_path').value
        
        self.get_logger().info(f"Loading YOLO model from {weights_path}")
        self.model = YOLO(weights_path)
        self.bridge = CvBridge()
        
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/detection/bboxes', 10)
        
        # Mapping class_id to string based on YOLOv8n/kitti.yaml
        self.class_names = {
            0: 'car',
            1: 'pedestrian',
            2: 'van',
            3: 'cyclist',
            4: 'truck',
            5: 'misc',
            6: 'tram',
            7: 'person_sitting'
        }
        self.get_logger().info("yolo_detector started")

    def image_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        results = self.model(cv_image, verbose=False)
        
        det_array = Detection2DArray()
        det_array.header = msg.header
        
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                det = Detection2D()
                det.header = msg.header
                
                # Bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = box.conf[0].item()
                cls_id = int(box.cls[0].item())
                
                w = x2 - x1
                h = y2 - y1
                cx = x1 + w / 2.0
                cy = y1 + h / 2.0
                
                det.bbox.center.position.x = float(cx)
                det.bbox.center.position.y = float(cy)
                det.bbox.size_x = float(w)
                det.bbox.size_y = float(h)
                
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = self.class_names.get(cls_id, str(cls_id))
                hyp.hypothesis.score = float(conf)
                
                det.results.append(hyp)
                det_array.detections.append(det)
                
        self.det_pub.publish(det_array)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
