import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int32
import numpy as np
import os
import json
import math

from ev_localization.landmark_db import LandmarkDB, Landmark
from ev_localization.object_tracker import ObjectTracker, Track
from ev_localization.triangulator import (
    get_camera_transform,
    triangulate_two_rays,
    depth_from_known_size,
    flat_ground_projection,
    sanity_check,
    euler_to_mat,
    select_best_keyframe_pair  # Added missing import
)

class LandmarkBuilderNode(Node):
    def __init__(self):
        super().__init__('landmark_builder')
        
        # Parameters
        self.declare_parameter('db_path', 'config/landmarks_urbannav.json')
        self.declare_parameter('fx', 264.9425)
        self.declare_parameter('fy', 264.79)
        self.declare_parameter('cx', 334.3975)
        self.declare_parameter('cy', 183.162)
        
        # Camera Extrinsics relative to base_link
        self.declare_parameter('cam_x', 0.0)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 1.5)
        self.declare_parameter('cam_roll', -1.5708)
        self.declare_parameter('cam_pitch', 0.0)
        self.declare_parameter('cam_yaw', -1.5708)
        
        self.declare_parameter('min_observations', 5)
        self.declare_parameter('max_position_variance_m', 3.0)
        
        # Parse params
        db_path = self.get_parameter('db_path').value
        if not os.path.isabs(db_path):
            # Write directly to the source directory to persist changes
            pkg_dir = '/home/tranleduy/GPS-Degraded-Localization/ev_localization'
            self.db_path = os.path.join(pkg_dir, db_path)
            os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        else:
            self.db_path = db_path
            
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        
        cam_x = self.get_parameter('cam_x').value
        cam_y = self.get_parameter('cam_y').value
        cam_z = self.get_parameter('cam_z').value
        cam_roll = self.get_parameter('cam_roll').value
        cam_pitch = self.get_parameter('cam_pitch').value
        cam_yaw = self.get_parameter('cam_yaw').value
        
        self.min_observations = self.get_parameter('min_observations').value
        self.max_position_variance_m = self.get_parameter('max_position_variance_m').value
        
        # Extrinsics Homogeneous Matrix T_bc
        self.T_bc = np.eye(4)
        self.T_bc[:3, :3] = euler_to_mat(cam_roll, cam_pitch, cam_yaw)
        self.T_bc[:3, 3] = [cam_x, cam_y, cam_z]
        
        # Database
        self.db = LandmarkDB()
        if os.path.exists(self.db_path):
            try:
                self.db.load(self.db_path)
                self.get_logger().info(f"Loaded {len(self.db.landmarks)} landmarks from {self.db_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load DB: {e}. Starting fresh.")
        else:
            self.get_logger().info(f"DB path {self.db_path} does not exist. Starting fresh.")
            
        # Tracker
        self.tracker = ObjectTracker(iou_threshold=0.3, max_lost_frames=15)
        
        # Known sizes for fallback
        self.known_heights = {
            'car': 1.5,
            'van': 2.0,
            'truck': 3.5,
            'tram': 3.2,
            'misc': 1.5
        }
        
        self.latest_pose = None
        self.next_landmark_id = max(self.db.landmarks.keys()) + 1 if self.db.landmarks else 1
        
        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/ekf/pose', self.pose_cb, 10)
        self.det_sub = self.create_subscription(Detection2DArray, '/detection/bboxes', self.detection_cb, 10)
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/landmark/markers', 10)
        self.count_pub = self.create_publisher(Int32, '/landmark/count', 10)
        
        # Timer to save DB periodically and publish markers
        self.create_timer(10.0, self.timer_cb)
        
        self.get_logger().info("Landmark Builder Node initialized.")

    def pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg.pose

    def detection_cb(self, msg: Detection2DArray):
        if self.latest_pose is None:
            return
            
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Convert Detection2DArray to tracker detections
        tracker_dets = []
        allowed_classes = ['car', 'van', 'truck', 'tram', 'misc']
        
        for det in msg.detections:
            if not det.results:
                continue
            cls_name = det.results[0].hypothesis.class_id.lower()
            if cls_name not in allowed_classes:
                continue
                
            score = det.results[0].hypothesis.score
            bbox = [
                det.bbox.center.position.x,
                det.bbox.center.position.y,
                det.bbox.size_x,
                det.bbox.size_y
            ]
            tracker_dets.append({
                'cls': cls_name,
                'bbox': bbox,
                'score': score
            })
            
        # Update tracker
        active_tracks = self.tracker.update(tracker_dets, timestamp, self.latest_pose)
        
        # For each active track, perform Flat-Ground Projection to estimate its current 3D position
        R_wc, t_wc = get_camera_transform(self.latest_pose, self.T_bc)
        
        # heading yaw
        q = self.latest_pose.orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)
        
        for track in active_tracks:
            # Only process tracks that just received a new observation at this timestamp
            if abs(track.history[-1][0] - timestamp) < 1e-3:
                last_bbox = track.history[-1][1]
                p_3d = flat_ground_projection(
                    last_bbox, self.fx, self.fy, self.cx, self.cy, R_wc, t_wc
                )
                if p_3d is not None:
                    # Sanity check
                    vehicle_pos = np.array([self.latest_pose.position.x, self.latest_pose.position.y, self.latest_pose.position.z])
                    if sanity_check(p_3d, vehicle_pos, yaw):
                        track.spatial_history.append((p_3d, timestamp))
                        
        # Process completed tracks (those that are deleted or went out of view)
        for track in self.tracker.completed_tracks:
            self.process_completed_track(track)

    def process_completed_track(self, track: Track):
        valid_spatial_obs = [p3d for p3d, ts in track.spatial_history]
        
        if len(valid_spatial_obs) < self.min_observations:
            return
            
        # Compute spatial variance
        vars_xyz = np.var(valid_spatial_obs, axis=0)
        total_variance = float(np.sum(vars_xyz))
        
        # If variance is too high, it means the object was moving
        if total_variance > self.max_position_variance_m:
            self.get_logger().info(f"Rejected Track {track.id} ({track.cls}) due to high variance: {total_variance:.2f} m^2 (dynamic object)")
            return
            
        # Refined position using average of flat-ground projection
        avg_p3d = np.mean(valid_spatial_obs, axis=0)
        
        # Try to triangulate from history to verify/improve
        triangulation_success = False
        best_pair = select_best_keyframe_pair(track.history)
        if best_pair is not None:
            obs_a, obs_b = best_pair
            R_wc_a, t_wc_a = get_camera_transform(obs_a[2], self.T_bc)
            R_wc_b, t_wc_b = get_camera_transform(obs_b[2], self.T_bc)
            
            # Construct ray vectors
            ray_a_cam = np.array([(obs_a[1][0] - self.cx)/self.fx, (obs_a[1][1] - self.cy)/self.fy, 1.0])
            ray_a_world = R_wc_a @ ray_a_cam
            ray_a_world = ray_a_world / np.linalg.norm(ray_a_world)
            
            ray_b_cam = np.array([(obs_b[1][0] - self.cx)/self.fx, (obs_b[1][1] - self.cy)/self.fy, 1.0])
            ray_b_world = R_wc_b @ ray_b_cam
            ray_b_world = ray_b_world / np.linalg.norm(ray_b_world)
            
            triang_res = triangulate_two_rays(ray_a_world, t_wc_a, ray_b_world, t_wc_b)
            if triang_res is not None:
                p_triang, parallax = triang_res
                # If parallax is good, and it passes sanity check
                if parallax >= 5.0:
                    pos_i = np.array([obs_b[2].position.x, obs_b[2].position.y, obs_b[2].position.z])
                    # heading yaw
                    q = obs_b[2].orientation
                    t3 = +2.0 * (q.w * q.z + q.x * q.y)
                    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    yaw = math.atan2(t3, t4)
                    if sanity_check(p_triang, pos_i, yaw):
                        avg_p3d = p_triang
                        triangulation_success = True
                        
        source = "triangulation" if triangulation_success else "flat_ground"
        
        # Check if this landmark matches an existing one in DB
        matching_lm = self.db.associate(track.cls, avg_p3d, radius=3.0)
        
        if matching_lm is not None:
            # Update existing landmark
            matching_lm.update_position(avg_p3d, track.history[-1][0])
            # Promote provisional/candidate if it gets more observations
            if matching_lm.n_obs >= self.min_observations * 2:
                matching_lm.status = "CONFIRMED"
            elif matching_lm.n_obs >= self.min_observations:
                matching_lm.status = "PROVISIONAL"
                
            self.get_logger().info(f"Associated Track {track.id} with existing Landmark {matching_lm.id} ({matching_lm.cls}). Total obs: {matching_lm.n_obs}")
        else:
            # Create new landmark
            new_lm = Landmark(
                id=self.next_landmark_id,
                cls=track.cls,
                p3d=avg_p3d,
                descriptor=np.zeros(4, dtype=np.float64),
                t_first=track.history[0][0],
                t_last=track.history[-1][0],
                n_obs=len(valid_spatial_obs),
                bbox_size=(float(track.history[-1][1][2]), float(track.history[-1][1][3])),
                status="PROVISIONAL" if len(valid_spatial_obs) >= self.min_observations else "CANDIDATE",
                confidence=1.0,
                position_variance=total_variance,
                source=source
            )
            # If it has really high observations and low variance, confirm immediately
            if new_lm.n_obs >= self.min_observations * 1.5 and total_variance < 1.0:
                new_lm.status = "CONFIRMED"
                
            self.db.landmarks[new_lm.id] = new_lm
            self.next_landmark_id += 1
            self.get_logger().info(f"Created new Landmark {new_lm.id} ({new_lm.cls}) at {new_lm.p3d} using {source}. Obs: {new_lm.n_obs}")

    def timer_cb(self):
        # Save DB
        try:
            self.db.save(self.db_path)
            self.get_logger().info(f"Auto-saved Landmark DB to {self.db_path}. Total landmarks: {len(self.db.landmarks)}")
        except Exception as e:
            self.get_logger().error(f"Failed to auto-save DB: {e}")
            
        # Publish total confirmed landmark count
        confirmed_count = sum(1 for lm in self.db.landmarks.values() if lm.status in ["CONFIRMED", "PROVISIONAL"])
        count_msg = Int32()
        count_msg.data = confirmed_count
        self.count_pub.publish(count_msg)
        
        # Publish RViz markers
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # Delete old markers first to avoid ghost markers in rviz
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)
        self.marker_pub.publish(marker_array)
        
        marker_array = MarkerArray()
        for lm in self.db.landmarks.values():
            # Only show confirmed or provisional landmarks in RViz
            if lm.status not in ["CONFIRMED", "PROVISIONAL"]:
                continue
                
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'landmarks'
            marker.id = lm.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(lm.p3d[0])
            marker.pose.position.y = float(lm.p3d[1])
            marker.pose.position.z = float(lm.p3d[2])
            marker.pose.orientation.w = 1.0
            
            # Scale
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.5 if lm.cls in ['truck', 'tram'] else 1.0
            
            # Color based on class
            marker.color.a = 0.8
            if lm.cls == 'car':
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0 # Green
            elif lm.cls == 'van':
                marker.color.r = 0.0; marker.color.g = 0.8; marker.color.b = 0.8 # Cyan
            elif lm.cls == 'truck':
                marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0 # Orange
            elif lm.cls == 'tram':
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0 # Magenta
            else:
                marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5 # Gray
                
            marker_array.markers.append(marker)
            
            # Also publish a text label marker
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = marker.header.stamp
            text_marker.ns = 'landmark_labels'
            text_marker.id = lm.id + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(lm.p3d[0])
            text_marker.pose.position.y = float(lm.p3d[1])
            text_marker.pose.position.z = float(lm.p3d[2]) + 1.2
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.8 # Text size
            text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0
            text_marker.text = f"{lm.cls}#{lm.id}"
            
            marker_array.markers.append(text_marker)
            
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
