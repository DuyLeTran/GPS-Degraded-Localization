import numpy as np
from typing import List, Dict, Tuple, Optional

class Track:
    def __init__(self, track_id: int, cls: str, bbox: List[float], timestamp: float, pose):
        """
        bbox format: [cx, cy, w, h]
        pose: geometry_msgs/Pose (contains position, orientation)
        """
        self.id = track_id
        self.cls = cls
        self.history = []  # List of Tuple[timestamp, bbox, pose]
        self.spatial_history = []  # List of Tuple[np.ndarray (3D position), timestamp]
        self.add_observation(timestamp, bbox, pose)
        self.lost_frames = 0
        self.status = "TENTATIVE"  # TENTATIVE | ACTIVE | LOST | DELETED

    def add_observation(self, timestamp: float, bbox: List[float], pose):
        self.history.append((timestamp, bbox, pose))
        self.lost_frames = 0
        if len(self.history) >= 3:
            self.status = "ACTIVE"

    def mark_missed(self, max_lost_frames: int):
        self.lost_frames += 1
        if self.lost_frames > max_lost_frames:
            self.status = "DELETED"
        else:
            self.status = "LOST"


class ObjectTracker:
    def __init__(self, iou_threshold: float = 0.3, max_lost_frames: int = 15):
        self.iou_threshold = iou_threshold
        self.max_lost_frames = max_lost_frames
        self.tracks: List[Track] = []
        self.completed_tracks: List[Track] = []
        self.next_track_id = 1

    def update(self, detections: List[Dict], timestamp: float, pose) -> List[Track]:
        """
        detections: List of Dict with keys: 'cls' (str), 'bbox' (List[float] -> [cx, cy, w, h]), 'score' (float)
        pose: geometry_msgs/Pose
        """
        matched_detections = set()
        matched_tracks = set()
        
        # Clear completed tracks from previous update
        self.completed_tracks = []
        
        # 1. Match detections with existing active/lost tracks using IoU
        matches = []
        for t_idx, track in enumerate(self.tracks):
            if track.status == "DELETED":
                continue
            last_bbox = track.history[-1][1]
            for d_idx, det in enumerate(detections):
                if det['cls'] == track.cls:
                    iou = self.compute_iou(last_bbox, det['bbox'])
                    if iou >= self.iou_threshold:
                        matches.append((t_idx, d_idx, iou))
        
        # Sort matches by IoU descending
        matches.sort(key=lambda x: x[2], reverse=True)
        
        for t_idx, d_idx, iou in matches:
            if t_idx not in matched_tracks and d_idx not in matched_detections:
                matched_tracks.add(t_idx)
                matched_detections.add(d_idx)
                self.tracks[t_idx].add_observation(timestamp, detections[d_idx]['bbox'], pose)
        
        # 2. Handle unmatched tracks
        for t_idx, track in enumerate(self.tracks):
            if t_idx not in matched_tracks and track.status != "DELETED":
                track.mark_missed(self.max_lost_frames)
                
        # 3. Handle unmatched detections (new tracks)
        for d_idx, det in enumerate(detections):
            if d_idx not in matched_detections:
                new_track = Track(self.next_track_id, det['cls'], det['bbox'], timestamp, pose)
                self.tracks.append(new_track)
                self.next_track_id += 1
                
        # 4. Filter out deleted tracks for the active tracks list, save to completed list first
        self.completed_tracks = [t for t in self.tracks if t.status == "DELETED"]
        
        active_tracks = [t for t in self.tracks if t.status in ["ACTIVE", "TENTATIVE", "LOST"]]
        
        # Cleanup deleted tracks to keep the list from growing indefinitely
        self.tracks = [t for t in self.tracks if t.status != "DELETED"]
        
        return active_tracks

    def compute_iou(self, boxA: List[float], boxB: List[float]) -> float:
        wA, hA = boxA[2], boxA[3]
        xA1, yA1 = boxA[0] - wA / 2.0, boxA[1] - hA / 2.0
        xA2, yA2 = boxA[0] + wA / 2.0, boxA[1] + hA / 2.0

        wB, hB = boxB[2], boxB[3]
        xB1, yB1 = boxB[0] - wB / 2.0, boxB[1] - hB / 2.0
        xB2, yB2 = boxB[0] + wB / 2.0, boxB[1] + hB / 2.0

        xI1 = max(xA1, xB1)
        yI1 = max(yA1, yB1)
        xI2 = min(xA2, xB2)
        yI2 = min(yA2, yB2)

        interArea = max(0.0, xI2 - xI1) * max(0.0, yI2 - yI1)
        boxAArea = wA * hA
        boxBArea = wB * hB

        iou = interArea / float(boxAArea + boxBArea - interArea + 1e-6)
        return iou
