import json
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict

@dataclass
class Landmark:
    id: int
    cls: str
    p3d: np.ndarray      # [x, y, z]
    descriptor: np.ndarray
    t_first: float
    t_last: float
    n_obs: int
    bbox_size: Tuple[float, float]
    # --- New fields for lifecycle and filtering ---
    status: str = "CANDIDATE"        # CANDIDATE | PROVISIONAL | CONFIRMED | ARCHIVED
    confidence: float = 0.0          # [0, 1]
    position_variance: float = 999.0 # Variance of position estimates
    source: str = "triangulation"    # triangulation | depth_from_size | structure
    position_history: List[np.ndarray] = field(default_factory=list, repr=False)

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "class": self.cls,
            "position_enu": self.p3d.tolist() if self.p3d is not None else [],
            "descriptor": self.descriptor.tolist() if self.descriptor is not None else [],
            "t_first": self.t_first,
            "t_last": self.t_last,
            "n_obs": self.n_obs,
            "bbox_size": list(self.bbox_size),
            "status": self.status,
            "confidence": self.confidence,
            "position_variance": self.position_variance,
            "source": self.source
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'Landmark':
        p3d_data = data.get("p3d", data.get("position_enu", [0.0, 0.0, 0.0]))
        descriptor_data = data.get("descriptor", [])
        
        lm = cls(
            id=data.get("id", -1),
            cls=data.get("cls", data.get("class", "unknown")),
            p3d=np.array(p3d_data, dtype=np.float64),
            descriptor=np.array(descriptor_data, dtype=np.float64),
            t_first=data.get("t_first", 0.0),
            t_last=data.get("t_last", 0.0),
            n_obs=data.get("n_obs", 0),
            bbox_size=tuple(data.get("bbox_size", (0.0, 0.0))),
            status=data.get("status", "CANDIDATE"),
            confidence=data.get("confidence", 0.0),
            position_variance=data.get("position_variance", 999.0),
            source=data.get("source", "triangulation")
        )
        lm.position_history = [lm.p3d]
        return lm

    def update_position(self, new_p3d: np.ndarray, timestamp: float):
        if not hasattr(self, 'position_history') or self.position_history is None:
            self.position_history = [self.p3d]
        self.position_history.append(new_p3d)
        self.n_obs += 1
        self.t_last = timestamp
        
        # Running average
        self.p3d = np.mean(self.position_history, axis=0)
        
        # Calculate variance (sum of variances of x, y, z)
        if len(self.position_history) > 1:
            vars_xyz = np.var(self.position_history, axis=0)
            self.position_variance = float(np.sum(vars_xyz))
        else:
            self.position_variance = 0.0


class LandmarkDB:
    def __init__(self):
        self.landmarks: Dict[int, Landmark] = {}

    def load(self, path: str) -> None:
        """Đọc database từ file JSON"""
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            
        self.landmarks.clear()
        # Hỗ trợ list trả về trực tiếp hoặc nằm trong object "landmarks"
        items = data.get("landmarks", data) if isinstance(data, dict) else data
        
        for item in items:
            lm = Landmark.from_dict(item)
            self.landmarks[lm.id] = lm

    def save(self, path: str) -> None:
        """Lưu toàn bộ database xuống file JSON"""
        data = {
            "landmarks": [lm.to_dict() for lm in self.landmarks.values()]
        }
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4)

    def query_nearby(self, x: float, y: float, radius: float) -> List[Landmark]:
        """Lọc danh sách các landmark nằm trong bán kính radius (so với x, y 2D)"""
        results = []
        radius_sq = radius ** 2
        for lm in self.landmarks.values():
            # Chỉ dùng landmarks đã được xác nhận (CONFIRMED) hoặc tạm thời (PROVISIONAL) để chiếu
            if lm.status not in ["CONFIRMED", "PROVISIONAL"]:
                continue
            dx = lm.p3d[0] - x
            dy = lm.p3d[1] - y
            if (dx**2 + dy**2) <= radius_sq:
                results.append(lm)
        return results

    def get_by_id(self, lm_id: int) -> Optional[Landmark]:
        """Lấy một landmark cụ thể từ database thông qua ID"""
        return self.landmarks.get(lm_id, None)

    def associate(self, cls: str, position: np.ndarray, radius: float = 3.0) -> Optional[Landmark]:
        """Tìm landmark cùng class có khoảng cách 3D gần nhất trong bán kính radius"""
        best_lm = None
        best_dist = radius
        for lm in self.landmarks.values():
            if lm.cls == cls:
                dist = np.linalg.norm(lm.p3d - position)
                if dist < best_dist:
                    best_dist = dist
                    best_lm = lm
        return best_lm
