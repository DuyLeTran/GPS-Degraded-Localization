import json
import numpy as np
from dataclasses import dataclass
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

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "class": self.cls,
            "position_enu": self.p3d.tolist() if self.p3d is not None else [],
            "descriptor": self.descriptor.tolist() if self.descriptor is not None else [],
            "t_first": self.t_first,
            "t_last": self.t_last,
            "n_obs": self.n_obs,
            "bbox_size": list(self.bbox_size)
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'Landmark':
        # Bản đồ các field JSON mẫu ("position_enu", "class") sang object
        p3d_data = data.get("p3d", data.get("position_enu", [0.0, 0.0, 0.0]))
        descriptor_data = data.get("descriptor", [])
        
        return cls(
            id=data.get("id", -1),
            cls=data.get("cls", data.get("class", "unknown")),
            p3d=np.array(p3d_data, dtype=np.float64),
            descriptor=np.array(descriptor_data, dtype=np.float64),
            t_first=data.get("t_first", 0.0),
            t_last=data.get("t_last", 0.0),
            n_obs=data.get("n_obs", 0),
            bbox_size=tuple(data.get("bbox_size", (0.0, 0.0)))
        )

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
            dx = lm.p3d[0] - x
            dy = lm.p3d[1] - y
            # Dùng khoảng cách Euclidean tính bình phương cho nhanh (Euclidean distance 2D)
            if (dx**2 + dy**2) <= radius_sq:
                results.append(lm)
        return results

    def get_by_id(self, lm_id: int) -> Optional[Landmark]:
        """Lấy một landmark cụ thể từ database thông qua ID"""
        return self.landmarks.get(lm_id, None)
