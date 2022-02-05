from dataclasses import dataclass
import numpy as np


@dataclass
class RoadInfo:
    frame: np.ndarray
    position: float
    curveness: tuple
    stop_distance: float
