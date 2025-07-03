"""
A dataclass that holds the information for a model object.
"""
import pybullet as p
from dataclasses import dataclass, field

@dataclass
class ModelObj:
    name: str
    path: str
    position: list[float] = field(default_factory=lambda: [0, 0, 0])
    orientation: list[float] = field(default_factory=lambda: p.getQuaternionFromEuler([0, 0, 0]))
    scale: float = field(default=1.0)