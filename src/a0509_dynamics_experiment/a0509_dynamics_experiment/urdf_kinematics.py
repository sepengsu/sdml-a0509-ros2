"""Small URDF FK reader for visualizing the six A0509 links without ROS."""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET


def find_a0509_urdf() -> Path | None:
    local = Path.cwd() / "src" / "doosan-robot2" / "dsr_description2" / "urdf" / "a0509.urdf"
    if local.is_file():
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    try:
        installed = Path(get_package_share_directory("dsr_description2")) / "urdf" / "a0509.urdf"
        return installed if installed.is_file() else None
    except Exception:
        return None


class A0509Urdf:
    def __init__(self, path: str | Path):
        import numpy as np
        self.np = np
        root = ET.parse(path).getroot()
        by_name = {joint.attrib.get("name"): joint for joint in root.findall("joint")}
        self.joints = []
        expected_parent = "base_link"
        for i in range(1, 7):
            element = by_name.get(f"joint_{i}")
            if element is None:
                raise ValueError(f"URDF missing joint_{i}")
            parent = element.find("parent").attrib["link"]
            child = element.find("child").attrib["link"]
            if parent != expected_parent:
                raise ValueError("A0509 URDF arm chain is not continuous")
            origin = element.find("origin")
            xyz = self._values(origin.attrib.get("xyz", "0 0 0"))
            rpy = self._values(origin.attrib.get("rpy", "0 0 0"))
            axis = self._values(element.find("axis").attrib.get("xyz", "0 0 1"))
            self.joints.append((xyz, rpy, axis))
            expected_parent = child

    @staticmethod
    def _values(text: str):
        values = tuple(float(x) for x in text.split())
        if len(values) != 3:
            raise ValueError("URDF transform requires three values")
        return values

    def _rotation(self, axis, angle):
        np = self.np
        vector = np.asarray(axis, dtype=float)
        vector /= np.linalg.norm(vector)
        x, y, z = vector
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c + x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
                         [y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s],
                         [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)]])

    def links_mm(self, q_rad):
        np = self.np
        if len(q_rad) != 6:
            raise ValueError("A0509 FK requires six joint angles")
        transform = np.eye(4)
        points = [(0.0, 0.0, 0.0)]
        for (xyz, rpy, axis), angle in zip(self.joints, q_rad):
            roll, pitch, yaw = rpy
            origin_rotation = self._rotation((0, 0, 1), yaw) @ self._rotation((0, 1, 0), pitch) @ self._rotation((1, 0, 0), roll)
            segment = np.eye(4)
            segment[:3, :3] = origin_rotation @ self._rotation(axis, float(angle))
            segment[:3, 3] = xyz
            transform = transform @ segment
            points.append(tuple(float(v * 1000) for v in transform[:3, 3]))
        return points
