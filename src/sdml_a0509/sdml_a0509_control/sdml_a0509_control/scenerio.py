import json
import os
import time
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sdml_a0509_control.position_move import movejoint, moveline
from sdml_a0509_control.utils import get_pick_place_config_path, get_pick_place_times_path

DEFAULT_HOME_POSJ = [-41.36, -32.01, 84.97994995117188, -3.99, 115.02, 45.00]


def _load_json(path, default=None):
    if default is None:
        default = {}
    if not os.path.isfile(path):
        return default
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError):
        return default


def _to_posj(posj):
    if posj is None:
        return None
    return [float(posj[i]) if i < len(posj) else 0.0 for i in range(6)]


def _posx_to_pose(posx, z_offset=0.0):
    if posx is None:
        return None
    p = Pose()
    p.position.x = float(posx[0]) if len(posx) > 0 else 0.0
    p.position.y = float(posx[1]) if len(posx) > 1 else 0.0
    p.position.z = (float(posx[2]) if len(posx) > 2 else 0.0) + z_offset
    p.orientation.x = float(posx[3]) if len(posx) > 3 else 0.0
    p.orientation.y = float(posx[4]) if len(posx) > 4 else 0.0
    p.orientation.z = float(posx[5]) if len(posx) > 5 else 0.0
    return p


def _load_config(path):
    if not os.path.isfile(path):
        raise FileNotFoundError("Config not found: %s" % path)
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _get_time(times_cfg, key, default=2.0):
    return float(times_cfg.get(key, times_cfg.get("default_time", default)))


def do_scenerio(node: Node, path: str | None = None, times_path: str | None = None) -> bool:
    path = path or get_pick_place_config_path()
    times_path = times_path or get_pick_place_times_path()
    cfg = _load_config(path)
    times = _load_json(times_path, {"default_time": 2.0})

    pick_posj = _to_posj(cfg.get("pick_posj"))
    pick_posx = cfg.get("pick_posx")
    place_posj = _to_posj(cfg.get("place_posj"))
    place_posx = cfg.get("place_posx")
    z_up = float(cfg.get("z_up", 100.0))
    pick_j6_offset_deg = float(cfg.get("pick_j6_offset_deg", 0.0))
    place_j6_offset_deg = float(cfg.get("place_j6_offset_deg", 0.0))
    gripper_open = float(cfg.get("gripper_open_percent", 80.0))
    gripper_close = float(cfg.get("gripper_close_percent", 10.0))

    t_pick_approach = _get_time(times, "pick_approach_time")
    t_pick_depart = _get_time(times, "pick_depart_time")
    t_place_approach = _get_time(times, "place_approach_time")
    t_place_position = _get_time(times, "place_position_time")
    t_place_depart = _get_time(times, "place_depart_time")
    t_gripper_close = _get_time(times, "gripper_close_time", 1.2)
    t_gripper_open = _get_time(times, "gripper_open_time", 1.2)

    if not pick_posx or len(pick_posx) < 6 or not place_posx or len(place_posx) < 6:
        node.get_logger().error("pick_posx/place_posx (6 values) required in config")
        return False
    if not pick_posj or len(pick_posj) < 6 or not place_posj or len(place_posj) < 6:
        node.get_logger().error("pick_posj/place_posj (6 values) required in config")
        return False

    pick_posx_off = list(pick_posx)
    pick_posx_off[5] = float(pick_posx_off[5]) + pick_j6_offset_deg

    pos1 = _posx_to_pose(pick_posx_off, z_up)
    pos2 = _to_posj(pick_posj)
    pos3 = list(pos2)
    pos3[5] = float(pos3[5]) + pick_j6_offset_deg
    pos4 = pos1
    pos5 = _posx_to_pose(place_posx, z_up)
    pos6 = _to_posj(place_posj)
    pos7 = list(pos6)
    pos7[5] = float(pos7[5]) + place_j6_offset_deg
    pos8 = pos5

    if not moveline(pos1, time_sec=t_pick_approach):
        return False
    time.sleep(t_pick_approach)

    t_pick_j6 = _get_time(times, "pick_j6_offset_time", _get_time(times, "pick_position_time"))
    if not movejoint(pos3, time_sec=t_pick_j6):
        return False
    time.sleep(t_pick_j6)

    deg = gripper_open
    while deg >= gripper_close:
        node.set_gripper(deg)
        time.sleep(0.1)
        deg -= 1.0
    time.sleep(t_gripper_close)

    if not moveline(pos4, time_sec=t_pick_depart):
        return False
    time.sleep(t_pick_depart)

    if not moveline(pos5, time_sec=t_place_approach):
        return False
    time.sleep(max(0.0, t_place_approach - 3.0))

    if not movejoint(pos7, time_sec=t_place_position):
        return False
    time.sleep(t_place_position)

    deg = gripper_close
    while deg <= gripper_open:
        node.set_gripper(deg)
        time.sleep(0.1)
        deg += 1.0
    time.sleep(t_gripper_open)

    if not moveline(pos8, time_sec=t_place_depart):
        return False
    time.sleep(t_place_depart)

    if not movejoint(DEFAULT_HOME_POSJ, time_sec=_get_time(times, "home_time", 5.0)):
        return False

    return True

