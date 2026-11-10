import json
import time
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sdml_a0509_control.position_move import moveline, movejoint, movecircle


def do_position_based_scenerio(node: Node, path: str):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, list):
        node.get_logger().error("List format expected, got %s" % type(data))
        return False
    for item in data:
        if not isinstance(item, dict):
            continue
        posx = item.get("posx")
        posj = item.get("posj")
        gripper = item.get("gripper")
        movec = item.get("movec")

        def _time_sec(key_default, default_val):
            v = item.get("time")
            if v is not None:
                return float(v)
            return float(item.get(key_default, default_val))

        if posx is not None:
            pose = Pose()
            pose.position.x = float(posx[0]) if len(posx) > 0 else 0.0
            pose.position.y = float(posx[1]) if len(posx) > 1 else 0.0
            pose.position.z = float(posx[2]) if len(posx) > 2 else 0.0
            pose.orientation.x = float(posx[3]) if len(posx) > 3 else 0.0
            pose.orientation.y = float(posx[4]) if len(posx) > 4 else 0.0
            pose.orientation.z = float(posx[5]) if len(posx) > 5 else 0.0
            moveline(pose, time_sec=_time_sec("posx_time", 2.0))
        elif posj is not None:
            joint = [float(posj[i]) if i < len(posj) else 0.0 for i in range(6)]
            movejoint(joint, time_sec=_time_sec("posj_time", 2.0))
        elif movec is not None:
            if isinstance(movec, (list, tuple)) and len(movec) >= 2:
                pos1 = [float(movec[0][i]) if i < len(movec[0]) else 0.0 for i in range(6)]
                pos2 = [float(movec[1][i]) if i < len(movec[1]) else 0.0 for i in range(6)]
                movecircle(pos1, pos2, time_sec=_time_sec("movec_time", 3.0))
            else:
                node.get_logger().error("movec must be [[pos1_6], [pos2_6]]: %s" % movec)
                continue
        elif gripper is not None:
            node.set_gripper(float(gripper))
            time.sleep(0.3)
        else:
            node.get_logger().error(f"Invalid position: {item}")
            continue
    return True

