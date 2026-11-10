import rclpy
from std_msgs.msg import Float64MultiArray
from dsr_msgs2.srv import MoveLine, MoveJoint, MoveCircle

MOVE_LINE_SERVICE = "/motion/move_line"
MOVE_JOINT_SERVICE = "/motion/move_joint"
MOVE_CIRCLE_SERVICE = "/motion/move_circle"


def configure_service_names(
    move_line_service=None,
    move_joint_service=None,
    move_circle_service=None,
):
    global MOVE_LINE_SERVICE, MOVE_JOINT_SERVICE, MOVE_CIRCLE_SERVICE
    if move_line_service:
        MOVE_LINE_SERVICE = str(move_line_service)
    if move_joint_service:
        MOVE_JOINT_SERVICE = str(move_joint_service)
    if move_circle_service:
        MOVE_CIRCLE_SERVICE = str(move_circle_service)


def moveline(pose, time_sec=2.0):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    r = getattr(pose.orientation, "x", 0.0)
    p = getattr(pose.orientation, "y", 0.0)
    phy = getattr(pose.orientation, "z", 0.0)
    pos_list = [float(x), float(y), float(z), float(r), float(p), float(phy)]
    node = rclpy.create_node("move_to_pose")
    cli = node.create_client(MoveLine, MOVE_LINE_SERVICE)
    req = MoveLine.Request(pos=pos_list)
    req.vel = [10.0, 10.0]
    req.acc = [10.0, 10.0]
    req.time = float(time_sec)
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ok = bool(future.result() is not None and future.result().success)
    node.destroy_client(cli)
    node.destroy_node()
    return ok


def movejoint(joint, time_sec=2.0):
    node = rclpy.create_node("move_to_joint")
    cli = node.create_client(MoveJoint, MOVE_JOINT_SERVICE)
    req = MoveJoint.Request(pos=joint)
    req.vel = 10.0
    req.acc = 10.0
    req.time = float(time_sec)
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ok = bool(future.result() is not None and future.result().success)
    node.destroy_client(cli)
    node.destroy_node()
    return ok


def movecircle(pos1, pos2, vel=None, acc=None, time_sec=3.0, ref=0, mode=0):
    pos1 = [float(pos1[i]) if i < len(pos1) else 0.0 for i in range(6)]
    pos2 = [float(pos2[i]) if i < len(pos2) else 0.0 for i in range(6)]
    if vel is None:
        vel = [10.0, 10.0]
    if acc is None:
        acc = [10.0, 10.0]
    node = rclpy.create_node("move_circle")
    cli = node.create_client(MoveCircle, MOVE_CIRCLE_SERVICE)
    req = MoveCircle.Request()
    req.pos = [Float64MultiArray(data=pos1), Float64MultiArray(data=pos2)]
    req.vel = list(vel)
    req.acc = list(acc)
    req.time = float(time_sec)
    req.radius = 0.0
    req.ref = int(ref)
    req.mode = int(mode)
    req.angle1 = 0.0
    req.angle2 = 0.0
    req.blend_type = 0
    req.sync_type = 0
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ok = bool(future.result() is not None and future.result().success)
    node.destroy_client(cli)
    node.destroy_node()
    return ok

