#!/usr/bin/env python3
"""
a0509_position_getter node: ROS2 node for Doosan A0509.
Handles services, jogging, and JSON storage for positions/scenarios.
"""

import json
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from dsr_msgs2.srv import (
    GetCurrentPosx,
    GetCurrentPosj,
    GetCurrentSolutionSpace,
    Jog,
    JogMulti,
    MoveJoint,
    MoveLine,
    MoveStop,
    ParallelAxis2,
    SetRobotMode,
)

# Gripper: DH Robotics AG145
DEFAULT_GRIPPER_COMMAND_TOPIC = "/ag145/forward_position_controller/commands"
GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSED_POS = 0.93

DEFAULT_HOME_POSJ = [4.00, -32.01, 84.98, -3.99, 115.02, 45.00]
DEFAULT_HOME_POSJ = [ -41.36,-32.01,84.97994995117188,-3.99,115.02,45.00]

def _get_json_dir():
    """저장/로드용 json 폴더. JSON_DIR env 또는 cwd/json."""
    d = os.environ.get("JSON_DIR", os.path.join(os.getcwd(), "json"))
    os.makedirs(d, exist_ok=True)
    return d


class PositionGetterNode(Node):
    def __init__(self):
        super().__init__("position_getter_node")

        self.declare_parameter("get_posx_service", "/aux_control/get_current_posx")
        self.declare_parameter("get_posj_service", "/aux_control/get_current_posj")
        self.declare_parameter(
            "get_solution_space_service",
            "/aux_control/get_current_solution_space",
        )
        self.declare_parameter("jog_service", "/motion/jog")
        self.declare_parameter("jog_multi_service", "/motion/jog_multi")
        self.declare_parameter("move_joint_service", "/motion/move_joint")
        self.declare_parameter("move_line_service", "/motion/move_line")
        self.declare_parameter("move_stop_service", "/motion/move_stop")
        self.declare_parameter("set_robot_mode_service", "set_robot_mode")
        self.declare_parameter("parallel_axis2_service", "/force/parallel_axis2")
        self.declare_parameter("do_pick_and_place_topic", "/do_pick_and_place")
        self.declare_parameter("gripper_command_topic", DEFAULT_GRIPPER_COMMAND_TOPIC)
        
        # 파일 경로: json 폴더 사용 (~/.ros 아님). JSON_DIR env 또는 cwd/json
        storage_dir = _get_json_dir()
        self._saved_path = os.path.join(storage_dir, "saved_positions.json")

        # ROS 2 서비스 클라이언트 초기화
        self._get_posx = self.create_client(GetCurrentPosx, self._param_string("get_posx_service"))
        self._get_posj = self.create_client(GetCurrentPosj, self._param_string("get_posj_service"))
        self._get_solution_space = self.create_client(
            GetCurrentSolutionSpace, self._param_string("get_solution_space_service")
        )
        self._jog = self.create_client(Jog, self._param_string("jog_service"))
        self._jog_multi = self.create_client(JogMulti, self._param_string("jog_multi_service"))
        self._move_joint = self.create_client(MoveJoint, self._param_string("move_joint_service"))
        self._move_line = self.create_client(MoveLine, self._param_string("move_line_service"))
        self._move_stop = self.create_client(MoveStop, self._param_string("move_stop_service"))
        self._set_robot_mode = self.create_client(SetRobotMode, self._param_string("set_robot_mode_service"))
        self._parallel_axis2 = self.create_client(ParallelAxis2, self._param_string("parallel_axis2_service"))
        self._do_pick_and_place_pub = self.create_publisher(
            String,
            self._param_string("do_pick_and_place_topic"),
            10,
        )
        
        # 그리퍼 퍼블리셔
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            self._param_string("gripper_command_topic"),
            10,
        )
        self._last_gripper_open_percent = 50.0

        # GUI 메인 루프와 연동할 퓨처 관리
        self._pending_futures = []
        self._position_request_lock = threading.Lock()
        self._position_request_in_flight = False

    def _param_string(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _wait_for_services(self, timeout_sec=5.0):
        clients = [
            self._get_posx,
            self._get_posj,
            self._jog,
            self._jog_multi,
            self._move_joint,
            self._move_line,
            self._move_stop,
        ]
        for client in clients:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return False
        return True

    # ---------- 좌표 가져오기 (GUI 실시간 업데이트용) ----------
    def get_current_position(self, done_callback):
        """좌표 정보 및 Solution Space를 비동기로 가져와 콜백으로 반환.
        done_callback(posx, posj, solution_space, err)
        solution_space: 0~7 또는 None(조회 실패 시)
        """
        with self._position_request_lock:
            if self._position_request_in_flight:
                return
            self._position_request_in_flight = True

        def _call():
            try:
                req_x = GetCurrentPosx.Request()
                req_x.ref = 0  # Base Reference

                fut_x = self._get_posx.call_async(req_x)
                fut_j = self._get_posj.call_async(GetCurrentPosj.Request())
                fut_sol = self._get_solution_space.call_async(GetCurrentSolutionSpace.Request())

                rclpy.spin_until_future_complete(self, fut_x, timeout_sec=1.0)
                rclpy.spin_until_future_complete(self, fut_j, timeout_sec=1.0)
                rclpy.spin_until_future_complete(self, fut_sol, timeout_sec=1.0)

                posx, posj, err = None, None, None
                solution_space = None

                if fut_x.done() and fut_x.result() and fut_x.result().success:
                    posx = list(fut_x.result().task_pos_info[0].data)[:6]
                else:
                    err = "PosX Failed"

                if fut_j.done() and fut_j.result() and fut_j.result().success:
                    posj = list(fut_j.result().pos)[:6]
                else:
                    err = (err + " & PosJ Failed") if err else "PosJ Failed"

                if fut_sol.done() and fut_sol.result() and getattr(fut_sol.result(), "success", False):
                    solution_space = int(fut_sol.result().sol_space)

                self._pending_futures.append(
                    (None, lambda: done_callback(posx, posj, solution_space, err))
                )
            finally:
                with self._position_request_lock:
                    self._position_request_in_flight = False

        threading.Thread(target=_call, daemon=True).start()

    # ---------- 조깅 제어 (이미지 기반 새 GUI 대응) ----------
    def execute_jog(self, axis_index, speed_percent, ref=0):
        """
        axis_index: 0-5 (J1-J6), 6-11 (X,Y,Z,Rx,Ry,Rz)
        speed_percent: 방향이 포함된 속도 (-100 ~ 100)
        """
        if axis_index < 6:
            # Joint Jog
            req = Jog.Request()
            req.jog_axis = int(axis_index)
            req.move_reference = int(ref)
            req.speed = float(speed_percent)
            self._jog.call_async(req)
        else:
            # Task Jog (JogMulti 사용)
            req = JogMulti.Request()
            vec = [0.0] * 6
            vec[axis_index - 6] = 1.0 if speed_percent > 0 else -1.0
            req.jog_axis = vec
            req.move_reference = int(ref)
            req.speed = float(abs(speed_percent))
            self._jog_multi.call_async(req)

    def move_stop_soft(self):
        req = MoveStop.Request()
        req.stop_mode = 2  # 1: Quick, 2: Soft
        self._move_stop.call_async(req)

    def parallel_axis2_align(self, axis, ref=0, done_callback=None):
        """
        툴 축을 기준 좌표계의 X/Y/Z 방향과 평행하게 맞춤 (TCP 위치 유지, 자세만 변경).
        axis: 0=X, 1=Y, 2=Z (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z)
        ref: 0=DR_BASE, 2=DR_WORLD
        """
        if not self._parallel_axis2.service_is_ready():
            if done_callback:
                self._pending_futures.append((None, lambda: done_callback(False)))
            return
        vect = [0.0, 0.0, 0.0]
        vect[axis] = -1.0 if axis == 2 else 1.0  # Z는 -1, X/Y는 +1
        req = ParallelAxis2.Request()
        req.vect = vect
        req.axis = int(axis)
        req.ref = int(ref)
        fut = self._parallel_axis2.call_async(req)
        if done_callback:
            def _done(f):
                res = f.result()
                ok = bool(res and getattr(res, "success", False))
                self._pending_futures.append((None, lambda: done_callback(ok)))

            fut.add_done_callback(_done)

    def trigger_do_pick_and_place(self, go_text: str = "go"):
        """pick_and_place 노드가 시나리오를 실행하도록 /do_pick_and_place 에 한 번 publish."""
        msg = String()
        msg.data = go_text
        self._do_pick_and_place_pub.publish(msg)

    # ---------- 로봇 모드 및 그리퍼 ----------
    def go_home(self, done_callback=None):
        """DEFAULT_HOME_POSJ로 MoveJoint 이동."""
        self.go_to_posj(DEFAULT_HOME_POSJ, done_callback=done_callback)

    def go_to_posj(self, posj_list, done_callback=None, vel=50.0, acc=50.0, time_sec=0.0):
        """조인트 각도 리스트(6개, deg)로 이동. posj_list: [j1..j6]."""
        pos = [float(posj_list[i]) if i < len(posj_list) else 0.0 for i in range(6)]
        req = MoveJoint.Request()
        req.pos = pos
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = float(time_sec)
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        fut = self._move_joint.call_async(req)
        if done_callback:
            def _done(f):
                res = f.result()
                ok = bool(res and getattr(res, "success", False))
                self._pending_futures.append((None, lambda: done_callback(ok)))
            fut.add_done_callback(_done)

    def go_to_posx(self, posx_list, done_callback=None, vel=None, acc=None, time_sec=8.0):
        """task 좌표 리스트(6개: x,y,z,rx,ry,rz mm, deg)로 직선 이동."""
        pos = [float(posx_list[i]) if i < len(posx_list) else 0.0 for i in range(6)]
        if vel is None:
            vel = [10.0, 10.0]
        if acc is None:
            acc = [10.0, 10.0]
        req = MoveLine.Request()
        req.pos = pos
        req.vel = list(vel)
        req.acc = list(acc)
        req.time = float(time_sec)
        req.radius = 0.0
        req.ref = 0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        fut = self._move_line.call_async(req)
        if done_callback:
            def _done(f):
                res = f.result()
                ok = bool(res and getattr(res, "success", False))
                self._pending_futures.append((None, lambda: done_callback(ok)))
            fut.add_done_callback(_done)

    def set_robot_mode(self, mode_value, done_callback=None):
        req = SetRobotMode.Request()
        req.robot_mode = int(mode_value)
        fut = self._set_robot_mode.call_async(req)
        if done_callback:
            def _done(f):
                res = f.result()
                ok = res.success if res else False
                msg = "Mode Changed" if ok else "Mode Change Failed"
                self._pending_futures.append((None, lambda: done_callback(ok, msg)))
            fut.add_done_callback(_done)

    def set_gripper_open_percent(self, open_percent: float) -> bool:
        if self._gripper_pub.get_subscription_count() == 0:
            return False
        p = max(0.0, min(100.0, float(open_percent)))
        self._last_gripper_open_percent = p
        # open percent: 100=open, 0=closed/grip.
        pos = GRIPPER_CLOSED_POS - (p / 100.0) * (GRIPPER_CLOSED_POS - GRIPPER_OPEN_POS)
        msg = Float64MultiArray()
        msg.data = [pos]
        self._gripper_pub.publish(msg)
        self.get_logger().info("Gripper command: %.1f%% open -> joint %.3f" % (p, pos))
        return True

    def get_current_gripper_open_percent(self):
        return self._last_gripper_open_percent

    # ---------- 위치 저장 및 시나리오 (JSON) ----------
    def save_position(self, save_as="both"):
        """현재 위치/그리퍼를 파일에 추가 저장.
        save_as: 'posx', 'posj', 'both', 'gripper'
        """
        save_as = (save_as or "both").lower()
        px = pj = None

        if save_as == "both":
            success, px, pj = self._sync_get_all_pos()
            if not success:
                return False, None, None
        elif save_as == "posx":
            req_x = GetCurrentPosx.Request(); req_x.ref = 0
            fut_x = self._get_posx.call_async(req_x)
            rclpy.spin_until_future_complete(self, fut_x, timeout_sec=2.0)
            res_x = fut_x.result()
            if not (res_x and getattr(res_x, "success", False) and getattr(res_x, "task_pos_info", None)):
                return False, None, None
            px = list(res_x.task_pos_info[0].data)[:6]
        elif save_as == "posj":
            fut_j = self._get_posj.call_async(GetCurrentPosj.Request())
            rclpy.spin_until_future_complete(self, fut_j, timeout_sec=2.0)
            res_j = fut_j.result()
            if not (res_j and getattr(res_j, "success", False)):
                return False, None, None
            pj = list(res_j.pos)[:6]
        elif save_as == "gripper":
            # 포즈는 저장하지 않고, 현재 그리퍼만 저장
            pass
        else:
            # 알 수 없는 값이면 both로 처리
            success, px, pj = self._sync_get_all_pos()
            if not success:
                return False, None, None

        data = self._load_file(self._saved_path)
        gr = self._last_gripper_open_percent if save_as in ("gripper", "both") else None
        data.append({"posx": px, "posj": pj, "gripper": gr})
        self._save_file(self._saved_path, data)
        return True, px, pj

    def _sync_get_all_pos(self):
        """내부용 동기 좌표 획득 (스레드 내부에서만 호출)"""
        req_x = GetCurrentPosx.Request(); req_x.ref = 0
        fut_x = self._get_posx.call_async(req_x)
        fut_j = self._get_posj.call_async(GetCurrentPosj.Request())
        rclpy.spin_until_future_complete(self, fut_x, timeout_sec=2.0)
        rclpy.spin_until_future_complete(self, fut_j, timeout_sec=2.0)
        
        if fut_x.done() and fut_j.done():
            px = list(fut_x.result().task_pos_info[0].data)[:6]
            pj = list(fut_j.result().pos)[:6]
            return True, px, pj
        return False, None, None

    def list_saved(self):
        return list(range(len(self._load_file(self._saved_path))))

    def get_saved_labels(self):
        """저장 목록 표시용 라벨: 'Position 1', 'Gripper 1', 'Position 2' ..."""
        data = self._load_file(self._saved_path)
        labels = []
        pos_n, gr_n = 0, 0
        for item in data:
            has_pos = item.get("posx") is not None or item.get("posj") is not None
            has_gr = item.get("gripper") is not None
            if not has_pos and has_gr:
                gr_n += 1
                labels.append(f"Gripper {gr_n}")
            else:
                pos_n += 1
                labels.append(f"Position {pos_n}")
        return labels

    def load_position(self, index):
        """해당 인덱스 데이터 반환. 반환: dict with posx, posj, gripper, kind."""
        data = self._load_file(self._saved_path)
        if 0 <= index < len(data):
            item = data[index]
            px, pj, gr = item.get("posx"), item.get("posj"), item.get("gripper")
            has_posx = px is not None
            has_posj = pj is not None
            if not has_posx and not has_posj and gr is not None:
                kind = "gripper"
            elif has_posx and has_posj:
                kind = "both"
            elif has_posx:
                kind = "posx"
            else:
                kind = "posj"
            return {"posx": px, "posj": pj, "gripper": gr, "kind": kind}
        return None

    def clear_saved(self):
        """모든 저장된 포지션 삭제."""
        self._save_file(self._saved_path, [])

    def delete_position(self, index: int) -> bool:
        """지정 인덱스의 포지션 하나만 삭제."""
        data = self._load_file(self._saved_path)
        if 0 <= index < len(data):
            data.pop(index)
            self._save_file(self._saved_path, data)
            return True
        return False

    # ---------- 유틸리티 ----------
    def _load_file(self, path):
        if not os.path.isfile(path): return []
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except: return []

    def _save_file(self, path, data):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def spin_once_and_drain_futures(self):
        """Tkinter 메인 루프에서 호출되어 ROS 이벤트를 처리함"""
        rclpy.spin_once(self, timeout_sec=0)
        if self._pending_futures:
            for i in range(len(self._pending_futures) - 1, -1, -1):
                fut, cb = self._pending_futures[i]
                if fut is None or fut.done():
                    self._pending_futures.pop(i)
                    cb()

def main(args=None):
    rclpy.init(args=args)
    node = PositionGetterNode()
    if not node._wait_for_services():
        node.get_logger().error("Robot Services Unavailable.")
        return

    import tkinter as tk
    from .position_getter_gui import run_gui # 실제 패키지 경로에 맞춰 수정 필요
    root = tk.Tk()
    run_gui(node, root)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
