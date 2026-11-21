"""Tk GUI for JSON trajectory practice in the isolated MuJoCo ROS domain."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time

from .trajectory import JOINT_NAMES, Trajectory, load_trajectory, save_for_gripper
from .home_pose import HOME_JOINT_DEG, plan_sim_home

GRIPPER_LABEL_TO_CODE = {"None": "none", "AG145": "ag145", "mDPG": "mdpg_1s1"}
GRIPPER_CODE_TO_LABEL = {code: label for label, code in GRIPPER_LABEL_TO_CODE.items()}


def _base_stem(stem: str) -> str:
    stem = re.sub(r"_\d{3}$", "", stem)
    for gripper in GRIPPER_CODE_TO_LABEL:
        suffix = f"_{gripper}"
        if stem.endswith(suffix):
            return stem[:-len(suffix)]
    return stem


def _next_available(path: Path) -> Path:
    if not path.exists():
        return path
    for index in range(1, 1000):
        candidate = path.with_name(f"{path.stem}_{index:03d}{path.suffix}")
        if not candidate.exists():
            return candidate
    raise OSError("Too many files with the same name")


class PracticeGui:
    def __init__(self, initial_file: str, real_domain_id: int, sim_gripper_type: str):
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self.tk = tk
        self.filedialog = filedialog
        self.messagebox = messagebox
        self.rclpy = rclpy
        self.Float64MultiArray = Float64MultiArray
        self.real_domain_id = real_domain_id
        self.sim_gripper_type = sim_gripper_type
        self.events: queue.Queue = queue.Queue()
        self.commands: queue.Queue = queue.Queue()
        self.stopping = threading.Event()
        self.latest = None
        self.trajectory: Trajectory | None = None
        self.last_rt_report: dict | None = None
        self.last_rt_checked_at: float | None = None
        self.running = False
        self.real_process: subprocess.Popen | None = None
        self.active_real_gripper: str | None = None
        self.batch_gui_process: subprocess.Popen | None = None

        rclpy.init()
        self.node = Node(
            "sim_real_gap_trajectory_gui",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.publisher = self.node.create_publisher(Float64MultiArray, "/dsr_position_controller/commands", 10)

        def on_joint_state(msg: JointState):
            by_name = {name: index for index, name in enumerate(msg.name)}
            if any(name not in by_name for name in JOINT_NAMES):
                return
            try:
                q = tuple(float(msg.position[by_name[name]]) for name in JOINT_NAMES)
                dq = tuple(float(msg.velocity[by_name[name]]) for name in JOINT_NAMES)
            except (IndexError, TypeError, ValueError):
                return
            if all(math.isfinite(value) for value in (*q, *dq)):
                self.latest = (time.monotonic(), q, dq)

        self.subscription = self.node.create_subscription(JointState, "/joint_states", on_joint_state, 10)
        self.worker = threading.Thread(target=self._ros_loop, daemon=True)
        self.worker.start()

        self.root = tk.Tk()
        self.root.title("A0509 Sim → Real Gap Practice")
        self.root.geometry("820x670")
        frame = ttk.Frame(self.root, padding=14)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="JSON trajectory (joint_1 … joint_6, rad)").pack(anchor="w")
        path_row = ttk.Frame(frame)
        path_row.pack(fill="x", pady=(4, 8))
        self.path_var = tk.StringVar(value=initial_file)
        ttk.Entry(path_row, textvariable=self.path_var).pack(side="left", fill="x", expand=True)
        ttk.Button(path_row, text="Browse", command=self._browse).pack(side="left", padx=(6, 0))
        ttk.Button(path_row, text="Load / validate", command=self._load).pack(side="left", padx=(6, 0))
        self.summary_var = tk.StringVar(value="Choose a trajectory JSON file.")
        ttk.Label(frame, textvariable=self.summary_var).pack(anchor="w", pady=(0, 8))
        tool_row = ttk.Frame(frame)
        tool_row.pack(fill="x", pady=(0, 8))
        ttk.Label(tool_row, text="Gripper option:").pack(side="left")
        self.gripper_var = tk.StringVar(value=GRIPPER_CODE_TO_LABEL[sim_gripper_type])
        self.gripper_combo = ttk.Combobox(tool_row, textvariable=self.gripper_var, state="readonly", width=12,
                                          values=list(GRIPPER_LABEL_TO_CODE))
        self.gripper_combo.pack(side="left", padx=(8, 10))
        self.gripper_combo.bind("<<ComboboxSelected>>", self._on_gripper_changed)
        ttk.Button(tool_row, text="Save gripper JSON", command=self._save_gripper_json).pack(side="left")
        ttk.Label(tool_row, text=f"MuJoCo model: {GRIPPER_CODE_TO_LABEL[sim_gripper_type]}").pack(side="left", padx=(18, 0))
        folder_row = ttk.Frame(frame)
        folder_row.pack(fill="x", pady=(0, 8))
        ttk.Label(folder_row, text="Save folder:").pack(side="left")
        self.output_dir_var = tk.StringVar(value=str(Path.cwd() / "results" / "sim_real_gap_getting"))
        ttk.Entry(folder_row, textvariable=self.output_dir_var).pack(side="left", fill="x", expand=True, padx=(8, 6))
        ttk.Button(folder_row, text="Choose folder", command=self._choose_output_folder).pack(side="left")
        controls = ttk.Frame(frame)
        controls.pack(fill="x", pady=(0, 8))
        ttk.Button(controls, text="Run in simulation", command=self._start).pack(side="left")
        ttk.Button(controls, text="Go Home in simulation", command=self._go_home_sim).pack(side="left", padx=(8, 0))
        ttk.Button(controls, text="Stop / hold", command=self._stop).pack(side="left", padx=(8, 0))
        ttk.Button(controls, text="Check real RT topics", command=self._check_real).pack(side="left", padx=(24, 0))
        ttk.Button(controls, text="Save RT report", command=self._save_rt_report).pack(side="left", padx=(8, 0))
        batch_row = ttk.Frame(frame)
        batch_row.pack(fill="x", pady=(0, 8))
        ttk.Button(batch_row, text="Collect F1–F4 data (MuJoCo)",
                   command=self._open_front_batch).pack(side="left")
        ttk.Label(batch_row, text="Opens a separate batch window; saves compressed 100 Hz data",
                  foreground="#875600").pack(side="left", padx=(12, 0))
        real_controls = ttk.Frame(frame)
        real_controls.pack(fill="x", pady=(0, 8))
        ttk.Button(real_controls, text="Run on real (100 Hz)", command=self._run_real).pack(side="left")
        ttk.Button(real_controls, text="Stop real", command=self._stop_real).pack(side="left", padx=(8, 0))
        ttk.Label(frame, text=f"Practice Home (joint deg): {HOME_JOINT_DEG}").pack(anchor="w", pady=(0, 4))
        self.status_var = tk.StringVar(value="Waiting for MuJoCo joint states and controller…")
        ttk.Label(frame, textvariable=self.status_var, wraplength=715).pack(anchor="w", pady=(0, 8))
        ttk.Label(frame, text="Real RT field check (read only)").pack(anchor="w")
        self.report = tk.Text(frame, height=15, width=90, state="disabled")
        self.report.pack(fill="both", expand=True, pady=(4, 0))
        self._append("Real RT runs in ROS_DOMAIN_ID=" + str(real_domain_id) + ". Start real_rt.launch.py in that domain first.\n")
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(100, self._poll)
        if initial_file:
            self._load()

    def _append(self, message: str) -> None:
        self.report.configure(state="normal")
        self.report.insert("end", message)
        self.report.see("end")
        self.report.configure(state="disabled")

    def _browse(self) -> None:
        if self._real_running():
            self.status_var.set("Wait for Real execution to stop before changing the JSON.")
            return
        selected = self.filedialog.askopenfilename(filetypes=[("JSON trajectory", "*.json")])
        if selected:
            self.path_var.set(selected)
            self._load()

    def _load(self) -> None:
        if self._real_running():
            self.status_var.set("Wait for Real execution to stop before loading another trajectory.")
            return
        if self.running:
            self.status_var.set("Stop simulation playback before loading another trajectory.")
            return
        self.last_rt_report = None
        self.last_rt_checked_at = None
        try:
            trajectory = load_trajectory(self.path_var.get())
        except (OSError, ValueError, json.JSONDecodeError) as error:
            self.trajectory = None
            self.status_var.set(f"JSON validation failed: {error}")
            return
        self.trajectory = trajectory
        self.last_rt_report = None
        self.last_rt_checked_at = None
        if trajectory.gripper_type is not None:
            self.gripper_var.set(GRIPPER_CODE_TO_LABEL[trajectory.gripper_type])
        self.summary_var.set(f"{trajectory.name}  |  {trajectory.duration_s:.2f} s  |  "
                             f"{len(trajectory.points)} JSON points  |  {len(trajectory.samples)} checked samples")
        self.status_var.set("Trajectory passed sampled limits. Real execution needs a fresh RT check and operator confirmation.")

    def _selected_gripper(self) -> str:
        return GRIPPER_LABEL_TO_CODE[self.gripper_var.get()]

    def _real_running(self) -> bool:
        return self.real_process is not None and self.real_process.poll() is None

    def _choose_output_folder(self) -> None:
        if self._real_running():
            self.status_var.set("Wait for Real execution to stop before changing the save folder.")
            return
        selected = self.filedialog.askdirectory(
            title="Choose save folder", initialdir=self.output_dir_var.get() or str(Path.cwd()))
        if selected:
            self.output_dir_var.set(selected)
            self.status_var.set(f"Save folder: {selected}")

    def _output_folder(self, gripper: str) -> Path:
        raw = self.output_dir_var.get().strip()
        if not raw:
            raise ValueError("Choose a save folder first")
        folder = Path(raw).expanduser().resolve() / gripper
        folder.mkdir(parents=True, exist_ok=True)
        return folder

    def _on_gripper_changed(self, _event=None) -> None:
        if self._real_running():
            self.gripper_var.set(GRIPPER_CODE_TO_LABEL[self.active_real_gripper])
            self.status_var.set("Wait for Real execution to stop before changing the gripper.")
            return
        self.last_rt_report = None
        self.last_rt_checked_at = None
        self.status_var.set("Gripper option changed. Save a matching JSON before the Real RT check.")

    def _save_gripper_json(self) -> None:
        if self._real_running():
            self.status_var.set("Wait for Real execution to stop before saving another JSON.")
            return
        if self.running:
            self.status_var.set("Stop simulation playback before saving the trajectory.")
            return
        if self.trajectory is None:
            self._load()
        if self.trajectory is None:
            return
        gripper = self._selected_gripper()
        try:
            folder = self._output_folder(gripper)
            target = _next_available(folder / f"{_base_stem(self.trajectory.source.stem)}_{gripper}.json")
            saved = save_for_gripper(self.trajectory, target, gripper)
            self.trajectory = load_trajectory(saved)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            self.status_var.set(f"Could not save gripper JSON: {error}")
            return
        self.path_var.set(str(saved))
        self.last_rt_report = None
        self.last_rt_checked_at = None
        self.status_var.set(f"Saved {GRIPPER_CODE_TO_LABEL[gripper]} trajectory: {saved}")

    def _save_rt_report(self) -> None:
        if self.last_rt_report is None:
            self.status_var.set("Check real RT topics before saving a report.")
            return
        gripper = self._selected_gripper()
        if self.last_rt_report.get("selected_gripper_type") != gripper:
            self.status_var.set("Gripper option changed; check real RT topics again before saving.")
            return
        try:
            folder = self._output_folder(gripper)
            stem = _base_stem(self.trajectory.source.stem) if self.trajectory else "real_rt"
            target = _next_available(folder / f"{stem}_{gripper}_rt_report.json")
            with target.open("w", encoding="utf-8") as stream:
                json.dump(self.last_rt_report, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
        except (OSError, ValueError) as error:
            self.status_var.set(f"Could not save RT report: {error}")
            return
        self.status_var.set(f"Saved RT report: {target}")

    def _start(self) -> None:
        if self._real_running():
            self.status_var.set("Stop Real execution before starting simulation playback.")
            return
        if self.trajectory is None:
            self._load()
        if self.trajectory is not None:
            selected = self._selected_gripper()
            if selected != self.sim_gripper_type:
                self.status_var.set("Selected gripper differs from the running MuJoCo model; relaunch practice.launch.py with matching gripper_type.")
                return
            if self.trajectory.gripper_type is not None and self.trajectory.gripper_type != selected:
                self.status_var.set("JSON gripper_type differs from the option; save a matching gripper JSON first.")
                return
            self.commands.put(("start", self.trajectory))

    def _open_front_batch(self) -> None:
        if self.batch_gui_process is not None and self.batch_gui_process.poll() is None:
            self.status_var.set("F1–F4 batch collector is already open in another window.")
            return
        selected = self._selected_gripper()
        initial_tool = selected if selected != "none" else "ag145"
        try:
            self.batch_gui_process = subprocess.Popen(
                [sys.executable, "-m", "sim_real_gap_getting.front_batch_gui",
                 "--tool", initial_tool])
        except OSError as error:
            self.status_var.set(f"Could not open F1–F4 batch collector: {error}")
            return
        self.status_var.set("Opened separate F1–F4 MuJoCo data collector window.")

    def _stop(self) -> None:
        self.commands.put(("stop", None))

    def _go_home_sim(self) -> None:
        if self._real_running():
            self.status_var.set("Stop Real execution before moving the simulator Home.")
            return
        if self.running:
            self.status_var.set("Stop simulation playback before moving Home.")
            return
        self.commands.put(("home", None))

    def _hold_current(self) -> None:
        if self.latest is not None and time.monotonic() - self.latest[0] <= 0.5:
            self.publisher.publish(self.Float64MultiArray(data=list(self.latest[1])))

    def _ros_loop(self) -> None:
        active = None
        start_sim = None
        last_sim = None
        last_clock_wall = None
        next_index = 0
        while not self.stopping.is_set():
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            try:
                while True:
                    command, payload = self.commands.get_nowait()
                    if command == "stop":
                        self._hold_current()
                        active = None
                        self.events.put(("running", False))
                        self.events.put(("status", "Simulation stopped; current measured pose commanded as hold."))
                    elif command in ("start", "home"):
                        if active is not None:
                            self.events.put(("status", "Trajectory is already running."))
                            continue
                        latest = self.latest
                        graph = dict(self.node.get_topic_names_and_types())
                        real_publishers = any(
                            topic.startswith("/rt_topic/") and self.node.get_publishers_info_by_topic(topic)
                            for topic in graph)
                        if real_publishers:
                            self.events.put(("status", "RT topics detected in simulation domain; playback blocked."))
                        elif self.publisher.get_subscription_count() == 0:
                            self.events.put(("status", "MuJoCo position controller is not subscribed yet."))
                        elif latest is None or time.monotonic() - latest[0] > 0.5:
                            self.events.put(("status", "Fresh /joint_states data is required."))
                        elif max(abs(value) for value in latest[2]) > 0.05:
                            self.events.put(("status", "Robot must be stationary before playback."))
                        elif command == "start" and max(abs(a-b) for a, b in zip(latest[1], payload.samples[0].q_rad)) > 0.05:
                            self.events.put(("status", "Current pose differs from JSON start by over 0.05 rad. Move the simulator to the start pose first."))
                        else:
                            if command == "home":
                                try:
                                    payload = plan_sim_home(latest[1])
                                except ValueError as error:
                                    self.events.put(("status", f"Cannot plan simulation Home: {error}"))
                                    continue
                            now = self.node.get_clock().now().nanoseconds / 1e9
                            if now <= 0:
                                self.events.put(("status", "Waiting for MuJoCo /clock."))
                                continue
                            active = payload
                            start_sim = now
                            last_sim = now
                            last_clock_wall = time.monotonic()
                            next_index = 0
                            self.events.put(("running", True))
                            self.events.put(("status", f"Running {payload.name} in MuJoCo."))
            except queue.Empty:
                pass

            if active is None:
                continue
            now = self.node.get_clock().now().nanoseconds / 1e9
            if now != last_sim:
                last_sim = now
                last_clock_wall = time.monotonic()
            if time.monotonic() - last_clock_wall > 2.0 or self.publisher.get_subscription_count() == 0:
                self._hold_current()
                active = None
                self.events.put(("running", False))
                self.events.put(("status", "MuJoCo clock/controller stopped; playback aborted."))
                continue
            elapsed = now - start_sim
            if elapsed < -0.01 or (next_index < len(active.samples) and
                                   elapsed - active.samples[next_index].time_s > 0.2):
                self._hold_current()
                active = None
                self.events.put(("running", False))
                self.events.put(("status", "Simulation clock jumped; playback aborted."))
                continue
            if next_index < len(active.samples) and elapsed >= active.samples[next_index].time_s:
                while next_index+1 < len(active.samples) and elapsed >= active.samples[next_index+1].time_s:
                    next_index += 1
                self.publisher.publish(self.Float64MultiArray(data=list(active.samples[next_index].q_rad)))
                next_index += 1
            if next_index >= len(active.samples):
                active = None
                self.events.put(("running", False))
                self.events.put(("status", "Simulation trajectory complete; final target held."))

    def _check_real(self) -> None:
        selected_gripper = self._selected_gripper()
        trajectory = self.trajectory
        if trajectory is not None and trajectory.gripper_type != selected_gripper:
            self.status_var.set("Save gripper JSON with the selected option before the Real RT check.")
            return
        self.last_rt_report = None
        self.last_rt_checked_at = None
        self.status_var.set("Checking real RT topics in the separate real ROS domain…")

        def run():
            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = str(self.real_domain_id)
            try:
                command = [sys.executable, "-m", "sim_real_gap_getting.rt_check", "--timeout", "5"]
                if trajectory is not None:
                    command.extend(["--trajectory", str(trajectory.source)])
                result = subprocess.run(
                    command,
                    capture_output=True, text=True, timeout=12, env=env, check=False,
                )
                if not result.stdout:
                    raise RuntimeError(result.stderr.strip() or f"RT check exited {result.returncode}")
                report = json.loads(result.stdout)
                report["selected_gripper_type"] = selected_gripper
                report["physical_tool_verified"] = False
                report["trajectory_file"] = str(trajectory.source) if trajectory else None
                lines = [f"ROS_DOMAIN_ID={self.real_domain_id}  all fields: "
                         f"{'OK' if report['all_fields_available'] else 'INCOMPLETE'}  "
                         f"gripper={GRIPPER_CODE_TO_LABEL[selected_gripper]}"]
                for field, detail in report["fields"].items():
                    lines.append(f"{field:25} {detail['status']:10} "
                                 f"topic={'yes' if detail['topic_present'] else 'no'}  "
                                 f"{detail['source'] or ''}  {detail['values'] or ''}")
                start = report.get("trajectory_start")
                if start:
                    lines.append(f"JSON start: {start['status']}  "
                                 f"pose match={start['start_pose_match']}  stationary={start['stationary']}")
                    if "max_start_error_rad" in start:
                        lines.append(f"max start error={start['max_start_error_rad']:.4f} rad  "
                                     f"max speed={start['max_speed_rad_s']:.4f} rad/s")
                lines.append("actual_joint_torque is a controller estimate; review its meaning before analysis.")
                lines.append("Selected gripper is a file label; physical attachment is not verified by RT topics.")
                self.events.put(("rt_report", report))
                self.events.put(("report", "\n".join(lines) + "\n\n"))
                self.events.put(("status", "Real RT check finished. No real motion command was sent."))
            except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
                self.events.put(("status", f"Real RT check failed: {error}"))

        threading.Thread(target=run, daemon=True).start()


    def _run_real(self) -> None:
        trajectory = self.trajectory
        selected = self._selected_gripper()
        if trajectory is None or trajectory.gripper_type != selected:
            self.status_var.set("Save a JSON for the selected gripper before Real execution.")
            return
        if self.running:
            self.status_var.set("Stop simulation playback before Real execution.")
            return
        if self.real_process is not None and self.real_process.poll() is None:
            self.status_var.set("Real execution is already running.")
            return
        report = self.last_rt_report
        if (report is None or self.last_rt_checked_at is None or
                time.monotonic() - self.last_rt_checked_at > 15 or
                not report.get("all_fields_available") or
                report.get("trajectory_start", {}).get("status") != "review" or
                report.get("selected_gripper_type") != selected):
            self.status_var.set("Run a passing Real RT check within 15 seconds before execution.")
            return
        try:
            folder = self._output_folder(selected)
            from .real_execute import validate_real_plan
            samples = validate_real_plan(trajectory, selected)
        except (OSError, ValueError) as error:
            self.status_var.set(f"Real execution preflight failed: {error}")
            return
        confirmed = self.messagebox.askyesno(
            "Confirm real A0509 motion",
            f"This will move the REAL A0509 using {len(samples)} setpoints at 100 Hz "
            f"for {trajectory.duration_s:.2f} s.\n\n"
            f"Trajectory: {trajectory.source}\nGripper: {GRIPPER_CODE_TO_LABEL[selected]}\n\n"
            "Confirm that the physical gripper and payload are correct, the workspace is clear, "
            "the operator is supervising the robot, and the emergency stop is reachable. "
            "The GUI has no automatic workspace collision or physical tool verification.",
        )
        if not confirmed:
            self.status_var.set("Real execution cancelled.")
            return
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.real_domain_id)
        command = [sys.executable, "-m", "sim_real_gap_getting.real_execute",
                   "--trajectory", str(trajectory.source), "--gripper-type", selected,
                   "--output-dir", str(folder), "--operator-confirmed"]
        try:
            process = subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, bufsize=1)
        except OSError as error:
            self.status_var.set(f"Could not start Real runner: {error}")
            return
        self.real_process = process
        self.active_real_gripper = selected
        self.last_rt_report = None
        self.last_rt_checked_at = None
        self.status_var.set("Real execution preflight and 100 Hz runner started.")

        def watch():
            if process.stdout is not None:
                for line in process.stdout:
                    self.events.put(("report", line))
            code = process.wait()
            self.events.put(("real_done", code))

        threading.Thread(target=watch, daemon=True).start()

    def _stop_real(self) -> None:
        process = self.real_process
        if process is None or process.poll() is not None:
            self.status_var.set("No Real execution is running.")
            return
        process.send_signal(signal.SIGTERM)
        self.status_var.set("Real stop requested; waiting for quick-stop result.")

    def _poll(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "status":
                    self.status_var.set(value)
                elif kind == "report":
                    self._append(value)
                elif kind == "rt_report":
                    self.last_rt_report = value
                    self.last_rt_checked_at = time.monotonic()
                elif kind == "running":
                    self.running = value
                elif kind == "real_done":
                    self.real_process = None
                    self.active_real_gripper = None
                    self.status_var.set("Real execution finished." if value == 0 else
                                        f"Real execution stopped or failed (exit {value}); see log above.")
        except queue.Empty:
            pass
        if not self.stopping.is_set():
            self.root.after(100, self._poll)

    def _close(self) -> None:
        if self.real_process is not None and self.real_process.poll() is None:
            self._stop_real()
            self.status_var.set("Wait for Real stop to finish before closing this window.")
            return
        self.stopping.set()
        self.worker.join(timeout=2)
        self._hold_current()
        self.node.destroy_node()
        self.rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="A0509 JSON trajectory simulation GUI")
    parser.add_argument("--trajectory", default="")
    parser.add_argument("--real-domain-id", type=int, default=0)
    parser.add_argument("--gripper-type", choices=("none", "ag145", "mdpg_1s1"), default="none")
    args = parser.parse_args(argv)
    sim_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if not 0 <= args.real_domain_id <= 232 or sim_domain == args.real_domain_id:
        parser.error("real ROS domain must differ from the current simulation domain")
    PracticeGui(args.trajectory, args.real_domain_id, args.gripper_type).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
