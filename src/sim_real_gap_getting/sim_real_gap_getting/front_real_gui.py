"""Operator-controlled GUI for real A0509 F1-F4 RT data collection."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import queue
import threading

from .front_gripper_audit import _default_trajectory_dir
from .front_real_batch import prepare_front_real_batch, run_prepared_real_batch
from .front_trajectory import PROFILE_NAMES
from .lr_trajectory import _scene_path


LABELS = {"AG145": "ag145", "mDPG": "mdpg_1s1"}


class FrontRealGui:
    def __init__(self, *, initial_tool: str = "mdpg_1s1", lock_tool: bool = False,
                 initial_mount_xyz: str = "0 0 0", initial_mount_rpy: str = "0 0 0"):
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk

        self.tk, self.ttk = tk, ttk
        self.filedialog, self.messagebox = filedialog, messagebox
        self.root = tk.Tk()
        self.root.title("A0509 실기 F1~F4 수집 · 실제 로봇 동작")
        self.root.geometry("1020x850")
        self.events: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.preflight: Path | None = None
        self.preflight_ready = False
        self.lock_tool = lock_tool

        frame = ttk.Frame(self.root, padding=15)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="실제 A0509 · F1~F4 데이터 수집",
                  font=("Malgun Gothic", 19, "bold")).pack(anchor="w")
        ttk.Label(frame, text="실기 동작: 모델 검증 → 작업자 확인 → 준비 이동 1회 → 선택 경로 반복 → RT 데이터 저장",
                  foreground="#a42b1e").pack(anchor="w", pady=(0, 12))

        def row(label, variable, browse=None):
            box = ttk.Frame(frame)
            box.pack(fill="x", pady=(0, 6))
            ttk.Label(box, text=label, width=20).pack(side="left")
            entry = ttk.Entry(box, textvariable=variable)
            entry.pack(side="left", fill="x", expand=True, padx=(0, 8))
            button = None
            if browse is not None:
                button = ttk.Button(box, text="선택", command=browse)
                button.pack(side="left")
            return entry, button

        self.source_var = tk.StringVar(value=str(_default_trajectory_dir()))
        self.scene_var = tk.StringVar(value=str(_scene_path()))
        self.output_var = tk.StringVar(value=str(Path.cwd() / "results/front_real_batch"))
        self.source_entry, self.source_button = row("F1~F4 궤적 폴더", self.source_var,
                                                     lambda: self._browse_dir(self.source_var))
        self.scene_entry, self.scene_button = row("충돌 검사 MuJoCo 장면", self.scene_var,
                                                   self._browse_scene)
        self.output_entry, self.output_button = row("실기 데이터 저장 폴더", self.output_var,
                                                     lambda: self._browse_dir(self.output_var))

        options = ttk.Frame(frame)
        options.pack(fill="x", pady=(6, 8))
        ttk.Label(options, text="실제 장착 그리퍼", width=20).pack(side="left")
        label = next((key for key, value in LABELS.items() if value == initial_tool), "mDPG")
        self.tool_var = tk.StringVar(value=label)
        self.tool_combo = ttk.Combobox(options, textvariable=self.tool_var,
                                       values=list(LABELS),
                                       state="disabled" if lock_tool else "readonly", width=13)
        self.tool_combo.pack(side="left", padx=(0, 22))
        ttk.Label(options, text="경로별 반복").pack(side="left", padx=(0, 6))
        self.repeats_var = tk.StringVar(value="1")
        self.repeat_box = ttk.Spinbox(options, from_=1, to=10, textvariable=self.repeats_var, width=5)
        self.repeat_box.pack(side="left", padx=(0, 22))
        ttk.Label(options, text="Real ROS_DOMAIN_ID").pack(side="left", padx=(0, 6))
        self.domain_var = tk.StringVar(value=os.environ.get("ROS_DOMAIN_ID", "0"))
        self.domain_entry = ttk.Entry(options, textvariable=self.domain_var, width=5)
        self.domain_entry.pack(side="left")

        profiles = ttk.Frame(frame)
        profiles.pack(fill="x", pady=(0, 8))
        ttk.Label(profiles, text="실행할 경로", width=20).pack(side="left")
        self.profile_vars = {name: tk.BooleanVar(value=True) for name in PROFILE_NAMES}
        self.profile_checks = []
        for name in PROFILE_NAMES:
            check = ttk.Checkbutton(profiles, text=name.split("_", 1)[0].upper(),
                                    variable=self.profile_vars[name])
            check.pack(side="left", padx=(0, 18))
            self.profile_checks.append(check)

        geometry = ttk.LabelFrame(frame, text="실제 공간 측정값 · DR_BASE 좌표계", padding=8)
        geometry.pack(fill="x", pady=(0, 8))
        measures = ttk.Frame(geometry)
        measures.pack(fill="x")
        self.y_min_var, self.y_max_var, self.z_var = tk.StringVar(), tk.StringVar(), tk.StringVar()
        self.geometry_entries = []
        for title, var in (("Y 최소 mm", self.y_min_var), ("Y 최대 mm", self.y_max_var),
                           ("작업대 상면 Z mm", self.z_var)):
            ttk.Label(measures, text=title).pack(side="left", padx=(0, 5))
            entry = ttk.Entry(measures, textvariable=var, width=11)
            entry.pack(side="left", padx=(0, 17))
            self.geometry_entries.append(entry)
        ttk.Label(geometry, text="총폭 400 mm만으로는 경계 위치를 알 수 없습니다. 실제 측정한 세 값을 입력하세요.",
                  foreground="#7d4b13").pack(anchor="w", pady=(5, 0))

        mount = ttk.Frame(frame)
        mount.pack(fill="x", pady=(0, 8))
        ttk.Label(mount, text="mDPG 장착 변환", width=20).pack(side="left")
        self.mount_xyz_var = tk.StringVar(value=initial_mount_xyz)
        self.mount_rpy_var = tk.StringVar(value=initial_mount_rpy)
        ttk.Label(mount, text="XYZ m").pack(side="left")
        self.mount_xyz_entry = ttk.Entry(mount, textvariable=self.mount_xyz_var, width=18)
        self.mount_xyz_entry.pack(side="left", padx=(5, 18))
        ttk.Label(mount, text="RPY rad").pack(side="left")
        self.mount_rpy_entry = ttk.Entry(mount, textvariable=self.mount_rpy_var, width=18)
        self.mount_rpy_entry.pack(side="left", padx=(5, 10))
        ttk.Label(mount, text="0은 기존 모델 가정값", foreground="#7d4b13").pack(side="left")

        self.tool_confirmed = tk.BooleanVar(value=False)
        self.obstacles_confirmed = tk.BooleanVar(value=False)
        check1 = ttk.Checkbutton(frame, variable=self.tool_confirmed,
                                 text="실제 그리퍼·어댑터 장착 위치가 입력한 모델과 일치함을 확인")
        check1.pack(anchor="w")
        check2 = ttk.Checkbutton(frame, variable=self.obstacles_confirmed,
                                 text="전방·하부·케이블 등 나머지 장애물을 확인하고 작업 영역을 비움")
        check2.pack(anchor="w", pady=(0, 9))
        self.operator_checks = [check1, check2]

        actions = ttk.Frame(frame)
        actions.pack(fill="x", pady=(0, 8))
        self.validate_button = ttk.Button(actions, text="1. 실기 경로 검증", command=self._validate)
        self.validate_button.pack(side="left")
        self.start_button = ttk.Button(actions, text="2. 실제 로봇 실행·저장", command=self._start,
                                       state="disabled")
        self.start_button.pack(side="left", padx=(8, 0))
        self.stop_button = ttk.Button(actions, text="실기 중지", command=self._stop, state="disabled")
        self.stop_button.pack(side="left", padx=(8, 0))
        self.progress_var = tk.DoubleVar(value=0)
        ttk.Progressbar(actions, variable=self.progress_var, maximum=100).pack(
            side="left", fill="x", expand=True, padx=(15, 8))
        self.progress_label = ttk.Label(actions, text="0 / 0")
        self.progress_label.pack(side="left")

        self.status_var = tk.StringVar(value="실제 측정값을 입력하고 1. 실기 경로 검증을 누르세요.")
        ttk.Label(frame, textvariable=self.status_var, wraplength=980).pack(anchor="w", pady=(0, 7))
        self.table = ttk.Treeview(frame, columns=("profile", "state", "y", "z"),
                                   show="headings", height=6)
        for key, title, width in (("profile", "경로", 290), ("state", "판정", 330),
                                  ("y", "모델 Y 범위 mm", 190), ("z", "최저 Z mm", 130)):
            self.table.heading(key, text=title)
            self.table.column(key, width=width, stretch=(key == "state"))
        self.table.pack(fill="x", pady=(0, 8))
        for name in ("setup_10cm", *PROFILE_NAMES):
            self.table.insert("", "end", iid=name, values=(name, "미검증", "—", "—"))
        ttk.Label(frame, text="실행 로그 / 저장 위치").pack(anchor="w")
        self.log = tk.Text(frame, height=7, state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True)
        self._append("이 창은 실제 로봇을 움직입니다. 준비 이동은 전체 배치에서 한 번만 실행합니다.\n")
        for variable in (self.source_var, self.scene_var, self.output_var,
                         self.tool_var, self.repeats_var, self.domain_var,
                         self.y_min_var, self.y_max_var, self.z_var,
                         self.mount_xyz_var, self.mount_rpy_var,
                         self.tool_confirmed, self.obstacles_confirmed,
                         *self.profile_vars.values()):
            variable.trace_add("write", lambda *_args: self._invalidate())
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(100, self._poll)

    def _append(self, value: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", value)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _browse_dir(self, variable) -> None:
        path = self.filedialog.askdirectory(initialdir=variable.get())
        if path:
            variable.set(path)
            self._invalidate()

    def _browse_scene(self) -> None:
        path = self.filedialog.askopenfilename(
            initialdir=str(Path(self.scene_var.get()).parent),
            filetypes=[("MuJoCo XML", "*.xml"), ("모든 파일", "*.*")])
        if path:
            self.scene_var.set(path)
            self._invalidate()

    def _invalidate(self) -> None:
        self.preflight_ready = False
        self.start_button.configure(state="disabled")

    @staticmethod
    def _triple(text: str) -> tuple[float, float, float]:
        values = tuple(float(item) for item in text.split())
        if len(values) != 3:
            raise ValueError("장착 변환은 공백으로 구분한 숫자 3개여야 합니다")
        return values

    def _set_busy(self, busy: bool, *, executing: bool = False) -> None:
        state = "disabled" if busy else "normal"
        for widget in (self.source_entry, self.source_button, self.scene_entry,
                       self.scene_button, self.output_entry, self.output_button,
                       self.repeat_box, self.domain_entry, self.mount_xyz_entry,
                       self.mount_rpy_entry, *self.geometry_entries,
                       *self.profile_checks, *self.operator_checks, self.validate_button):
            widget.configure(state=state)
        self.tool_combo.configure(state="disabled" if busy or self.lock_tool else "readonly")
        self.stop_button.configure(state="normal" if executing else "disabled")
        self.start_button.configure(state="disabled" if busy or not self.preflight_ready else "normal")

    def _validate(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        self._invalidate()
        try:
            arguments = {
                "source_dir": Path(self.source_var.get()),
                "arm_scene": Path(self.scene_var.get()),
                "output_dir": Path(self.output_var.get()),
                "tool": LABELS[self.tool_var.get()],
                "profiles": tuple(name for name in PROFILE_NAMES if self.profile_vars[name].get()),
                "repeats": int(self.repeats_var.get()),
                "corridor_y_min_mm": float(self.y_min_var.get()),
                "corridor_y_max_mm": float(self.y_max_var.get()),
                "lower_obstacle_top_z_mm": float(self.z_var.get()),
                "mdpg_mount_xyz_m": self._triple(self.mount_xyz_var.get()),
                "mdpg_mount_rpy_rad": self._triple(self.mount_rpy_var.get()),
                "physical_tool_confirmed": self.tool_confirmed.get(),
                "other_obstacles_clear_confirmed": self.obstacles_confirmed.get(),
            }
        except (ValueError, KeyError) as error:
            self.status_var.set(f"입력값 확인: {error}")
            return
        self._set_busy(True)
        self.status_var.set("MuJoCo 모델과 입력한 실제 공간 측정값을 검증 중…")

        def work():
            try:
                path = prepare_front_real_batch(**arguments)
                self.events.put(("prepared", str(path)))
            except Exception as error:
                self.events.put(("error", str(error)))

        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def _start(self) -> None:
        if not self.preflight_ready or self.preflight is None:
            return
        if not self.messagebox.askyesno(
                "실제 로봇 동작 확인",
                "실제 A0509가 움직입니다. 현재 시작 자세, 장착 도구, 작업영역과 정지 버튼을 현장에서 확인했습니까?\n"
                "준비 이동 1회 후 선택한 F 경로를 순서대로 실행합니다."):
            return
        try:
            domain = int(self.domain_var.get())
            if domain < 0:
                raise ValueError("Real ROS_DOMAIN_ID는 0 이상이어야 합니다")
        except ValueError as error:
            self.status_var.set(str(error))
            return
        report = json.loads(self.preflight.read_text(encoding="utf-8"))
        self.stop_event.clear()
        self.progress_var.set(0)
        self.progress_label.configure(text=f"0 / {1 + len(report['profiles']) * report['repeats_per_profile']}")
        self._set_busy(True, executing=True)
        self.status_var.set("실제 로봇 실행 중. 각 구간의 RT 데이터를 저장합니다.")
        path = self.preflight

        def work():
            try:
                result = run_prepared_real_batch(
                    path, real_domain_id=domain,
                    stop_requested=self.stop_event.is_set,
                    progress=lambda event: self.events.put(("progress", event)))
                self.events.put(("completed", str(result)))
            except Exception as error:
                self.events.put(("error", str(error)))

        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def _stop(self) -> None:
        self.stop_event.set()
        self.status_var.set("실기 중지 요청 중… 현재 실행기에 정지 신호를 보냈습니다.")

    def _poll(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "prepared":
                    self.preflight = Path(value)
                    report = json.loads(self.preflight.read_text(encoding="utf-8"))
                    self.preflight_ready = report["ready_for_supervised_trial"]
                    for name, check in report["model_checks"].items():
                        state = "통과" if check["passed"] else "차단: " + "; ".join(check["reasons"])
                        self.table.set(name, "state", state)
                        self.table.set(name, "y", str(check["model_y_mm"]))
                        self.table.set(name, "z", f"{check['minimum_model_z_mm']:.1f}")
                    self._append(f"검증 보고서: {self.preflight}\n")
                    self._append(f"필요한 시작 관절각 (deg): {report['required_start_joint_deg']}\n")
                    if report.get("operator_blocker"):
                        self._append(f"작업자 확인 필요: {report['operator_blocker']}\n")
                    self.status_var.set("실기 실행 준비됨" if self.preflight_ready else "실기 차단: 보고서의 경로별 이유를 확인하세요.")
                    self._set_busy(False)
                elif kind == "progress":
                    if value["event"] == "segment_start":
                        label = value["name"] + (f" #{value['repeat_index']}" if value["repeat_index"] else "")
                        self.status_var.set(f"실기 실행 중: {label} ({value['index']}/{value['total']})")
                        self._append(f"시작 {value['index']}/{value['total']}: {label}\n")
                    elif value["event"] == "segment_done":
                        self.progress_var.set(value["index"] / value["total"] * 100)
                        self.progress_label.configure(text=f"{value['index']} / {value['total']}")
                        self._append(f"{value['name']}: {value['result']} · {value['record_file']}\n")
                elif kind == "completed":
                    self.status_var.set(f"실기 수집 완료: {value}")
                    self._append(f"배치 결과: {value}\n")
                    self.preflight_ready = False
                    self._set_busy(False)
                elif kind == "error":
                    self.status_var.set(f"중지 또는 오류: {value}")
                    self._append(f"중지 또는 오류: {value}\n")
                    self.preflight_ready = False
                    self._set_busy(False)
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    def _close(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            self._stop()
            self.status_var.set("실행기가 정지하고 기록을 닫을 때까지 기다리는 중…")
            return
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tool", choices=("ag145", "mdpg_1s1"), default="mdpg_1s1")
    parser.add_argument("--lock-tool", action="store_true")
    parser.add_argument("--mount-xyz", default="0 0 0")
    parser.add_argument("--mount-rpy", default="0 0 0")
    args = parser.parse_args(argv)
    FrontRealGui(initial_tool=args.tool, lock_tool=args.lock_tool,
                 initial_mount_xyz=args.mount_xyz if args.tool == "mdpg_1s1" else "0 0 0",
                 initial_mount_rpy=args.mount_rpy if args.tool == "mdpg_1s1" else "0 0 0").run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
