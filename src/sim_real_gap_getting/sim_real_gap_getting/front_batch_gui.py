"""Standalone Tk control panel for compressed, simulation-only F1–F4 batches."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading

from .front_gripper_audit import _default_trajectory_dir
from .front_sim_batch import collect_front_batch
from .front_trajectory import PROFILE_NAMES
from .lr_trajectory import _scene_path


TOOL_LABELS = {"AG145": "ag145", "mDPG": "mdpg_1s1", "두 모델 모두": "both"}


class FrontBatchGui:
    def __init__(self, *, initial_tool: str = "ag145"):
        import tkinter as tk
        from tkinter import filedialog, ttk

        self.tk = tk
        self.filedialog = filedialog
        self.ttk = ttk
        self.root = tk.Tk()
        self.root.title("A0509 F1–F4 데이터 일괄 수집 · 시뮬레이션 전용")
        self.root.geometry("940x715")
        self.events: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.run_dir: Path | None = None
        self.completed = 0
        self.total = 4
        self.trial_keys: list[str] = []

        frame = ttk.Frame(self.root, padding=18)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="F1–F4 데이터 한 번에 수집",
                  font=("Malgun Gothic", 19, "bold")).pack(anchor="w")
        ttk.Label(frame,
                  text="MuJoCo 전용 · 경로별 준비 이동 1회 · 100 Hz 압축 저장 · 실로봇 명령 없음",
                  foreground="#a65b00").pack(anchor="w", pady=(1, 14))

        input_row = ttk.Frame(frame)
        input_row.pack(fill="x", pady=(0, 8))
        ttk.Label(input_row, text="궤적 폴더", width=18).pack(side="left")
        self.input_var = tk.StringVar(value=str(_default_trajectory_dir()))
        self.input_entry = ttk.Entry(input_row, textvariable=self.input_var)
        self.input_entry.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.input_button = ttk.Button(input_row, text="선택", command=self._choose_input)
        self.input_button.pack(side="left")

        scene_row = ttk.Frame(frame)
        scene_row.pack(fill="x", pady=(0, 8))
        ttk.Label(scene_row, text="MuJoCo 장면", width=18).pack(side="left")
        self.scene_var = tk.StringVar(value=str(_scene_path()))
        self.scene_entry = ttk.Entry(scene_row, textvariable=self.scene_var)
        self.scene_entry.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.scene_button = ttk.Button(scene_row, text="선택", command=self._choose_scene)
        self.scene_button.pack(side="left")

        output_row = ttk.Frame(frame)
        output_row.pack(fill="x", pady=(0, 12))
        ttk.Label(output_row, text="저장 폴더", width=18).pack(side="left")
        self.output_var = tk.StringVar(value=str(Path.cwd() / "results/front_sim_batch"))
        self.output_entry = ttk.Entry(output_row, textvariable=self.output_var)
        self.output_entry.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.output_button = ttk.Button(output_row, text="선택", command=self._choose_output)
        self.output_button.pack(side="left")

        options = ttk.Frame(frame)
        options.pack(fill="x", pady=(0, 12))
        ttk.Label(options, text="그리퍼 모델", width=18).pack(side="left")
        initial_label = next((label for label, code in TOOL_LABELS.items()
                              if code == initial_tool), "AG145")
        self.tool_var = tk.StringVar(value=initial_label)
        self.tool_combo = ttk.Combobox(options, textvariable=self.tool_var,
                                       values=list(TOOL_LABELS), state="readonly", width=24)
        self.tool_combo.pack(side="left", padx=(0, 22))
        self.tool_combo.bind("<<ComboboxSelected>>", lambda _event: self._prepare_table())
        ttk.Label(options, text="경로별 반복").pack(side="left", padx=(0, 8))
        self.repeats_var = tk.StringVar(value="10")
        self.repeat_box = ttk.Spinbox(options, from_=1, to=100, textvariable=self.repeats_var, width=8)
        self.repeat_box.pack(side="left")
        ttk.Label(options, text="먼저 1회 시험 권장", foreground="#61788a").pack(
            side="left", padx=(22, 0))

        actions = ttk.Frame(frame)
        actions.pack(fill="x", pady=(0, 12))
        self.start_button = ttk.Button(actions, text="F1~F4 모두 수집", command=self._start)
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(actions, text="수집 중지", command=self._stop,
                                      state="disabled")
        self.stop_button.pack(side="left", padx=(10, 0))
        self.open_button = ttk.Button(actions, text="저장 폴더 열기",
                                      command=self._open_run_dir, state="disabled")
        self.open_button.pack(side="left", padx=(10, 0))
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_bar = ttk.Progressbar(actions, variable=self.progress_var,
                                            maximum=100, length=340)
        self.progress_bar.pack(side="left", padx=(23, 10), fill="x", expand=True)
        self.progress_label = ttk.Label(actions, text="0 / 4")
        self.progress_label.pack(side="left")

        self.status_var = tk.StringVar(value="준비됨: 그리퍼와 저장 폴더를 선택한 뒤 F1~F4를 수집하세요.")
        ttk.Label(frame, textvariable=self.status_var, wraplength=895).pack(anchor="w", pady=(0, 10))

        columns = ("tool", "profile", "state", "rows")
        self.table = ttk.Treeview(frame, columns=columns, show="headings", height=8)
        for name, title, width in (("tool", "그리퍼", 95), ("profile", "궤적", 285),
                                   ("state", "상태", 200), ("rows", "저장 행수", 120)):
            self.table.heading(name, text=title)
            self.table.column(name, width=width, stretch=(name == "profile"))
        self.table.pack(fill="x", pady=(0, 10))
        self._prepare_table()

        ttk.Label(frame, text="진행 로그와 저장 파일", font=("Malgun Gothic", 10, "bold")).pack(anchor="w")
        self.log = tk.Text(frame, height=8, wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True, pady=(4, 0))
        self._append("경로마다 압축 데이터(.jsonl.gz)와 실행 요약(.summary.json)을 저장합니다.\n")
        self._append("배치 폴더에는 입력 JSON, 모델 검증, 품질 보고서, manifest.json도 함께 저장합니다.\n")
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(100, self._poll)

    def _prepare_table(self) -> None:
        for item in self.table.get_children():
            self.table.delete(item)
        selected = TOOL_LABELS[self.tool_var.get()]
        tools = ("ag145", "mdpg_1s1") if selected == "both" else (selected,)
        self.trial_keys = []
        for tool in tools:
            for profile in PROFILE_NAMES:
                key = f"{tool}:{profile}"
                self.trial_keys.append(key)
                self.table.insert("", "end", iid=key,
                                  values=(tool, profile, "대기", "—"))
        self.total = len(self.trial_keys)
        self.completed = 0
        self.progress_var.set(0)
        self.progress_label.configure(text=f"0 / {self.total}")

    def _append(self, value: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", value)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _choose_input(self) -> None:
        chosen = self.filedialog.askdirectory(initialdir=self.input_var.get())
        if chosen:
            self.input_var.set(chosen)

    def _choose_output(self) -> None:
        chosen = self.filedialog.askdirectory(initialdir=self.output_var.get())
        if chosen:
            self.output_var.set(chosen)

    def _choose_scene(self) -> None:
        chosen = self.filedialog.askopenfilename(
            initialdir=str(Path(self.scene_var.get()).parent),
            filetypes=[("MuJoCo XML", "*.xml"), ("모든 파일", "*.*")])
        if chosen:
            self.scene_var.set(chosen)

    def _open_run_dir(self) -> None:
        if self.run_dir is None or not self.run_dir.is_dir():
            return
        try:
            if sys.platform == "win32":
                os.startfile(str(self.run_dir))
            else:
                subprocess.Popen(["xdg-open", str(self.run_dir)])
        except OSError as error:
            self.status_var.set(f"저장 폴더를 열지 못했습니다: {error}")

    def _set_controls(self, running: bool) -> None:
        state = "disabled" if running else "normal"
        for widget in (self.input_entry, self.input_button, self.scene_entry,
                       self.scene_button, self.output_entry,
                       self.output_button, self.repeat_box, self.start_button):
            widget.configure(state=state)
        self.tool_combo.configure(state="disabled" if running else "readonly")
        self.stop_button.configure(state="normal" if running else "disabled")

    def _start(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        try:
            repeats = int(self.repeats_var.get())
            if not 1 <= repeats <= 100:
                raise ValueError("반복 횟수는 1~100이어야 합니다")
            source = Path(self.input_var.get()).expanduser().resolve()
            scene = Path(self.scene_var.get()).expanduser().resolve()
            output = Path(self.output_var.get()).expanduser().resolve()
            if not source.is_dir():
                raise ValueError("궤적 폴더가 없습니다")
            if not scene.is_file():
                raise ValueError("MuJoCo 장면 파일이 없습니다")
        except (ValueError, OSError) as error:
            self.status_var.set(f"시작할 수 없습니다: {error}")
            return
        self._prepare_table()
        self.stop_event.clear()
        self.run_dir = None
        self._set_controls(True)
        tool = TOOL_LABELS[self.tool_var.get()]
        self.status_var.set("그리퍼 모델 생성 및 네 경로 검증 중…")
        self._append(f"\n시뮬레이션 {self.total}개 시작: tool={tool}, repeats={repeats}.\n")

        def work():
            try:
                manifest = collect_front_batch(
                    source, output, tool=tool, repeats=repeats,
                    arm_scene=scene,
                    stop_requested=self.stop_event.is_set,
                    progress=lambda event: self.events.put(("progress", event)))
                self.events.put(("done", str(manifest)))
            except Exception as error:
                self.events.put(("error", str(error)))

        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def _stop(self) -> None:
        self.stop_event.set()
        self.status_var.set("수집을 중지하고 현재 압축 파일을 닫는 중…")

    def _poll(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "progress":
                    self._handle_progress(value)
                elif kind == "done":
                    self.status_var.set(f"F1~F4 수집 완료. 결과: {value}")
                    self._append(f"배치 완료: {value}\n")
                    self._set_controls(False)
                elif kind == "error":
                    self.status_var.set(f"배치 중지 또는 실패: {value}")
                    self._append(f"배치 중지 또는 실패: {value}\n")
                    self._set_controls(False)
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    def _handle_progress(self, event: dict) -> None:
        name = event["event"]
        if name == "audit_start":
            self.run_dir = Path(event["run_dir"])
            self.open_button.configure(state="normal")
            self._append(f"저장 폴더: {self.run_dir}\n")
            self.status_var.set("모델 사전 검사 중(실로봇 이동 없음)…")
        elif name == "audit_done":
            self._append("모델 접촉·추종 검사 완료. 실제 작업공간은 아직 검증되지 않았습니다.\n")
            for tool, screen in event["lateral_width_screen"].items():
                verdict = "폭 이내" if screen["fits_400_mm"] else "40 cm 폭 초과"
                self._append(f"{tool}: {screen['span_mm']:.2f} mm ({verdict})\n")
        elif name == "trial_start":
            key = f"{event['tool']}:{event['profile']}"
            self.table.set(key, "state", "수집 중")
            self.status_var.set(f"수집 중: {event['tool']} / {event['profile']}…")
        elif name == "trial_progress":
            done, total = event["samples_done"], event["samples_total"]
            whole = (self.completed + done / total) / self.total * 100
            self.progress_var.set(whole)
            self.table.set(f"{event['tool']}:{event['profile']}", "rows", str(done))
        elif name == "trial_done":
            key = f"{event['tool']}:{event['profile']}"
            self.completed += 1
            self.table.set(key, "state", "완료")
            self.progress_var.set(self.completed / self.total * 100)
            self.progress_label.configure(text=f"{self.completed} / {self.total}")
            self._append(f"저장 완료 {key}: {event['summary_file']}\n")
        elif name == "aborted":
            self._append(f"중지: {event['message']}\n")

    def _close(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            self._stop()
            self.status_var.set("현재 시뮬레이션 파일이 닫힐 때까지 기다리는 중…")
            return
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="A0509 F1–F4 simulation batch GUI")
    parser.add_argument("--tool", choices=("ag145", "mdpg_1s1", "both"), default="ag145")
    args = parser.parse_args(argv)
    FrontBatchGui(initial_tool=args.tool).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
