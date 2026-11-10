"""Capture the MuJoCo X11 display during a simulation trial."""

from __future__ import annotations

from pathlib import Path
import threading
import time


class SimulationVideoRecorder:
    def __init__(self, path: str | Path, fps: int = 15):
        self.path = Path(path)
        self.fps = fps
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: Exception | None = None
        self.frames = 0

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("video recorder already started")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._record, name="mujoco-video", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=10):
            raise RuntimeError("video capture did not initialize")
        if self._error is not None:
            raise RuntimeError(f"video capture failed: {self._error}") from self._error

    def _record(self) -> None:
        writer = None
        try:
            import imageio_ffmpeg
            import mss

            with mss.mss() as screen:
                monitor = screen.monitors[1]
                width, height = monitor["width"], monitor["height"]
                writer = imageio_ffmpeg.write_frames(
                    str(self.path), (width, height), fps=self.fps,
                    codec="libx264", pix_fmt_in="rgb24", pix_fmt_out="yuv420p",
                    output_params=["-crf", "24", "-movflags", "+faststart"],
                )
                writer.send(None)
                self._ready.set()
                next_frame = time.monotonic()
                while not self._stop.is_set():
                    frame = screen.grab(monitor)
                    writer.send(frame.rgb)
                    self.frames += 1
                    next_frame += 1 / self.fps
                    self._stop.wait(max(0, next_frame - time.monotonic()))
        except Exception as error:
            self._error = error
            self._ready.set()
        finally:
            if writer is not None:
                try:
                    writer.close()
                except Exception as error:
                    self._error = error

    def stop(self) -> dict:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=20)
            if self._thread.is_alive():
                raise RuntimeError("video encoder did not stop")
        if self._error is not None:
            raise RuntimeError(f"video capture failed: {self._error}") from self._error
        if self.frames == 0 or not self.path.is_file() or self.path.stat().st_size == 0:
            raise RuntimeError("video capture produced no frames")
        return {"path": str(self.path), "relative_path": self.path.name,
                "fps": self.fps, "frames": self.frames,
                "size_bytes": self.path.stat().st_size}
