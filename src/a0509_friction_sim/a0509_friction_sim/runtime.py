"""Bounded worker and file locking on both Linux and Windows."""
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import psutil


def sha256(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


def available_ram():
    return psutil.virtual_memory().available


@contextmanager
def run_lock(output):
    with (output / "run.lock").open("a+b") as stream:
        if stream.tell() == 0:
            stream.write(b"0"); stream.flush()
        stream.seek(0)
        if os.name == "nt":
            import msvcrt
            msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            yield
        finally:
            stream.seek(0)
            if os.name == "nt":
                msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(stream, fcntl.LOCK_UN)


def worker_memory(process):
    parent = psutil.Process(process.pid)
    total = 0
    for item in [parent, *parent.children(recursive=True)]:
        try:
            info = item.memory_info()
            total += max(info.rss, getattr(info, "private", 0))
        except psutil.NoSuchProcess:
            pass
    return total


def kill_worker(process):
    try:
        children = psutil.Process(process.pid).children(recursive=True)
    except psutil.NoSuchProcess:
        children = []
    for child in reversed(children):
        try:
            child.kill()
        except psutil.NoSuchProcess:
            pass
    if process.poll() is None:
        process.kill()
    process.wait()


def run_worker(arguments, index, log_path, timeout_s=900):
    command = [sys.executable, "-u", "-m", "a0509_friction_sim", *arguments, "--worker", str(index)]
    started, peak = time.monotonic(), 0
    with log_path.open("w", encoding="utf-8") as log:
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join(dict.fromkeys(p for p in sys.path if p))
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env)
        try:
            while process.poll() is None:
                try:
                    peak = max(peak, worker_memory(process))
                except psutil.NoSuchProcess:
                    if process.poll() is not None:
                        break
                    raise
                if peak > 2*1024**3:
                    raise RuntimeError("Worker exceeded 2 GiB memory guard")
                if available_ram() < 1024**3:
                    raise RuntimeError("Host available RAM fell below 1 GiB")
                if time.monotonic()-started > timeout_s:
                    raise RuntimeError(f"Worker exceeded {timeout_s} second timeout")
                time.sleep(.05)
            return process.wait(), round(peak/1024**2, 1)
        finally:
            if process.poll() is None:
                kill_worker(process)
