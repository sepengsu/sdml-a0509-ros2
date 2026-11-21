"""Render one recorded T16 MuJoCo simulation frame for each condition.

All frames use the same trial number, simulation time, and camera.  The saved
condition XML and observed qpos from dynamics.csv reproduce a recorded state;
this does not run a new trajectory or command a real robot.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from model.campaign_rbd import DEFAULT_CAMPAIGN, load_scene


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "results/condition_simulation_captures_T16"
CONDITIONS = (
    ("bare_p0", "none", 0.0),
    ("ag145_p0", "ag145", 0.0),
    ("ag145_p05", "ag145", 0.5),
    ("ag145_p10", "ag145", 1.0),
    ("mdpg_p0", "mdpg", 0.0),
    ("mdpg_p05", "mdpg", 0.5),
    ("mdpg_p10", "mdpg", 1.0),
)


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for path in (Path("C:/Windows/Fonts/arial.ttf"),
                 Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")):
        if path.is_file():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def _camera(lookat: tuple[float, float, float] | np.ndarray, distance: float) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = lookat
    camera.distance = distance
    camera.azimuth = 125
    camera.elevation = -22
    return camera


def _recorded_state(path: Path, time_s: float) -> tuple[dict, int]:
    selected = None
    selected_number = 0
    best_distance = float("inf")
    with path.open(encoding="utf-8", newline="") as stream:
        for number, row in enumerate(csv.DictReader(stream), start=1):
            distance = abs(float(row["ros_time_s"]) - time_s)
            if distance < best_distance:
                best_distance = distance
                selected = row
                selected_number = number
    if selected is None or best_distance > 0.0051:
        raise ValueError(f"no T16 frame near {time_s:.3f} s in {path}")
    return selected, selected_number


def capture(campaign: Path, output: Path, condition: str, gripper: str,
            payload_kg: float, *, time_s: float = 6.0, trial: str = "trial_001") -> dict:
    source = campaign / "simulation/T16" / condition / trial
    metadata_path = source / "metadata.json"
    csv_path = source / "dynamics.csv"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if (metadata.get("condition_id") != condition or metadata.get("trajectory_id") != "T16"
            or metadata.get("trial_id") != trial or metadata.get("mode") != "simulation"
            or float(metadata.get("payload_mass_kg", -1)) != payload_kg):
        raise ValueError(f"metadata identity mismatch: {source}")
    row, number = _recorded_state(csv_path, time_s)
    if (row["trajectory_id"] != "T16" or row["trial_id"] != trial
            or row["mode"] != "simulation" or row["gripper_type"] != metadata["gripper_type"]):
        raise ValueError(f"CSV identity mismatch: {source}, row {number}")
    qpos = np.asarray(json.loads(row["qpos"]), dtype=np.float64)
    q = np.asarray([float(row[f"q{i}"]) for i in range(1, 7)], dtype=np.float64)
    if qpos.shape != (6,) or not np.allclose(qpos, q, atol=1e-8, rtol=0):
        raise ValueError(f"recorded qpos mismatch: {source}, row {number}")
    scene = campaign / "models/conditions" / f"{condition}.xml"
    model = load_scene(scene)
    if model.nq != 6:
        raise ValueError(f"expected six joints in {scene}")
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    tool_name = {"none": None, "ag145": "ag145_tcp", "mdpg": "mdpg_tcp"}[gripper]
    focus = (data.site_xpos[model.site(tool_name).id].copy() if tool_name
             else data.xpos[model.body("link_6").id].copy())
    close_distance = {"none": 0.30, "ag145": 0.43, "mdpg": 0.33}[gripper]
    model.vis.global_.offwidth = 1600
    model.vis.global_.offheight = 900
    with (mujoco.Renderer(model, height=800, width=1000) as full_renderer,
          mujoco.Renderer(model, height=800, width=600) as close_renderer):
        full_renderer.update_scene(data, camera=_camera((0.05, -0.02, 0.65), 1.8))
        full = full_renderer.render()
        close_renderer.update_scene(data, camera=_camera(focus, close_distance))
        close = close_renderer.render()

    canvas = Image.new("RGB", (1600, 900), "#17202c")
    canvas.paste(Image.fromarray(np.asarray(full)), (0, 70))
    canvas.paste(Image.fromarray(np.asarray(close)), (1000, 70))
    draw = ImageDraw.Draw(canvas)
    draw.text((22, 13), f"A0509 | {condition} | payload {payload_kg:.1f} kg | T16 {float(row['ros_time_s']):.2f} s",
              font=_font(27), fill="white")
    draw.text((22, 49), "Full robot", font=_font(14), fill="#e4eaf0")
    draw.text((1020, 49), "Tool close-up", font=_font(14), fill="#e4eaf0")
    draw.text((22, 875), f"Recorded MuJoCo state | {trial} | row {number}",
              font=_font(14), fill="#e4eaf0")
    if gripper == "mdpg":
        draw.text((1240, 875), "Unmeasured CAD mount", font=_font(14), fill="#ffc76a")
    output.mkdir(parents=True, exist_ok=True)
    image_path = output / f"{condition}.png"
    canvas.save(image_path)
    return {
        "condition": condition, "gripper": gripper, "payload_mass_kg": payload_kg,
        "image": image_path.name, "trajectory": "T16", "trial": trial,
        "sim_time_s": float(row["ros_time_s"]), "source_row": number,
        "qpos_rad": qpos.tolist(), "source_csv": str(csv_path),
        "source_csv_sha256": hashlib.sha256(csv_path.read_bytes()).hexdigest(),
        "source_metadata": str(metadata_path), "scene": str(scene),
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "rendered_from_recorded_state": True,
        "mdpg_mount_verified": False if gripper == "mdpg" else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--time", type=float, default=6.0)
    parser.add_argument("--trial", default="trial_001")
    parser.add_argument("--condition", action="append")
    args = parser.parse_args()
    selected = args.condition or [item[0] for item in CONDITIONS]
    if len(selected) != len(set(selected)) or any(item not in dict((c[0], c) for c in CONDITIONS)
                                               for item in selected):
        raise ValueError("conditions must be distinct names from the seven-condition campaign")
    specs = {item[0]: item for item in CONDITIONS}
    records = [capture(args.campaign.resolve(), args.output.resolve(), *specs[name],
                       time_s=args.time, trial=args.trial) for name in selected]
    if len(records) == len(CONDITIONS):
        (args.output / "manifest.json").write_text(json.dumps(records, ensure_ascii=False, indent=2) + "\n",
                                                   encoding="utf-8")
    for record in records:
        print(args.output.resolve() / record["image"])


if __name__ == "__main__":
    main()
