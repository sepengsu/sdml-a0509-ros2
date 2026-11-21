"""Render a Korean, captioned guide to the A0509 front data workflow.

The video contains an actual one-cycle MuJoCo log and a rendered model preview.
It is an instructional animation, not a recording of a physical robot run.
"""

from __future__ import annotations

import gzip
import json
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output/a0509_front_data_collection_guide.mp4"
PREVIEW = ROOT / "output/a0509_front_f1_f4_ag145_preview.png"
SUMMARY = ROOT / "results/front_sim_data_demo/a0509_front_f1_slow_forward_ag145_1cycles.summary.json"
WIDTH, HEIGHT, FPS = 1280, 720, 15
SCENES = (5, 7, 10, 9, 10, 7)
BG = "#0b1421"
PANEL = "#142438"
PANEL2 = "#1c314a"
WHITE = "#f5f7fa"
MUTED = "#b8cad9"
CYAN = "#50d6f5"
ORANGE = "#ffbf66"
GREEN = "#83d19b"
RED = "#ff8c8c"
FONT = Path("C:/Windows/Fonts/malgun.ttf")
BOLD = Path("C:/Windows/Fonts/malgunbd.ttf")


def font(size: int, bold: bool = False):
    return ImageFont.truetype(str(BOLD if bold else FONT), size)


def text(draw, xy, value, size=25, color=WHITE, bold=False):
    draw.text(xy, value, font=font(size, bold), fill=color)


def panel(draw, rect, fill=PANEL):
    draw.rounded_rectangle(rect, radius=22, fill=fill, outline="#2a4660", width=2)


def scene_base(chapter: str, title: str, subtitle: str, progress: float):
    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, WIDTH, 7), fill=CYAN)
    text(draw, (55, 30), "A0509  |  Sim-to-Real 데이터 수집", 20, CYAN, True)
    text(draw, (55, 74), title, 38, WHITE, True)
    text(draw, (57, 133), subtitle, 21, MUTED)
    draw.rounded_rectangle((1050, 37, 1222, 76), radius=17, fill="#463b29")
    text(draw, (1069, 44), chapter, 18, ORANGE, True)
    draw.rectangle((55, 667, 1225, 670), fill="#2b4559")
    draw.rectangle((55, 667, 55 + int(1170 * progress), 670), fill=CYAN)
    text(draw, (57, 678), "설명용 영상 · 실로봇 촬영 아님", 15, MUTED)
    text(draw, (1036, 678), "simulation only", 15, ORANGE, True)
    return image, draw


def load_example():
    summary = json.loads(SUMMARY.read_text(encoding="utf-8"))
    with gzip.open(summary["samples_file"], "rt", encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream]
    return summary, rows


def draw_plot(draw, rows):
    x0, y0, x1, y1 = 109, 323, 1151, 561
    draw.rectangle((x0, y0, x1, y1), fill="#0d1a2a", outline="#38506a", width=2)
    for k in range(5):
        y = y0 + (y1-y0) * k/4
        draw.line((x0, y, x1, y), fill="#263b50", width=1)
    sampled = rows[::8]
    lo = min(min(r["target_q_rad"][1], r["sim_q_rad"][1]) for r in sampled) - .04
    hi = max(max(r["target_q_rad"][1], r["sim_q_rad"][1]) for r in sampled) + .04
    def coords(key):
        return [(x0 + int((x1-x0)*r["time_s"]/rows[-1]["time_s"]),
                 y1 - int((y1-y0)*(r[key][1]-lo)/(hi-lo))) for r in sampled]
    draw.line(coords("target_q_rad"), fill=CYAN, width=5, joint="curve")
    draw.line(coords("sim_q_rad"), fill=ORANGE, width=2, joint="curve")
    text(draw, (110, 281), "실제 저장된 1 Cycle 예시: J2 목표 vs MuJoCo 관측", 20, WHITE, True)
    draw.line((742, 298, 785, 298), fill=CYAN, width=4)
    text(draw, (795, 286), "목표", 17, CYAN)
    draw.line((892, 298, 935, 298), fill=ORANGE, width=4)
    text(draw, (945, 286), "시뮬레이션 관측", 17, ORANGE)
    text(draw, (103, 567), "0 s", 16, MUTED)
    text(draw, (1090, 567), "36 s", 16, MUTED)


def render_scene(number: int, progress: float, summary: dict, rows: list[dict]):
    if number == 0:
        image, draw = scene_base("INTRO", "데이터는 어디에서 나오나?",
                                 "목표 JSON → MuJoCo 관측값 → 압축 로그 → Sim/Real 비교", progress)
        preview = Image.open(PREVIEW).convert("RGB")
        preview.thumbnail((1050, 424))
        image.paste(preview, ((WIDTH-preview.width)//2, 196))
        text(draw, (72, 609), "지금은 모델 재생과 시뮬레이션 수집만 가능합니다.", 22, ORANGE, True)
        return image
    if number == 1:
        image, draw = scene_base("01 / 입력", "한 번 준비하고 Cycle을 반복",
                                 "같은 관절 목표를 도구별로 수집합니다.", progress)
        panel(draw, (61, 197, 456, 575))
        text(draw, (88, 226), "준비 이동 · 1회", 27, CYAN, True)
        text(draw, (88, 283), "+X 100 mm → +Z 60 mm", 25)
        text(draw, (88, 329), "a0509_front_setup_10cm.json", 19, MUTED)
        panel(draw, (483, 197, 1218, 575))
        text(draw, (514, 226), "반복 Cycle · 기본 10회", 27, GREEN, True)
        for y, label in zip((286, 344, 402, 460),
                            ("F1  느린 전방 왕복", "F2  중속 전방 왕복",
                             "F3  전방·상방 자세 + 정지", "F4  손목 J4~J6 + 정지")):
            text(draw, (517, y), label, 24)
        text(draw, (79, 605), "AG145와 mDPG는 각각 별도 조건으로 저장", 21, ORANGE, True)
        return image
    if number == 2:
        image, draw = scene_base("02 / SIM", "MuJoCo 데이터 수집 명령",
                                 "GUI 재생만으로는 측정 파일이 생기지 않습니다.", progress)
        panel(draw, (61, 200, 1218, 553))
        text(draw, (88, 225), "1  모델·그리퍼 경계 재검사", 24, CYAN, True)
        text(draw, (93, 267), "ros2 run sim_real_gap_getting audit_front_grippers " + chr(92), 20)
        text(draw, (93, 298), "--trajectory-dir /path/to/examples/front", 19, MUTED)
        text(draw, (88, 346), "2  1 Cycle 시범 수집", 24, GREEN, True)
        text(draw, (93, 388), "ros2 run sim_real_gap_getting collect_front_sim " + chr(92), 20)
        text(draw, (93, 420), "--trajectory-dir /path/to/examples/front " + chr(92), 19, MUTED)
        text(draw, (93, 453), "--tool ag145 --profile f1_slow_forward --repeats 1", 20, ORANGE)
        text(draw, (83, 585), "확인 후 --repeats 10  ·  mDPG는 --tool mdpg_1s1", 21, WHITE, True)
        return image
    if number == 3:
        image, draw = scene_base("03 / 저장", "압축 로그에 목표와 관측을 함께 기록",
                                 "시범 실행: AG145 · F1 · 준비 16초 + Cycle 20초", progress)
        panel(draw, (65, 190, 1216, 265))
        text(draw, (91, 204), "*.jsonl.gz  3,601행  ·  약 0.87 MB", 23, GREEN, True)
        text(draw, (689, 204), "*.summary.json  완료·충돌·오차", 21, CYAN)
        draw_plot(draw, rows)
        text(draw, (91, 613), "각 행: 시간, 반복 번호, 목표 q/dq/ddq, 관측 q/dq, 시뮬레이션 명령 토크", 19, MUTED)
        return image
    if number == 4:
        image, draw = scene_base("04 / REAL", "실로봇 수집은 검증 완료 후",
                                 "현재 전방 경로는 simulation_only로 실기 자동 전송이 차단됩니다.", progress)
        panel(draw, (61, 202, 1219, 580))
        checks = (("01", "작업대 Z·좌우 경계, 실제 그리퍼/어댑터 측정"),
                  ("02", "현재 관절 자세와 JSON 시작점 일치 확인"),
                  ("03", "Check real RT topics: 시각·관절값·토크 신호 점검"),
                  ("04", "100 Hz 명령/피드백 및 보호정지 시험"))
        for row, (index, label) in enumerate(checks):
            y = 230 + 73*row
            draw.ellipse((91, y+2, 133, y+44), fill="#3c5367")
            text(draw, (100, y+7), index, 17, WHITE, True)
            text(draw, (155, y+5), label, 22)
        text(draw, (87, 600), "검증 뒤 실기 실행기는 *_samples.jsonl + 실행 요약을 저장", 19, ORANGE, True)
        return image
    image, draw = scene_base("05 / 비교", "Gap 계산: 두 지표를 분리",
                             "같은 경로·그리퍼·반복 번호를 맞춘 뒤 비교합니다.", progress)
    panel(draw, (61, 218, 624, 550))
    panel(draw, (650, 218, 1218, 550))
    text(draw, (88, 248), "관절 추종 Gap", 28, CYAN, True)
    text(draw, (88, 321), "목표 q  ↔  실제/시뮬레이션 q", 22)
    text(draw, (88, 370), "RMSE · 최대오차 · 지연", 22, MUTED)
    text(draw, (678, 248), "토크 모델 Residual", 28, ORANGE, True)
    text(draw, (678, 321), "관측 q/dq/ddq → MuJoCo 역동역학", 20)
    text(draw, (678, 370), "검증된 실측 토크와 비교", 22, MUTED)
    text(draw, (86, 586), "토크 신호 검증 전에는 Ground Truth로 표시하지 않습니다.", 21, RED, True)
    return image


def render(output: Path = OUTPUT) -> Path:
    summary, rows = load_example()
    if summary["result"] != "completed" or len(rows) != 3601:
        raise ValueError("the one-cycle MuJoCo example is incomplete")
    output.parent.mkdir(parents=True, exist_ok=True)
    command = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
               "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS),
               "-i", "-", "-an", "-c:v", "libx264", "-crf", "20",
               "-preset", "medium", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
               str(output)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    total_frames = sum(SCENES) * FPS
    frame_number = 0
    try:
        for scene_number, seconds in enumerate(SCENES):
            for local_frame in range(seconds * FPS):
                progress = frame_number / max(1, total_frames-1)
                image = render_scene(scene_number, progress, summary, rows)
                fade = min(local_frame / 6, (seconds*FPS-1-local_frame) / 6, 1)
                if fade < 1:
                    image = Image.blend(Image.new("RGB", (WIDTH, HEIGHT), BG), image, max(0, fade))
                if frame_number == 390:
                    image.save(output.with_suffix(".png"))
                assert process.stdin is not None
                process.stdin.write(image.tobytes())
                frame_number += 1
    finally:
        if process.stdin is not None:
            process.stdin.close()
    error = process.stderr.read().decode(errors="replace") if process.stderr else ""
    if process.wait() != 0:
        raise RuntimeError(f"ffmpeg failed: {error}")
    manifest = {
        "video": str(output.resolve()),
        "thumbnail": str(output.with_suffix(".png").resolve()),
        "duration_s": sum(SCENES), "fps": FPS, "resolution": [WIDTH, HEIGHT],
        "kind": "instructional animation; no physical robot footage",
        "example_summary": str(SUMMARY.resolve()),
    }
    output.with_suffix(".json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return output


if __name__ == "__main__":
    print(render())
