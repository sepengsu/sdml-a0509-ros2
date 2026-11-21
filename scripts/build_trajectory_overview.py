"""Build a standalone overview of the recorded T1–T16 reference paths.

The page uses the completed bare_p0/trial_001 trajectory CSVs as a stable
representative of the 16-lane campaign. It never starts a simulation.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "data/grid_16_trajectories/simulation"
DESTINATION = ROOT / "docs/trajectories-t1-t16.html"

DETAILS = {
    1: ("직선 왕복", "X 방향으로 120 mm 이동한 뒤 출발점으로 돌아온다.", "TCP X +120 mm → 원점"),
    2: ("대각 왕복", "Y 80 mm와 Z 60 mm를 동시에 이동한 뒤 돌아온다.", "TCP Y +80 / Z +60 mm → 원점"),
    3: ("직육면체 모서리", "X·Y·Z 방향의 8개 선분을 따라 직육면체 모서리를 돌고 출발점으로 돌아온다.", "TCP 120 × 80 × 60 mm, 8개 선분"),
    4: ("관절별 왕복", "J1부터 J6까지 한 축씩 +5° → −5° → 시작 자세 순서로 움직인다.", "J1–J6 순차, 각 ±5°"),
    5: ("두 관절 교대", "앞 구간에는 J2, 뒤 구간에는 J3에 서로 다른 사인파를 준다.", "J2 → J3, 각 ±5°"),
    6: ("어깨·손목 결합", "J1과 J4를 서로 다른 주파수의 사인파로 동시에 움직인다.", "J1 + J4, 각 ±5°"),
    7: ("손목 여기", "J5와 J6을 동시에 흔들어 손목 쪽 동역학을 관찰한다.", "J5 ±5° + J6 ±4°"),
    8: ("홀수축 다중 사인", "J1·J3·J5 각각에 두 주파수의 사인파를 합친다.", "J1 + J3 + J5"),
    9: ("짝수축 다중 사인", "J2·J4·J6 각각에 두 주파수의 사인파를 합친다.", "J2 + J4 + J6"),
    10: ("6축 동시 사인", "여섯 관절을 서로 다른 주파수의 사인파로 동시에 움직인다.", "J1–J6 동시, 각 ±4°"),
    11: ("홀수 묶음 주파수 상승", "J2·J3·J5의 진동 주파수를 시간에 따라 높이는 chirp다.", "J2 + J3 + J5 chirp"),
    12: ("다른 축 주파수 상승", "검증용으로 J1·J4·J6의 진동 주파수를 높인다.", "J1 + J4 + J6 chirp"),
    13: ("다른 시작 자세 A", "기본 자세와 다른 출발점에서 J1·J2·J3·J5를 함께 움직인다.", "새 시작 자세, J1/J2/J3/J5"),
    14: ("다른 시작 자세 B", "또 다른 출발점에서 J2 → J4 → J1·J5·J6 순으로 구간을 나눠 움직인다.", "새 시작 자세, 구간별 혼합"),
    15: ("저속 방향 반전", "낮은 주파수와 작은 진폭으로 6축의 저속 구간과 방향 반전을 늘린다.", "J1–J6 저속, 구간별 작동"),
    16: ("기록본: 6축 결합 chirp", "다른 시작 자세에서 여섯 관절의 주파수를 함께 높인다. 이 기록본은 현재 수정 중인 Fourier 설계 이전 버전이다.", "새 시작 자세, J1–J6 chirp"),
}


def load_trajectory(number: int) -> dict:
    folder = SOURCE / f"T{number}" / "bare_p0" / "trial_001"
    with (folder / "trajectory.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    metadata = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    config = metadata["trajectory_config"]
    cartesian = number <= 3
    columns = ("x_mm", "y_mm", "z_mm") if cartesian else tuple(f"q{i}_rad" for i in range(1, 7))
    first = [float(rows[0][column]) for column in columns]
    unit_scale = 1.0 if cartesian else 180.0 / math.pi
    whole = [[float(row[column]) for column in columns] for row in rows]
    sample_step = max(1, math.ceil((len(rows) - 1) / 280))
    sampled_indices = sorted(set(range(0, len(rows), sample_step)) | {len(rows) - 1})
    points = [
        [round(float(rows[index]["time_s"]), 2)]
        + [round((value - start) * unit_scale, 3) for value, start in zip(whole[index], first)]
        for index in sampled_indices
    ]
    ranges = [round((max(row[i] for row in whole) - min(row[i] for row in whole)) * unit_scale, 2)
              for i in range(len(columns))]
    title, description, motion = DETAILS[number]
    if number == 16 and float(rows[-1]["time_s"]) <= 15:
        title = "최적화 Fourier 6축"
        description = "다른 시작 자세에서 최적화된 Fourier 계수로 여섯 관절을 함께 움직인다."
        motion = "새 시작 자세, J1–J6 Fourier"
    active = [i + 1 for i, value in enumerate(ranges) if value > 0.1] if not cartesian else []
    return {
        "id": f"T{number}", "title": title, "description": description, "motion": motion,
        "split": metadata.get("dataset_split", "diagnostic"),
        "kind": "cartesian" if cartesian else "joint", "duration": float(rows[-1]["time_s"]),
        "rows": len(rows), "scale": metadata.get("grid_time_scale", config.get("time_scale", 1.0)),
        "active": active, "ranges": ranges, "home": [round(value * 180 / math.pi, 1) for value in config["joint_start_rad"]],
        "points": points, "source": f"../data/grid_16_trajectories/simulation/T{number}/bare_p0/trial_001/trajectory.csv",
    }


HTML = r'''<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>A0509 궤적 T1–T16</title>
<style>
:root { color-scheme: light dark; --bg:#f6f8fb; --panel:#fff; --ink:#192638; --muted:#53657a; --line:#dbe3ec; --accent:#235fbd; --soft:#eaf2ff; --plot:#fff; }
@media (prefers-color-scheme: dark) { :root { --bg:#101721; --panel:#182333; --ink:#e9f0f8; --muted:#acbbcc; --line:#344458; --accent:#8ab8ff; --soft:#203757; --plot:#141f2e; } }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--ink); font:15px/1.55 system-ui,-apple-system,"Malgun Gothic",sans-serif; }
main { max-width:1180px; margin:auto; padding:32px 24px 70px; }
h1 { font-size:clamp(1.8rem,4vw,2.7rem); letter-spacing:-.035em; line-height:1.2; margin:5px 0 10px; }
h2 { font-size:1.35rem; margin:36px 0 10px; }
p { margin:8px 0; }
.eyebrow { color:var(--accent); font-weight:700; letter-spacing:.09em; font-size:.76rem; text-transform:uppercase; }
.lead { max-width:75ch; color:var(--muted); font-size:1.06rem; }
.meta { display:flex; flex-wrap:wrap; gap:9px; margin:18px 0 25px; }
.meta span { background:var(--soft); padding:5px 11px; border-radius:999px; font-size:.85rem; font-weight:650; }
.note { border-left:4px solid var(--accent); padding:11px 16px; background:var(--soft); margin:20px 0; }
.note strong { display:block; margin-bottom:3px; }
.panel { background:var(--panel); border:1px solid var(--line); border-radius:14px; padding:20px; }
.table-wrap { overflow-x:auto; }
table { width:100%; border-collapse:collapse; min-width:780px; }
th,td { padding:10px 12px; text-align:left; border-bottom:1px solid var(--line); vertical-align:top; }
th { color:var(--muted); font-size:.8rem; font-weight:700; white-space:nowrap; }
td { font-size:.91rem; }
tr:last-child td { border-bottom:0; }
tr[data-selected="true"] { background:var(--soft); }
button { font:inherit; cursor:pointer; }
.pick { background:none; border:0; color:var(--accent); font-weight:800; padding:1px 4px; text-decoration:underline; text-underline-offset:3px; }
.pick:focus-visible,.legend button:focus-visible { outline:3px solid var(--accent); outline-offset:3px; }
.split { display:inline-block; font-size:.75rem; font-weight:750; letter-spacing:.02em; text-transform:uppercase; }
.split.diagnostic { color:var(--muted); } .split.train { color:#12806b; } .split.validation { color:#a56512; } .split.test { color:#a14c91; }
@media (prefers-color-scheme: dark) { .split.train { color:#65d7b9; } .split.validation { color:#f4c570; } .split.test { color:#e4a4d9; } }
.detail-head { display:flex; flex-wrap:wrap; align-items:baseline; gap:8px 16px; }
.detail-head h2 { margin:0; } .detail-head small { color:var(--muted); }
.facts { display:flex; flex-wrap:wrap; gap:7px; margin:14px 0 16px; }
.facts span { border:1px solid var(--line); border-radius:7px; padding:3px 8px; font-size:.82rem; }
.chart { width:100%; height:auto; display:block; background:var(--plot); border-radius:8px; }
.legend { display:flex; flex-wrap:wrap; gap:8px; margin-top:12px; }
.legend button { display:flex; align-items:center; gap:6px; border:1px solid var(--line); background:var(--panel); color:var(--ink); border-radius:7px; padding:5px 9px; font-size:.82rem; }
.legend button[aria-pressed="false"] { opacity:.45; }
.swatch { display:inline-block; width:14px; height:3px; border-radius:2px; }
.detail-text { margin-top:15px; max-width:80ch; }
.source { font-size:.83rem; margin-top:12px; } a { color:var(--accent); }
.foot { color:var(--muted); font-size:.86rem; margin-top:27px; }
@media (max-width:650px) { main { padding:24px 14px 50px; } .panel { padding:13px; } }
</style>
</head>
<body>
<main>
<div class="eyebrow">A0509 · MuJoCo reference paths</div>
<h1>Trajectory T1–T16 한눈에 보기</h1>
<p class="lead">16개 궤적의 설계와 실제 저장된 목표 경로를 함께 보여줍니다. 아래 그래프는 <strong>bare_p0 / trial_001</strong>의 <code>trajectory.csv</code>에서 추린 목표값이며 실로봇 측정값이 아닙니다.</p>
<div class="meta"><span>직교좌표 T1–T3</span><span>관절공간 T4–T16</span><span>기록 간격 100 Hz</span><span>한 조건당 16대 동시 실행</span></div>
<div class="note"><strong>그래프 읽기</strong>선은 시작값에서 얼마나 변했는지를 나타냅니다. 직교좌표는 mm, 관절각은 °입니다. 표의 시간에는 앞뒤 1초 정지 구간과 캠페인의 감속 배율이 포함됩니다.</div>
<h2>전체 궤적 표</h2>
<div class="panel table-wrap"><table><thead><tr><th>궤적</th><th>분할</th><th>방식</th><th>움직임</th><th>구동 축</th><th>기록 길이</th></tr></thead><tbody id="rows"></tbody></table></div>
<h2>궤적 상세</h2>
<section class="panel" id="detail" aria-live="polite">
  <div class="detail-head"><h2 id="detail-title"></h2><small id="detail-sub"></small></div>
  <div class="facts" id="facts"></div>
  <svg id="chart" class="chart" viewBox="0 0 860 300" role="img" aria-label="선택한 궤적의 시간별 목표값"></svg>
  <div class="legend" id="legend" aria-label="그래프 축 표시"></div>
  <p class="detail-text" id="detail-text"></p>
  <p class="source"><a id="source" href="#">이 궤적의 원본 trajectory.csv 열기</a></p>
</section>
<div class="note" id="t16-note"><strong>T16 버전 주의</strong>현재 저장된 T16 기록은 42.5초짜리 6축 결합 chirp입니다. 작업 폴더의 T16 코드는 최적화 Fourier 경로로 수정 중이며, 이 HTML 그래프는 그 새 경로를 보여주지 않습니다.</div>
<p class="foot">분할: T1–T4 진단 · T5–T11/T15 학습 · T12 검증 · T13/T14/T16 시험. 반복 3회는 동일 설정의 결정론적 시뮬레이션이며 독립적인 상태공간 확대가 아닙니다. 그리퍼·페이로드는 고정 장착으로 가정했고 실제 파지나 실로봇 안전 경로를 검증하지 않았습니다.</p>
</main>
<script>
const DATA = __DATA__;
const colors = ['#e75e64','#2777cf','#20a783','#ad6bcb','#d58b18','#5361b6'];
const labels = {diagnostic:'진단',train:'학습',validation:'검증',test:'시험'};
const rows = document.getElementById('rows');
let selected = 0;
let visible = [];
function make(tag, text, parent, className) { const el=document.createElement(tag); el.textContent=text; if(className) el.className=className; parent.appendChild(el); return el; }
DATA.forEach((d,index)=>{
  const tr=document.createElement('tr'); tr.dataset.index=index; rows.appendChild(tr);
  const id=make('td','',tr); const btn=make('button',d.id,id,'pick'); btn.type='button'; btn.addEventListener('click',()=>select(index));
  const split=make('td','',tr); make('span',labels[d.split]||d.split,split,'split '+d.split);
  make('td',d.kind==='cartesian'?'TCP 직교좌표':'관절공간',tr);
  make('td',d.motion,tr);
  make('td',d.kind==='cartesian'?'X/Y/Z':d.active.map(n=>'J'+n).join(' · '),tr);
  make('td',d.duration.toFixed(1)+'초',tr);
});
function svgEl(tag, attrs, parent) { const el=document.createElementNS('http://www.w3.org/2000/svg',tag); for(const [k,v] of Object.entries(attrs)) el.setAttribute(k,String(v)); parent.appendChild(el); return el; }
function draw() {
  const d=DATA[selected], svg=document.getElementById('chart'); svg.replaceChildren();
  const W=860,H=300,left=58,right=20,top=22,bottom=42,w=W-left-right,h=H-top-bottom;
  const unit=d.kind==='cartesian'?'mm':'°';
  const activeValues=d.points.flatMap(p=>p.slice(1).filter((_,i)=>visible[i]));
  let lo=Math.min(0,...activeValues),hi=Math.max(0,...activeValues);
  if(hi-lo<1) { lo-=1; hi+=1; } else { const pad=(hi-lo)*.12;lo-=pad;hi+=pad; }
  const x=t=>left+w*t/d.duration, y=v=>top+h-(v-lo)*h/(hi-lo);
  for(let i=0;i<=4;i++) { const v=lo+(hi-lo)*i/4, yy=y(v);
    svgEl('line',{x1:left,y1:yy,x2:W-right,y2:yy,stroke:'var(--line)', 'stroke-width':1},svg);
    const text=svgEl('text',{x:left-10,y:yy+4,'text-anchor':'end',fill:'var(--muted)','font-size':12},svg);text.textContent=v.toFixed(hi-lo>50?0:1);
  }
  for(let i=0;i<=4;i++) { const t=d.duration*i/4,xx=x(t);
    const text=svgEl('text',{x:xx,y:H-17,'text-anchor':'middle',fill:'var(--muted)','font-size':12},svg);text.textContent=t.toFixed(t%1?1:0);
  }
  const axis=svgEl('text',{x:16,y:15,fill:'var(--muted)','font-size':12},svg);axis.textContent='Δ '+unit;
  const time=svgEl('text',{x:W-right,y:H-17,'text-anchor':'end',fill:'var(--muted)','font-size':12},svg);time.textContent='시간 (s)';
  for(let j=0;j<d.points[0].length-1;j++) { if(!visible[j]) continue;
    const path=d.points.map((p,i)=>(i?'L':'M')+x(p[0]).toFixed(1)+' '+y(p[j+1]).toFixed(1)).join(' ');
    svgEl('path',{d:path,fill:'none',stroke:colors[j], 'stroke-width':2.2,'stroke-linejoin':'round','stroke-linecap':'round'},svg);
  }
}
function select(index) {
  selected=index; const d=DATA[index]; visible=d.ranges.map(v=>v>.1);
  document.querySelectorAll('#rows tr').forEach((tr,i)=>tr.dataset.selected=String(i===index));
  document.getElementById('detail-title').textContent=d.id+' · '+d.title;
  document.getElementById('detail-sub').textContent=labels[d.split]+' · '+(d.kind==='cartesian'?'TCP 직교좌표':'관절공간');
  const facts=document.getElementById('facts');facts.replaceChildren();
  [d.duration.toFixed(1)+'초',d.rows.toLocaleString()+'개 100 Hz 목표점','시간 배율 ×'+d.scale, d.kind==='cartesian'?'상대 TCP 경로':'시작 관절각 '+d.home.map(v=>v.toFixed(1)+'°').join(' / ')].forEach(s=>make('span',s,facts));
  document.getElementById('detail-text').textContent=d.description;
  document.getElementById('source').href=d.source;
  const legend=document.getElementById('legend');legend.replaceChildren();
  d.ranges.forEach((range,j)=>{if(range<=.1)return;
    const button=make('button','',legend);button.type='button';button.setAttribute('aria-pressed','true');
    const swatch=make('span','',button,'swatch');swatch.style.background=colors[j];
    make('span',(d.kind==='cartesian'?['X','Y','Z'][j]:'J'+(j+1))+' · 범위 '+range.toFixed(1)+(d.kind==='cartesian'?' mm':'°'),button);
    button.addEventListener('click',()=>{visible[j]=!visible[j];button.setAttribute('aria-pressed',String(visible[j]));draw();});
  });
  draw();
}
if(DATA[15].duration<=15) document.getElementById('t16-note').hidden=true;
select(0);
</script>
</body>
</html>
'''


def main() -> None:
    records = [load_trajectory(number) for number in range(1, 17)]
    json_data = json.dumps(records, ensure_ascii=False, separators=(",", ":"))
    DESTINATION.write_text(HTML.replace("__DATA__", json_data), encoding="utf-8")
    print(f"Wrote {DESTINATION} ({DESTINATION.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
