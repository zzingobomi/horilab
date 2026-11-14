"""출력 안내 페이지: 출력 준비 도구(horilab_design.slicing)가 낸 plan.json과 판 그림으로 판마다 할 일을 보인다.

    uv run python -m horilab_design.pages.print_page <묶음 이름>

build/print/<이름>/plan.json을 읽어 build/pages/print_<이름>.html을 쓴다 (판 그림은 파일 안에 넣는다).
"""

from __future__ import annotations

import argparse
import base64
import html
import json
from pathlib import Path
from typing import Any

from horilab_design.slicing import COLOR_KO, OUT_DIR
from horilab_design.values import REPO_ROOT

PAGES_DIR = REPO_ROOT / "build" / "pages"

EVERY_PRINT = [
    "파일 이름의 색과 같은 필라멘트를 끼운다.",
    "판은 텍스처 PEI다 (가진 판은 이것 하나다).",
    "출력 창에서 AMS를 끄고(외부 스풀) 출력만 누른다. 다른 설정은 건드리지 않는다.",
    "프린터 화면의 파일 이름 끝 6자리가 이 페이지와 같은지 본다.",
    "첫 층이 잘 붙는지 본다. 떨어진 부품이 있으면 프린터 화면에서 그 부품만 뺀다.",
    "끝나면 대화창에 알린다.",
]

STYLE = """
:root { --bg:#f2f3f1; --card:#ffffff; --ink:#1b211f; --muted:#5c6762; --rule:#d5dbd8; --accent:#c8641c;
  --warn:#9a5b00; --warn-bg:#fff4df; --white-chip:#ffffff; --black-chip:#1f1f1f;
  --font:"IBM Plex Sans KR","Apple SD Gothic Neo","Malgun Gothic",system-ui,sans-serif;
  --mono:"IBM Plex Mono",ui-monospace,Consolas,monospace; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --bg:#141817; --card:#1c2120; --ink:#e4e9e6;
  --muted:#9ba6a1; --rule:#313936; --accent:#f08a3e; --warn:#ffcf7a; --warn-bg:#3a2d12; color-scheme:dark; } }
:root[data-theme="dark"] { --bg:#141817; --card:#1c2120; --ink:#e4e9e6; --muted:#9ba6a1; --rule:#313936;
  --accent:#f08a3e; --warn:#ffcf7a; --warn-bg:#3a2d12; color-scheme:dark; }
* { box-sizing:border-box; }
body { background:var(--bg); color:var(--ink); font-family:var(--font); font-size:15px; line-height:1.6; margin:0;
  padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px));
  padding-block:20px 32px; }
main { max-width:980px; margin:0 auto; display:flex; flex-direction:column; gap:16px; }
h1 { font-size:22px; margin:0; text-wrap:balance; }
.meta { color:var(--muted); font-size:13px; }
.sum { display:flex; flex-wrap:wrap; gap:6px 18px; font-variant-numeric:tabular-nums; }
.box { background:var(--card); border:1px solid var(--rule); border-radius:8px; padding:16px; min-width:0; }
.box h2 { font-size:13px; letter-spacing:.05em; color:var(--muted); margin:0 0 8px; }
ol { margin:0; padding-left:20px; display:flex; flex-direction:column; gap:4px; }
.plate { display:grid; grid-template-columns:minmax(0,1fr) 260px; gap:16px; }
@media (max-width:760px) { .plate { grid-template-columns:minmax(0,1fr); } }
.plate img { width:100%; max-width:260px; border-radius:6px; border:1px solid var(--rule); background:#cfd4d1; }
.head { display:flex; flex-wrap:wrap; align-items:center; gap:6px 12px; margin-bottom:8px; }
.no { font-size:20px; font-weight:600; }
.file { font-family:var(--mono); font-size:15px; font-weight:500; overflow-wrap:anywhere; }
.chip { display:inline-flex; align-items:center; gap:6px; border:1px solid var(--rule); border-radius:999px;
  padding:1px 10px; font-size:13px; }
.dot { width:12px; height:12px; border-radius:50%; border:1px solid var(--muted); }
.night { background:var(--warn-bg); color:var(--warn); }
.facts { display:flex; flex-wrap:wrap; gap:4px 16px; color:var(--muted); font-size:13px; font-variant-numeric:tabular-nums;
  margin-bottom:8px; }
.tbl { overflow-x:auto; }
table { border-collapse:collapse; width:100%; font-size:13.5px; font-variant-numeric:tabular-nums; }
th, td { text-align:left; padding:5px 8px; border-bottom:1px solid var(--rule); white-space:nowrap; }
th { color:var(--muted); font-weight:500; }
td.name { white-space:normal; }
.mono { font-family:var(--mono); font-size:12.5px; }
.warn { color:var(--warn); }
"""


def _hm(minutes: int) -> str:
    return f"{minutes // 60}시간 {minutes % 60}분" if minutes >= 60 else f"{minutes}분"


def _plate(k: int, plate: dict[str, Any], folder: Path) -> str:
    color = str(plate["color"])
    dot = "var(--white-chip)" if color == "white" else "var(--black-chip)"
    night = '<span class="chip night">밤에 돌리기 좋음</span>' if plate["overnight"] else ""
    image = ""
    if plate.get("image"):
        data = base64.b64encode((folder / str(plate["image"])).read_bytes()).decode("ascii")
        image = f'<img src="data:image/png;base64,{data}" alt="판 {k} 배치 그림">'
    rows = []
    for part in plate["parts"]:
        warn = '<span class="warn">넘어질 수 있음 (참고)</span>' if part["slender"] else ""
        rows.append(
            f'<tr><td class="name">{html.escape(str(part["label"]))}</td><td class="mono">{html.escape(str(part["file"]))}</td>'
            f"<td>{part['count']}</td><td>{part['height_mm']}</td><td>{part['base_mm2']}</td>"
            f"<td>{part['overhang_mm2']}</td><td>{warn}</td></tr>"
        )
    count = sum(int(p["count"]) for p in plate["parts"])
    return f"""
<section class="box plate">
  <div>
    <div class="head"><span class="no">판 {k}</span><span class="file">{html.escape(str(plate["file"]))}</span>
      <span class="chip"><span class="dot" style="background:{dot}"></span>{COLOR_KO.get(color, color)}</span>{night}</div>
    <div class="facts"><span>{html.escape(str(plate["preset"]))}</span><span>{_hm(int(plate["minutes"]))}</span>
      <span>{plate["grams"]} g</span><span>부품 {count}개</span></div>
    <div class="tbl"><table>
      <thead><tr><th>부품</th><th>파일</th><th>개수</th><th>높이 mm</th><th>바닥 mm²</th><th>걸침 mm²</th><th></th></tr></thead>
      <tbody>{"".join(rows)}</tbody></table></div>
  </div>
  <div>{image}</div>
</section>"""


def render(name: str, out: Path = OUT_DIR) -> str:
    folder = out / name
    plan: dict[str, Any] = json.loads((folder / "plan.json").read_text(encoding="utf-8"))
    plates: list[dict[str, Any]] = plan["plates"]
    minutes = sum(int(p["minutes"]) for p in plates)
    grams = sum(float(p["grams"]) for p in plates)
    checks = "".join(f"<li>{html.escape(c)}</li>" for c in EVERY_PRINT)
    body = "".join(_plate(k, p, folder) for k, p in enumerate(plates, start=1))
    return f"""<title>출력 안내 {html.escape(name)}</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans+KR:wght@400;500;600&display=swap">
<style>{STYLE}</style>
<main>
  <header>
    <h1>출력 안내 · {html.escape(name)}</h1>
    <div class="meta">만든 때 {html.escape(str(plan["made"]))} · 뱀부랩 스튜디오 {html.escape(str(plan["studio"]))} ·
      프리셋 묶음 {html.escape(str(plan["profiles"]))} · 판 {html.escape(str(plan["bed_type"]))}</div>
  </header>
  <div class="sum"><span>판 {len(plates)}개</span><span>모두 {_hm(minutes)}</span><span>{grams:.1f} g</span>
    <span>순서대로 뽑는다 (같은 색을 이어 두었다)</span></div>
  <section class="box"><h2>출력할 때마다</h2><ol>{checks}</ol></section>
  {body}
  <p class="meta">높이, 바닥 넓이, 걸침은 코드의 형상으로 셈한 참고값이다. 자동 바닥 테두리와 출력 중 부품 빼기는 켜져 있지만
    실패를 막아 주지는 않으니 첫 층을 본다.</p>
</main>"""


def write(name: str, out: Path = OUT_DIR, pages: Path = PAGES_DIR) -> Path:
    pages.mkdir(parents=True, exist_ok=True)
    path = pages / f"print_{name}.html"
    path.write_text(render(name, out), encoding="utf-8")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description="출력 안내 페이지를 만든다.")
    parser.add_argument("name", help="출력 준비 묶음 이름 (build/print/<이름>/)")
    args = parser.parse_args()
    print(write(args.name).relative_to(REPO_ROOT).as_posix())


if __name__ == "__main__":
    main()
