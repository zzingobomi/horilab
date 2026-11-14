"""출력 안내 페이지: plan.json에서 판마다의 파일, 색, 부품, 확인할 것을 보인다."""

from __future__ import annotations

import json
from pathlib import Path

from horilab_design.pages import print_page


def test_page_shows_files_parts_and_checks(tmp_path: Path) -> None:
    folder = tmp_path / "first_joint"
    folder.mkdir()
    (folder / "01-black-a3f9c2.png").write_bytes(b"\x89PNG fake")
    plan = {
        "name": "first_joint",
        "made": "2026-11-14T21:00:00",
        "studio": "02.04.00.70",
        "profiles": "02.04.00.01",
        "bed_type": "Textured PEI Plate",
        "plates": [
            {
                "file": "01-black-a3f9c2.3mf",
                "image": "01-black-a3f9c2.png",
                "color": "black",
                "preset": "0.20mm Strength @BBL X1C",
                "minutes": 313,
                "grams": 81.0,
                "overnight": True,
                "parts": [
                    {
                        "file": "out_disc.stl",
                        "label": "출력 원판",
                        "count": 1,
                        "height_mm": 14.4,
                        "base_mm2": 900,
                        "overhang_mm2": 12,
                        "slender": False,
                    },
                    {
                        "file": "gripper_finger.stl",
                        "label": "손가락",
                        "count": 2,
                        "height_mm": 40.0,
                        "base_mm2": 60,
                        "overhang_mm2": 0,
                        "slender": True,
                    },
                ],
            }
        ],
    }
    (folder / "plan.json").write_text(json.dumps(plan, ensure_ascii=False), encoding="utf-8")
    path = print_page.write("first_joint", tmp_path, tmp_path / "pages")
    page = path.read_text(encoding="utf-8")
    assert "<title>출력 안내 first_joint</title>" in page
    assert "01-black-a3f9c2.3mf" in page and "검정" in page and "5시간 13분" in page
    assert "출력 원판" in page and "out_disc.stl" in page and "넘어질 수 있음" in page
    assert "밤에 돌리기 좋음" in page and "부품 3개" in page
    assert "AMS를 끄고" in page and "data:image/png;base64," in page
