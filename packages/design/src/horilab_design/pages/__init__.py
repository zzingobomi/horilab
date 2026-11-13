"""설계를 3D로 보여 주는 페이지를 만든다 (조립 순서, 팔 전체 모습).

틀(HTML)과 바탕 데이터는 이 폴더에 두고, 형상과 숫자는 설계 코드에서 뽑아 넣는다. 만든 페이지는 build/pages/에 쓴다.

    uv run python -m horilab_design.pages.joint_page    # 첫 관절 (손목 몸통 둘)과 어깨
    uv run python -m horilab_design.pages.arm_page      # 팔 전체
"""

from __future__ import annotations

from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT_DIR = Path(__file__).resolve().parents[5] / "build" / "pages"
