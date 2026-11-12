"""출력 검사에 쓰는 계산: 출력 무게 어림과, 바닥에 닿지 않고 아래를 보는 면 찾기 (docs/design.md의 "출력 전 확인").

형상은 부품 좌표 그대로 주고, 뽑을 때 위를 보는 방향(up)을 같이 준다.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

import numpy as np
from build123d import Compound

Vec = tuple[float, float, float]

# 출력 무게 어림식 (0.20mm Strength: 벽 6겹, 윗면 5층, 아랫면 3층, 채움 25%).
# 껍데기 부피 = Σ 넓이 × 두께 (세운 벽 2.52, 윗면 1.0, 아랫면 0.6, 비스듬한 면은 둘 중 큰 쪽), 부피를 넘으면 부피.
# 무게 = SOLID × (껍데기 + INFILL × 속). SOLID와 INFILL은 뽑아서 잰 Strength 부품 다섯(data/measured.toml의
# printed_mass_g: 서보 집, 원판 둘, 시험 링크, 받침 기둥)에 2% 안으로 맞춘 값이다 (tests/test_printcheck.py).
WALL_T, TOP_T, BOTTOM_T = 2.52, 1.0, 0.6
SOLID = 1.008e-3  # [g/mm³]
INFILL = 0.32
# 0.20mm Standard로 뽑는 얇은 덮개: 잰 것이 고리 하나뿐이라 그 값(무게 / 부피)을 쓴다
STANDARD_THIN = 1.125e-3  # [g/mm³]


def _mesh(
    shape: Compound, tol: float = 0.05, ang: float = 0.3
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """삼각형마다 꼭짓점(3 × 3), 넓이, 바깥 단위 법선."""
    verts, tris = shape.tessellate(tol, ang)
    pts = np.array([(v.X, v.Y, v.Z) for v in verts])
    idx = np.array(tris)
    tri = pts[idx]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(n, axis=1)
    return tri, length / 2, n / np.maximum(length, 1e-12)[:, None]


def printed_mass_g(shape: Compound, up: Vec, preset: str = "strength") -> float:
    """출력 무게 어림 [g]. preset은 "strength"(구조 부품)나 "standard"(두께 3 mm 이하 덮개)다."""
    volume = shape.volume
    if preset == "standard":
        return STANDARD_THIN * volume
    _, area, normal = _mesh(shape)
    nz = normal @ np.asarray(up, float)
    flat = np.where(nz > 0, TOP_T, BOTTOM_T) * np.abs(nz)
    wall = WALL_T * np.sqrt(np.clip(1 - nz**2, 0, 1))
    shell = min(float((area * np.maximum(flat, wall)).sum()), volume)
    return SOLID * (shell + INFILL * (volume - shell))


@dataclass(frozen=True)
class Overhangs:
    """바닥에 닿지 않으면서 수평에서 45°보다 누워 아래를 보는 면 [mm²]."""

    total: float
    flat: float  # 그중 거의 수평인 면 (다리로 뽑힌다)
    largest: float  # 한 자리(같은 높이, 6 mm 칸)에 모인 가장 넓은 면
    spots: tuple[
        tuple[float, float, float, float, bool], ...
    ]  # (넓이, 높이, 자리 u, 자리 v, 수평인지), 넓은 것부터


def overhangs(shape: Compound, up: Vec, grid: float = 6.0) -> Overhangs:
    """뽑는 방향으로 놓았을 때 아래를 보는 면. 바닥은 형상에서 가장 낮은 곳으로 본다."""
    tri, area, normal = _mesh(shape, ang=0.25)
    down = -np.asarray(up, float)
    cos_down = normal @ down  # 1이면 똑바로 아래를 본다
    center = tri.mean(axis=1)
    bed = float((tri.reshape(-1, 3) @ down).max())
    height = bed - center @ down
    over = (cos_down > 0.7072) & (height > 0.05)
    flat = over & (cos_down > 0.94)
    axes = [i for i in range(3) if abs(down[i]) < 0.5]
    groups: dict[tuple[int, int, int, bool], float] = defaultdict(float)
    for k in np.nonzero(over)[0]:
        u, v = center[k, axes[0]], center[k, axes[1]]
        groups[
            (round(float(height[k])), round(float(u) / grid), round(float(v) / grid), bool(flat[k]))
        ] += float(area[k])
    spots = sorted(
        ((a, float(h), u * grid, v * grid, fl) for (h, u, v, fl), a in groups.items()), reverse=True
    )
    return Overhangs(
        total=float(area[over].sum()),
        flat=float(area[flat].sum()),
        largest=spots[0][0] if spots else 0.0,
        spots=tuple(spots[:20]),
    )
