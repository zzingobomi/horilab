"""허브 판의 테가 베어링 안쪽 링을 누르는 힘과 휨 응력 (0029의 "안쪽 링을 누르는 힘").

혼 볼트를 조이면 허브 판의 테가 틈(CLAMP_GAP)만큼 휘면서 안쪽 링을 누른다. 테를 고리 모양 판으로 보고 푼다:
안쪽 끝(혼 위 돋움의 가장자리)은 단단한 가운데에 붙어 기울지 않고, 바깥(안쪽 링이 누르는 자리)은 받쳐져 기울 수 있다.
축대칭이고 분포 하중이 없는 판의 처짐은 w(r) = A + B r² + C ln r + D r² ln r이다.

단위는 mm, N, MPa다.

    uv run python -m horilab_design.studies.clamp_plate
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from horilab_design.parts import joint as jp
from horilab_design.parts import shoulder as sp

E_PLA = (2000.0, 2750.0)  # PLA 탄성 계수의 범위 [MPa]
NU = 0.35
BEND_STRENGTH = 76.0  # PLA Basic의 굽힘 강도 [MPa] (뱀부랩 자료)


@dataclass(frozen=True)
class Plate:
    name: str
    outer: float  # 안쪽 링이 누르는 반지름 (링 구멍과 단의 가운데)
    inner: float  # 혼 위 돋움의 가장자리
    thick: float
    gap: float  # 조이기 전의 틈 = 테가 휘는 양


def plates() -> tuple[Plate, Plate]:
    return (
        Plate(
            "STS 관절 (6806)",
            (jp.BRG_D / 2 + jp.R_STEP) / 2,
            jp.HUB_SEAT_R,
            jp.FLANGE_T,
            jp.CLAMP_GAP,
        ),
        Plate("어깨 (6808)", (sp.BRG_D / 2 + sp.R_STEP) / 2, sp.SEAT_R1, sp.FLANGE_T, jp.CLAMP_GAP),
    )


def _rows(
    r: float, nu: float
) -> tuple[list[float], list[float], list[float], list[float], list[float]]:
    """반지름 r에서 처짐, 기울기, M_r / (-D), M_t / (-D), Q_r / (-D)의 계수 (A, B, C, D)."""
    ln = math.log(r)
    w = [1.0, r * r, ln, r * r * ln]
    slope = [0.0, 2 * r, 1 / r, 2 * r * ln + r]
    w2 = [0.0, 2.0, -1 / r**2, 2 * ln + 3]  # w''
    s_r = [0.0, 2.0, 1 / r**2, 2 * ln + 1]  # w' / r
    m_r = [w2[i] + nu * s_r[i] for i in range(4)]
    m_t = [nu * w2[i] + s_r[i] for i in range(4)]
    q = [0.0, 0.0, 0.0, 4 / r]
    return w, slope, m_r, m_t, q


def press(p: Plate, e: float, nu: float = NU) -> tuple[float, float]:
    """틈이 닫힐 때 누르는 힘 [N]과 가장 큰 휨 응력 [MPa]."""
    d = e * p.thick**3 / (12 * (1 - nu**2))
    wa, _, mra, _, _ = _rows(p.outer, nu)
    wb, sb, _, _, qb = _rows(p.inner, nu)
    # 바깥: 처짐 0, 모멘트 0. 안쪽: 기울기 0, 전단력의 합이 단위 힘 1 N
    lhs = np.array([wa, mra, sb, qb])
    rhs = np.array([0.0, 0.0, 0.0, 1.0 / (2 * math.pi * p.inner * d)])
    c = np.linalg.solve(lhs, rhs)
    force = p.gap / abs(float(np.dot(wb, c)))
    stress = 0.0
    for r in np.linspace(p.inner, p.outer, 60):
        _, _, m_r, m_t, _ = _rows(float(r), nu)
        for m in (m_r, m_t):
            stress = max(stress, abs(float(np.dot(m, c))) * d * force * 6 / p.thick**2)
    return force, stress


def report() -> None:
    print(
        f"허브 판의 테 (PLA 탄성 계수 {E_PLA[0]:.0f} ~ {E_PLA[1]:.0f} MPa, 굽힘 강도 약 {BEND_STRENGTH:.0f} MPa)"
    )
    for p in plates():
        lo, hi = press(p, E_PLA[0]), press(p, E_PLA[1])
        print(
            f"- {p.name}: 반지름 {p.inner:.1f} ~ {p.outer:.2f}, 두께 {p.thick:.1f}, 틈 {p.gap:.1f} → "
            f"누르는 힘 {lo[0]:.0f} ~ {hi[0]:.0f} N, 휨 응력 {lo[1]:.0f} ~ {hi[1]:.0f} MPa"
        )


if __name__ == "__main__":
    report()
