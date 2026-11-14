//! 팔의 기구학 (docs/decisions/0033-planning-and-simulation.md의 만드는 순서 2): 정기구학과 닫힌 해 IK.
//!
//! 로봇 기술은 설계(`horilab_design.robot`)와 같다: 영점 자세에서 모든 링크 좌표계가 베이스와 같은 방향이고,
//! 관절마다 부모 좌표계에서 본 원점과 축이 있다. 정기구학은 원점으로 옮기고 축으로 돌리기를 관절마다 거듭한
//! 뒤 플랜지(J6 좌표계에서 본 회전과 위치)를 붙인다. 단위는 m와 rad다.
//!
//! IK는 이 팔의 꼴(UR형)만 푼다: J1 z, J2·J3·J4 y(나란함), J5 z, J6 y. J6의 원점은 J5 축 위여야 하고(옆으로
//! 비키면 J1의 식이 바뀐다), J3과 J4의 원점은 x가 0이어야 한다 (평면 두 링크). 해는 최대 8개다: J1 둘(어깨
//! 왼쪽·오른쪽) × J5 둘(손목 위·아래) × J3 둘(팔꿈치 위·아래). 각은 -π~π로 내고, 관절 한계는 보지 않는다.
//! 맞는지는 설계의 EAIK와 많은 자세로 견준다 (`tests/ik_eaik.rs`).

use std::f64::consts::PI;

pub type Vec3 = [f64; 3];
pub type Mat3 = [[f64; 3]; 3];

const X: Vec3 = [1.0, 0.0, 0.0];
const Y: Vec3 = [0.0, 1.0, 0.0];
const Z: Vec3 = [0.0, 0.0, 1.0];
const IDENTITY: Mat3 = [X, Y, Z];
/// 이 IK가 푸는 축 배치 (J1~J6)
const AXES: [Vec3; 6] = [Z, Y, Y, Y, Z, Y];
/// 구조를 볼 때와 특이 자세를 가를 때의 허용 차이
const EPS: f64 = 1e-9;

/// 플랜지 자세: 베이스에서 본 회전과 위치 [m].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub rotation: Mat3,
    pub position: Vec3,
}

/// 팔의 기구학 값.
#[derive(Debug, Clone, PartialEq)]
pub struct Kinematics {
    /// 관절마다 부모 좌표계에서 본 원점 [m]
    origins: [Vec3; 6],
    /// 플랜지: J6 좌표계에서 본 회전과 위치 [m]
    flange_rotation: Mat3,
    flange_position: Vec3,
}

impl Kinematics {
    /// 축 배치와 원점이 이 IK가 푸는 꼴인지 보고 만든다.
    pub fn new(
        origins: [Vec3; 6],
        axes: [Vec3; 6],
        flange_rotation: Mat3,
        flange_position: Vec3,
    ) -> Result<Self, String> {
        for (index, (axis, want)) in axes.iter().zip(AXES).enumerate() {
            if !axis.iter().zip(want).all(|(a, b)| (a - b).abs() < EPS) {
                return Err(format!(
                    "J{}의 축이 {want:?}가 아니라 {axis:?}다 (이 IK는 z, y, y, y, z, y만 푼다)",
                    index + 1
                ));
            }
        }
        if origins[2][0].abs() > EPS || origins[3][0].abs() > EPS {
            return Err("J3과 J4의 원점은 x가 0이어야 한다 (평면 두 링크)".to_owned());
        }
        if origins[5][0].abs() > EPS || origins[5][1].abs() > EPS {
            return Err("J6의 원점은 J5 축(z) 위여야 한다".to_owned());
        }
        Ok(Self {
            origins,
            flange_rotation,
            flange_position,
        })
    }

    /// 정기구학: 관절 각에서의 플랜지 자세.
    pub fn forward(&self, q: &[f64; 6]) -> Pose {
        let mut rotation = IDENTITY;
        let mut position = [0.0; 3];
        for ((origin, axis), angle) in self.origins.iter().zip(AXES).zip(q) {
            position = add(position, mat_vec(&rotation, origin));
            rotation = mat_mul(&rotation, &rot(axis, *angle));
        }
        Pose {
            position: add(position, mat_vec(&rotation, &self.flange_position)),
            rotation: mat_mul(&rotation, &self.flange_rotation),
        }
    }

    /// 역기구학: 플랜지 자세에 맞는 관절 각을 모두 낸다 (-π~π). 닿지 않으면 비어 있다. 손목 특이 자세(J5 = 0,
    /// π)에서는 J6를 0으로 둔 해 하나씩만 낸다.
    pub fn inverse(&self, target: &Pose) -> Vec<[f64; 6]> {
        let o = &self.origins;
        // J6 좌표계의 회전과 원점
        let r06 = mat_mul(&target.rotation, &transpose(&self.flange_rotation));
        let p6 = sub(target.position, mat_vec(&r06, &self.flange_position));
        // J1: 나란한 축(J2~J4) 방향으로 본 J6 원점까지의 거리는 관절과 상관없이 d다:
        // (R01 y)·(p6 - p1) = -v_x sin θ1 + v_y cos θ1 = d
        let d = o[1][1] + o[2][1] + o[3][1] + o[4][1];
        let v = sub(p6, o[0]);
        let r = v[0].hypot(v[1]);
        if r < d.abs() || r < EPS {
            return Vec::new(); // J1 축에 너무 가깝다 (어깨 특이 자세 안쪽)
        }
        let phi = v[1].atan2(v[0]);
        let a = (d / r).asin();
        let mut solutions: Vec<[f64; 6]> = Vec::new();
        for theta1 in [phi - a, phi - PI + a] {
            let r01 = rot(Z, theta1);
            let parallel = mat_vec(&r01, &Y);
            // J5: cos θ5 = (R01 y)·(R06 y)
            let c5 = dot(parallel, mat_vec(&r06, &Y)).clamp(-1.0, 1.0);
            for theta5 in [c5.acos(), -c5.acos()] {
                let s5 = theta5.sin();
                // J6: R06ᵀ(R01 y) = (s5 c6, c5, s5 s6)
                let w = mat_vec(&transpose(&r06), &parallel);
                let theta6 = if s5.abs() < EPS {
                    0.0
                } else {
                    (w[2] / s5).atan2(w[0] / s5)
                };
                // J2 + J3 + J4: Ry(θ234) = R01ᵀ R06 Ry(θ6)ᵀ Rz(θ5)ᵀ
                let m = mat_mul(
                    &mat_mul(
                        &mat_mul(&transpose(&r01), &r06),
                        &transpose(&rot(Y, theta6)),
                    ),
                    &transpose(&rot(Z, theta5)),
                );
                let theta234 = m[0][2].atan2(m[0][0]);
                // J4의 원점 = p6 - R05 p56 - R04 p45, J2의 원점 = p1 + R01 p12
                let r04 = mat_mul(&r01, &rot(Y, theta234));
                let r05 = mat_mul(&r04, &rot(Z, theta5));
                let p4 = sub(sub(p6, mat_vec(&r05, &o[5])), mat_vec(&r04, &o[4]));
                let p2 = add(o[0], mat_vec(&r01, &o[1]));
                // 평면 두 링크 (J1 좌표계의 x-z 평면): q = Ry(θ2) p23 + Ry(θ2 + θ3) p34
                let q = mat_vec(&transpose(&r01), &sub(p4, p2));
                let (l2, l3) = (o[2][2], o[3][2]);
                let c3 = (q[0] * q[0] + q[2] * q[2] - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);
                if c3.abs() > 1.0 + EPS {
                    continue; // 팔이 닿지 않는다
                }
                let t3 = c3.clamp(-1.0, 1.0).acos();
                for theta3 in [t3, -t3] {
                    let theta2 =
                        q[0].atan2(q[2]) - (l3 * theta3.sin()).atan2(l2 + l3 * theta3.cos());
                    let theta4 = theta234 - theta2 - theta3;
                    let solution = [theta1, theta2, theta3, theta4, theta5, theta6].map(wrap);
                    let duplicate = solutions.iter().any(|known| {
                        known
                            .iter()
                            .zip(&solution)
                            .all(|(a, b)| wrap(a - b).abs() < EPS)
                    });
                    if !duplicate {
                        solutions.push(solution);
                    }
                }
            }
        }
        solutions
    }
}

impl Kinematics {
    /// 야코비안: 관절 속도 → 플랜지의 선속도 [m/s]와 각속도 [rad/s] (베이스 좌표계). 행은 vx, vy, vz, wx, wy, wz이고
    /// 열은 관절이다.
    pub fn jacobian(&self, q: &[f64; 6]) -> [[f64; 6]; 6] {
        let mut rotation = IDENTITY;
        let mut position = [0.0; 3];
        let mut axes = [[0.0; 3]; 6];
        let mut points = [[0.0; 3]; 6];
        for i in 0..6 {
            position = add(position, mat_vec(&rotation, &self.origins[i]));
            axes[i] = mat_vec(&rotation, &AXES[i]);
            points[i] = position;
            rotation = mat_mul(&rotation, &rot(AXES[i], q[i]));
        }
        let tip = add(position, mat_vec(&rotation, &self.flange_position));
        let mut jacobian = [[0.0; 6]; 6];
        for i in 0..6 {
            let linear = cross(axes[i], sub(tip, points[i]));
            for row in 0..3 {
                jacobian[row][i] = linear[row];
                jacobian[row + 3][i] = axes[i][row];
            }
        }
        jacobian
    }

    /// 손끝 속도 [vx, vy, vz, wx, wy, wz]에 맞는 관절 속도 (감쇠 최소 제곱: q̇ = Jᵀ (J Jᵀ + λ² I)⁻¹ x). 특이 자세
    /// 가까이에서는 손끝 속도를 다 맞추지 못하는 대신 관절 속도가 터지지 않는다.
    pub fn joint_velocity(&self, q: &[f64; 6], twist: &[f64; 6], damping: f64) -> [f64; 6] {
        let j = self.jacobian(q);
        let mut a = [[0.0; 6]; 6];
        for (r, row) in a.iter_mut().enumerate() {
            for (c, value) in row.iter_mut().enumerate() {
                *value = (0..6).map(|k| j[r][k] * j[c][k]).sum::<f64>();
                if r == c {
                    *value += damping * damping;
                }
            }
        }
        let y = solve6(a, *twist).unwrap_or([0.0; 6]);
        std::array::from_fn(|k| (0..6).map(|r| j[r][k] * y[r]).sum())
    }
}

/// 두 플랜지 자세 사이의 곧은 길 (직선 이동): 위치는 곧은 선을 따라가고, 방향은 가장 짧은 회전으로 고르게
/// 바뀐다. `u`는 0(시작)에서 1(끝)까지다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseLine {
    start: Pose,
    /// 위치 변화 [m]
    shift: Vec3,
    /// 시작 자세의 좌표계에서 본 회전 축 (단위 벡터)
    axis: Vec3,
    /// 도는 각 [rad] (0~π)
    angle: f64,
}

impl PoseLine {
    pub fn new(start: &Pose, end: &Pose) -> Self {
        let relative = mat_mul(&transpose(&start.rotation), &end.rotation);
        let (axis, angle) = axis_angle(&relative);
        Self {
            start: *start,
            shift: sub(end.position, start.position),
            axis,
            angle,
        }
    }

    /// 곧은 선의 길이 [m]
    pub fn length(&self) -> f64 {
        dot(self.shift, self.shift).sqrt()
    }

    /// 도는 각 [rad] (0~π)
    pub fn angle(&self) -> f64 {
        self.angle
    }

    /// 길 위의 자세 (u = 0이면 시작, 1이면 끝).
    pub fn at(&self, u: f64) -> Pose {
        Pose {
            position: add(self.start.position, self.shift.map(|v| v * u)),
            rotation: mat_mul(&self.start.rotation, &rot(self.axis, self.angle * u)),
        }
    }
}

/// 6×6 연립 방정식 (부분 피벗 가우스 소거). 풀 수 없으면 없다.
fn solve6(mut a: [[f64; 6]; 6], mut b: [f64; 6]) -> Option<[f64; 6]> {
    for col in 0..6 {
        let pivot = (col..6).max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))?;
        if a[pivot][col].abs() < 1e-15 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..6 {
            let factor = a[row][col] / a[col][col];
            let pivot_row = a[col];
            for (k, value) in a[row].iter_mut().enumerate().skip(col) {
                *value -= factor * pivot_row[k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = [0.0; 6];
    for row in (0..6).rev() {
        let rest: f64 = (row + 1..6).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - rest) / a[row][row];
    }
    Some(x)
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// 각을 -π~π로 맞춘다.
pub fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn mat_vec(m: &Mat3, v: &Vec3) -> Vec3 {
    [dot(m[0], *v), dot(m[1], *v), dot(m[2], *v)]
}

fn transpose(m: &Mat3) -> Mat3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let t = transpose(b);
    [
        [dot(a[0], t[0]), dot(a[0], t[1]), dot(a[0], t[2])],
        [dot(a[1], t[0]), dot(a[1], t[1]), dot(a[1], t[2])],
        [dot(a[2], t[0]), dot(a[2], t[1]), dot(a[2], t[2])],
    ]
}

/// 회전 행렬의 축(단위 벡터)과 각 (0~π). 사원수를 거치고 대각 성분이 가장 큰 쪽에서 계산해, 각이 π 가까이에서도
/// 정확하다. 돌지 않으면 축은 x다.
fn axis_angle(m: &Mat3) -> (Vec3, f64) {
    let trace = m[0][0] + m[1][1] + m[2][2];
    let [w, x, y, z] = if trace > 0.0 {
        let s = 2.0 * (1.0 + trace).sqrt();
        [
            0.25 * s,
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
        ]
    } else if m[0][0] >= m[1][1] && m[0][0] >= m[2][2] {
        let s = 2.0 * (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt();
        [
            (m[2][1] - m[1][2]) / s,
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
        ]
    } else if m[1][1] >= m[2][2] {
        let s = 2.0 * (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt();
        [
            (m[0][2] - m[2][0]) / s,
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
        ]
    } else {
        let s = 2.0 * (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt();
        [
            (m[1][0] - m[0][1]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
        ]
    };
    // 가장 짧은 회전 (w ≥ 0)
    let sign = if w < 0.0 { -1.0 } else { 1.0 };
    let v = [x * sign, y * sign, z * sign];
    let size = dot(v, v).sqrt();
    if size < 1e-15 {
        return (X, 0.0);
    }
    (v.map(|c| c / size), 2.0 * size.atan2(w * sign))
}

/// 단위 축 둘레로 angle만큼 도는 회전 (로드리게스).
fn rot(axis: Vec3, angle: f64) -> Mat3 {
    let (s, c) = angle.sin_cos();
    let [x, y, z] = axis;
    let t = 1.0 - c;
    [
        [c + x * x * t, x * y * t - z * s, x * z * t + y * s],
        [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
        [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 설계의 지금 값과 같은 꼴 (params/arm.toml: 160, 220, 180, 63, 63, 50 mm)
    fn arm() -> Kinematics {
        let mm = |x: f64, y: f64, z: f64| [x * 1e-3, y * 1e-3, z * 1e-3];
        let flange = mat_mul(&rot(X, -PI / 2.0), &rot(Z, -PI / 2.0));
        Kinematics::new(
            [
                mm(0.0, 0.0, 0.0),
                mm(0.0, 0.0, 160.0),
                mm(0.0, 0.0, 220.0),
                mm(0.0, 0.0, 180.0),
                mm(0.0, 63.0, 0.0),
                mm(0.0, 0.0, 63.0),
            ],
            AXES,
            flange,
            mm(0.0, 50.0, 0.0),
        )
        .unwrap()
    }

    fn same_pose(a: &Pose, b: &Pose) -> bool {
        let rotation = a
            .rotation
            .iter()
            .flatten()
            .zip(b.rotation.iter().flatten())
            .all(|(x, y)| (x - y).abs() < 1e-9);
        let position = a
            .position
            .iter()
            .zip(&b.position)
            .all(|(x, y)| (x - y).abs() < 1e-9);
        rotation && position
    }

    #[test]
    fn zero_pose_stands_up() {
        let pose = arm().forward(&[0.0; 6]);
        // 160 + 220 + 180 + 63 = 623 mm 위, 63 + 50 = 113 mm 옆
        assert!(same_pose(
            &pose,
            &Pose {
                position: [0.0, 0.113, 0.623],
                rotation: pose.rotation,
            }
        ));
    }

    #[test]
    fn inverse_finds_solutions_that_all_reach_the_pose() {
        let arm = arm();
        let q = [0.3, -0.5, 1.1, 0.4, 0.8, -1.2];
        let target = arm.forward(&q);
        let solutions = arm.inverse(&target);
        // 이 자세는 어깨를 뒤집은 갈래가 닿지 않아 해가 넷이다 (해의 수는 EAIK 대조 시험이 본다)
        assert_eq!(solutions.len(), 4, "{solutions:?}");
        for solution in &solutions {
            assert!(same_pose(&arm.forward(solution), &target), "{solution:?}");
        }
        assert!(
            solutions
                .iter()
                .any(|s| s.iter().zip(&q).all(|(a, b)| wrap(a - b).abs() < 1e-9))
        );
    }

    #[test]
    fn unreachable_pose_has_no_solution() {
        let arm = arm();
        let mut target = arm.forward(&[0.0; 6]);
        target.position = [0.0, 0.0, 2.0]; // 팔 길이보다 멀다
        assert!(arm.inverse(&target).is_empty());
    }

    #[test]
    fn jacobian_matches_finite_differences() {
        let arm = arm();
        let q = [0.3, -0.5, 1.1, 0.4, 0.8, -1.2];
        let jacobian = arm.jacobian(&q);
        let h = 1e-6;
        let base = arm.forward(&q);
        for i in 0..6 {
            let mut moved = q;
            moved[i] += h;
            let pose = arm.forward(&moved);
            for (row, (moved_at, base_at)) in pose.position.iter().zip(&base.position).enumerate() {
                let numeric = (moved_at - base_at) / h;
                assert!((numeric - jacobian[row][i]).abs() < 1e-5, "v{row} q{i}");
            }
            // 각속도: R(q + h) R(q)ᵀ ≈ I + [ω]× h
            let d = mat_mul(&pose.rotation, &transpose(&base.rotation));
            let omega = [
                (d[2][1] - d[1][2]) / (2.0 * h),
                (d[0][2] - d[2][0]) / (2.0 * h),
                (d[1][0] - d[0][1]) / (2.0 * h),
            ];
            for (row, w) in omega.iter().enumerate() {
                assert!((w - jacobian[row + 3][i]).abs() < 1e-5, "w{row} q{i}");
            }
        }
    }

    #[test]
    fn joint_velocity_gives_the_asked_tool_velocity() {
        let arm = arm();
        let q = [0.3, -0.5, 1.1, 0.4, 0.8, -1.2];
        let twist = [0.02, -0.01, 0.005, 0.0, 0.1, -0.05];
        let qd = arm.joint_velocity(&q, &twist, 1e-6);
        let jacobian = arm.jacobian(&q);
        for row in 0..6 {
            let got: f64 = (0..6).map(|k| jacobian[row][k] * qd[k]).sum();
            assert!((got - twist[row]).abs() < 1e-6, "{row}: {got}");
        }
    }

    #[test]
    fn pose_line_goes_straight_and_turns_evenly() {
        let arm = arm();
        let start = arm.forward(&[0.3, -0.5, 1.1, 0.4, 0.8, -1.2]);
        let end = arm.forward(&[0.1, -0.3, 0.9, 0.7, 1.0, -0.6]);
        let line = PoseLine::new(&start, &end);
        assert!(same_pose(&line.at(0.0), &start));
        assert!(same_pose(&line.at(1.0), &end));
        let middle = line.at(0.5);
        for i in 0..3 {
            let half_way = (start.position[i] + end.position[i]) / 2.0;
            assert!((middle.position[i] - half_way).abs() < 1e-12);
        }
        // 가운데는 시작에서 절반만큼 돌았고, 나머지 절반은 같은 축으로 돈다
        let first = PoseLine::new(&start, &middle);
        assert!((first.angle() - line.angle() / 2.0).abs() < 1e-9);
        assert!((first.length() - line.length() / 2.0).abs() < 1e-12);
        assert!(same_pose(&first.at(2.0), &end));
    }

    #[test]
    fn pose_line_reaches_half_turns_and_no_turn() {
        let size = (0.3f64 * 0.3 + 0.5 * 0.5 + 0.8 * 0.8).sqrt();
        let axis = [0.3 / size, -0.5 / size, 0.8 / size];
        let start = Pose {
            rotation: rot(Z, 0.4),
            position: [0.0; 3],
        };
        for angle in [PI, PI - 1e-7, 2.0, 1e-9, 0.0] {
            let end = Pose {
                rotation: mat_mul(&start.rotation, &rot(axis, angle)),
                position: [0.1, 0.0, 0.0],
            };
            let line = PoseLine::new(&start, &end);
            assert!(same_pose(&line.at(1.0), &end), "{angle}");
            assert!(
                (line.angle() - angle).abs() < 1e-7,
                "{angle}: {}",
                line.angle()
            );
        }
    }

    #[test]
    fn other_axis_layouts_are_refused() {
        let mut axes = AXES;
        axes[4] = X;
        let message = Kinematics::new([[0.0; 3]; 6], axes, IDENTITY, [0.0; 3]).unwrap_err();
        assert!(message.contains("J5의 축"), "{message}");
    }
}
