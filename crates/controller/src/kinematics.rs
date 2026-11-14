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
    fn other_axis_layouts_are_refused() {
        let mut axes = AXES;
        axes[4] = X;
        let message = Kinematics::new([[0.0; 3]; 6], axes, IDENTITY, [0.0; 3]).unwrap_err();
        assert!(message.contains("J5의 축"), "{message}");
    }
}
