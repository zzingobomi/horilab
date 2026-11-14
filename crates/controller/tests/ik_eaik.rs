//! 컨트롤러의 IK를 설계의 EAIK와 견준다 (docs/decisions/0033의 만드는 순서 2).
//!
//! 설계 패키지의 `python -m horilab_design.ik_cases`가 관절 범위 안의 무작위 자세와 그 EAIK 해를 JSON으로
//! 낸다 (가상환경의 Python으로 띄운다, `uv sync`가 먼저). 이 시험은 그 기구학 값으로 컨트롤러의 IK를 만들고,
//! 자세마다 정기구학이 같은지, 해가 모두 자세에 닿는지, 처음 관절 각과 EAIK의 해를 모두 찾는지 본다.

use std::path::Path;
use std::process::{Command, Stdio};

use horilab_controller::kinematics::{Kinematics, Mat3, Pose, Vec3, wrap};
use serde_json::Value;

fn vec3(value: &Value) -> Vec3 {
    let v: Vec<f64> = serde_json::from_value(value.clone()).unwrap();
    [v[0], v[1], v[2]]
}

fn mat3(value: &Value) -> Mat3 {
    let rows: Vec<Vec<f64>> = serde_json::from_value(value.clone()).unwrap();
    [
        [rows[0][0], rows[0][1], rows[0][2]],
        [rows[1][0], rows[1][1], rows[1][2]],
        [rows[2][0], rows[2][1], rows[2][2]],
    ]
}

fn angles(value: &Value) -> [f64; 6] {
    let v: Vec<f64> = serde_json::from_value(value.clone()).unwrap();
    v.try_into().unwrap()
}

fn eaik_cases() -> Value {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let python = if cfg!(windows) {
        root.join(".venv/Scripts/python.exe")
    } else {
        root.join(".venv/bin/python")
    };
    let output = Command::new(&python)
        .args([
            "-m",
            "horilab_design.ik_cases",
            "--count",
            "300",
            "--seed",
            "7",
        ])
        .current_dir(&root)
        .stderr(Stdio::inherit())
        .output()
        .unwrap_or_else(|e| panic!("{}로 EAIK 자세를 만들지 못했다: {e}", python.display()));
    assert!(
        output.status.success(),
        "EAIK 자세를 만드는 명령이 실패했다"
    );
    serde_json::from_slice(&output.stdout).unwrap()
}

fn same_angles(a: &[f64; 6], b: &[f64; 6], tolerance: f64) -> bool {
    a.iter().zip(b).all(|(x, y)| wrap(x - y).abs() < tolerance)
}

fn pose_error(a: &Pose, b: &Pose) -> f64 {
    let rotation = a
        .rotation
        .iter()
        .flatten()
        .zip(b.rotation.iter().flatten())
        .map(|(x, y)| (x - y).abs());
    let position = a
        .position
        .iter()
        .zip(&b.position)
        .map(|(x, y)| (x - y).abs());
    rotation.chain(position).fold(0.0, f64::max)
}

#[test]
fn inverse_kinematics_matches_eaik() {
    let data = eaik_cases();
    let origins: Vec<Vec3> = data["origins"]
        .as_array()
        .unwrap()
        .iter()
        .map(vec3)
        .collect();
    let axes: Vec<Vec3> = data["axes"].as_array().unwrap().iter().map(vec3).collect();
    let arm = Kinematics::new(
        origins.try_into().unwrap(),
        axes.try_into().unwrap(),
        mat3(&data["flange_rotation"]),
        vec3(&data["flange_position"]),
    )
    .unwrap();
    let cases = data["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 300);
    for (index, case) in cases.iter().enumerate() {
        let q = angles(&case["q"]);
        let target = Pose {
            rotation: mat3(&case["rotation"]),
            position: vec3(&case["position"]),
        };
        // 정기구학이 설계와 같다
        let error = pose_error(&arm.forward(&q), &target);
        assert!(error < 1e-9, "{index}: 정기구학이 {error} 다르다");
        // 해가 모두 그 자세에 닿고, 처음 관절 각이 그 가운데 있다
        let solutions = arm.inverse(&target);
        for solution in &solutions {
            let error = pose_error(&arm.forward(solution), &target);
            assert!(
                error < 1e-7,
                "{index}: 해 {solution:?}가 자세에서 {error} 벗어난다"
            );
        }
        assert!(
            solutions.iter().any(|s| same_angles(s, &q, 1e-6)),
            "{index}: 처음 관절 각 {q:?}를 찾지 못했다: {solutions:?}"
        );
        // EAIK의 해를 모두 찾고, 수도 같다
        let eaik: Vec<[f64; 6]> = case["solutions"]
            .as_array()
            .unwrap()
            .iter()
            .map(angles)
            .collect();
        for expected in &eaik {
            assert!(
                solutions.iter().any(|s| same_angles(s, expected, 1e-6)),
                "{index}: EAIK의 해 {expected:?}를 찾지 못했다: {solutions:?}"
            );
        }
        assert_eq!(
            solutions.len(),
            eaik.len(),
            "{index}: 해의 수가 EAIK와 다르다"
        );
    }
}
