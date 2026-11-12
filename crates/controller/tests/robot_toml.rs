//! 레포의 로봇 설정(data/robot.toml)이 컨트롤러에 읽히는지 본다.
//! 설계(params/arm.toml)와 맞는지는 tests/test_robot_settings.py가 본다.

use std::path::Path;

use horilab_controller::settings::Settings;

#[test]
fn repo_settings_load() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data/robot.toml");
    let settings = Settings::load(&path).unwrap_or_else(|error| panic!("{error}"));
    // 관절 하나 시험의 J3부터 있다 (0021)
    let j3 = settings.joint("j3").expect("j3");
    assert_eq!((j3.id, j3.model), (3, "STS3250"));
}
