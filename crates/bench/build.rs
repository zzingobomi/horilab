//! 실행 파일에 빌드한 커밋을 넣는다. 시험 기록(meta.json)에 어떤 코드로 쟀는지 남긴다 (0024의 기록 형식).

use std::process::Command;

fn git(args: &[&str]) -> Option<String> {
    let output = Command::new("git").args(args).output().ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).trim().to_owned())
}

fn main() {
    let commit = git(&["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".to_owned());
    // git status가 색인 파일을 고쳐 쓰지 않게 한다. 고쳐 쓰면 아래 감시 때문에 빌드마다 다시 돈다.
    let uncommitted = match git(&["--no-optional-locks", "status", "--porcelain"]) {
        Some(status) if status.is_empty() => "false",
        Some(_) => "true",
        None => "unknown",
    };
    println!("cargo:rustc-env=HORILAB_COMMIT={commit}");
    println!("cargo:rustc-env=HORILAB_UNCOMMITTED={uncommitted}");
    // 이 크레이트와 서보 크레이트의 코드, 커밋, 색인이 바뀌면 다시 읽는다.
    for path in [
        "build.rs",
        "src",
        "../servo/src",
        "../../.git/logs/HEAD",
        "../../.git/index",
    ] {
        println!("cargo:rerun-if-changed={path}");
    }
}
