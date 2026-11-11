"""tools/pi_build.py 검사 (docs/decisions/0026-pi-runtime-deploy-records.md의 3)."""

from tools import pi_build


def test_zig_is_in_repo_environment() -> None:
    # uv sync가 레포 환경에 zig를 깐다. PC에 따로 설치하지 않는다.
    assert pi_build.zig_path().is_file()


def test_musl_static_binary_is_default() -> None:
    command = pi_build.command(["-p", "horilab-bench"])
    assert command[:5] == ["cargo", "zigbuild", "--release", "--target", pi_build.MUSL]
    assert command[5:] == ["-p", "horilab-bench"]


def test_gnu_flag_selects_glibc_target() -> None:
    command = pi_build.command(["--gnu", "-p", "horilab-bench"])
    assert pi_build.GNU in command
    assert "--gnu" not in command
