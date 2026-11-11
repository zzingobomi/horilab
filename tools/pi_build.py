"""파이용 실행 파일을 PC에서 교차 빌드한다.

결정과 이유는 docs/decisions/0026-pi-runtime-deploy-records.md의 3에 있다.

    uv run tools/pi_build.py -p horilab-bench --bin smoke         musl 정적 실행 파일 (기본)
    uv run tools/pi_build.py --gnu -p horilab-bench --bin smoke   glibc 2.35판 (비교용)

- cargo-zigbuild와 zig는 uv 개발 의존성으로 레포 환경에 있다. PC에 따로 설치하지 않는다.
- cargo-zigbuild가 레포 환경의 zig를 스스로 찾지 못해서(Windows), 깔린 zig의 자리를 알려 준다.
- 나머지 인자는 cargo에 그대로 넘긴다. 실행 파일은 target/<대상>/release/에 생긴다.
"""

import os
import subprocess
import sys
from collections.abc import Sequence
from pathlib import Path

import ziglang

ROOT = Path(__file__).resolve().parents[1]
MUSL = "aarch64-unknown-linux-musl"
GNU = "aarch64-unknown-linux-gnu.2.35"  # Ubuntu 22.04의 glibc. 이보다 새 glibc에서도 돈다


def zig_path() -> Path:
    folder = Path(ziglang.__file__).parent
    for name in ("zig.exe", "zig"):
        if (folder / name).is_file():
            return folder / name
    raise FileNotFoundError(f"zig가 없다: {folder}. uv sync를 다시 한다")


def command(args: Sequence[str]) -> list[str]:
    rest = list(args)
    target = MUSL
    if "--gnu" in rest:
        rest.remove("--gnu")
        target = GNU
    return ["cargo", "zigbuild", "--release", "--target", target, *rest]


def main(argv: Sequence[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    env = os.environ | {"CARGO_ZIGBUILD_ZIG_PATH": str(zig_path())}
    return subprocess.call(command(args), cwd=ROOT, env=env)


if __name__ == "__main__":
    sys.exit(main())
