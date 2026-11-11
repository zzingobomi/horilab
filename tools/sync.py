"""git에서 뺀 데이터 폴더를 구글 드라이브에 레포와 같은 경로로 올리고 받는다.

결정과 이유는 docs/decisions/0022-records-and-sync.md에 있다.

    uv run tools/sync.py login --client <앱 파일>       PC마다 처음 한 번: 구글 로그인
    uv run tools/sync.py push [경로 ...]                 새로 생기거나 바뀐 파일을 올린다
    uv run tools/sync.py pull [경로 ...] [--max-age 3d]  받는다

- 옮기는 것은 SYNC_DIRS의 폴더뿐이다. 경로를 주면 그 아래만 옮긴다.
- 어느 쪽에서도 지우지 않는다. 받는 쪽 파일이 더 새것이면 덮어쓰지 않는다.
- --dry-run을 주면 옮기지 않고 옮길 파일만 보여 준다.
- rclone 실행 파일, 로그인 설정, 캐시는 tools/local/(git 제외)에 둔다. PC에 설치하지 않는다.
- 로그인에는 구글 클라우드에서 만든 앱(OAuth 클라이언트)을 쓴다. rclone이 빌려주던 앱은
  2026년에 끝난다. 앱 만드는 순서는 docs/setup.md에 있다.
"""

import argparse
import hashlib
import io
import json
import platform
import subprocess
import sys
import urllib.request
import zipfile
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOCAL = ROOT / "tools" / "local"
CONFIG = LOCAL / "rclone.conf"  # 구글 로그인 토큰이 들어 있다. git에 넣지 않는다
CACHE = LOCAL / "cache"

# 동기화할 데이터 폴더 (레포 기준 경로). 표준 경로를 정할 때마다 여기와 .gitignore에 같이 넣는다.
# 목록이 맞는지는 tests/test_sync.py가 확인한다.
SYNC_DIRS: tuple[str, ...] = ("records", "videos")

REMOTE = "gdrive"  # rclone 설정 안의 이름
REMOTE_ROOT = f"{REMOTE}:horilab"  # 드라이브에서 레포 루트에 해당하는 폴더

RCLONE_VERSION = "v1.76.0"
# https://downloads.rclone.org/<버전>/SHA256SUMS 의 값. 버전을 올리면 같이 바꾼다.
RCLONE_SHA256 = {
    "windows-amd64": "8b4fd2737ade4e70c057ef4a49deb7cffe94b0fb9d7587c189d149823f21a1dc",
}

# rclone 종료 코드 3, 4: 폴더나 파일이 없다
NOT_FOUND_CODES = (3, 4)


@dataclass(frozen=True)
class Rclone:
    """레포 안의 rclone. PC의 rclone 설정(%APPDATA%)은 쓰지 않는다."""

    exe: Path
    config: Path
    cache: Path

    def command(self, *args: str) -> list[str]:
        return [str(self.exe), *args, "--config", str(self.config), "--cache-dir", str(self.cache)]

    def run(self, *args: str) -> None:
        """출력을 그대로 보여 주며 돌리고, 실패하면 멈춘다."""
        code = subprocess.run(self.command(*args), check=False).returncode
        if code != 0:
            raise SystemExit(f"rclone이 실패했다 (종료 코드 {code})")

    def output(self, *args: str) -> subprocess.CompletedProcess[str]:
        """출력을 화면에 보이지 않고 받아 온다."""
        return subprocess.run(
            self.command(*args), check=False, capture_output=True, text=True, encoding="utf-8"
        )


def rclone_path() -> Path:
    name = "rclone.exe" if sys.platform == "win32" else "rclone"
    return LOCAL / f"rclone-{RCLONE_VERSION}" / name


def platform_name() -> str:
    """rclone 배포 파일 이름의 플랫폼 부분 (예: windows-amd64)."""
    system = {"win32": "windows", "linux": "linux", "darwin": "osx"}.get(sys.platform, sys.platform)
    machine = platform.machine().lower()
    arch = {"x86_64": "amd64", "aarch64": "arm64"}.get(machine, machine)
    return f"{system}-{arch}"


def ensure_rclone() -> Path:
    """rclone이 없으면 공식 배포 파일을 받아 해시를 확인하고 실행 파일만 꺼낸다."""
    path = rclone_path()
    if path.exists():
        return path
    name = platform_name()
    expected = RCLONE_SHA256.get(name)
    if expected is None:
        raise SystemExit(f"{name}용 rclone 해시가 없다. tools/sync.py의 RCLONE_SHA256에 더한다")
    stem = f"rclone-{RCLONE_VERSION}-{name}"
    url = f"https://downloads.rclone.org/{RCLONE_VERSION}/{stem}.zip"
    print(f"rclone을 받는다: {url}")
    # 배포 서버가 Python 기본 User-Agent(Python-urllib)를 403으로 막는다.
    request = urllib.request.Request(url, headers={"User-Agent": "rclone-download"})
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            data: bytes = response.read()
    except OSError as error:
        raise SystemExit(f"rclone을 받지 못했다: {error}") from None
    actual = hashlib.sha256(data).hexdigest()
    if actual != expected:
        raise SystemExit(f"받은 rclone의 해시가 다르다: {actual}")
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        binary = archive.read(f"{stem}/{path.name}")
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + ".part")
    partial.write_bytes(binary)
    partial.chmod(0o755)
    partial.replace(path)
    return path


def to_repo_path(raw: str, root: Path) -> str:
    """명령줄의 경로를 레포 기준 '/' 경로로 바꾼다. 레포 밖이나 '..'은 받지 않는다."""
    path = Path(raw)
    if path.is_absolute():
        try:
            path = path.resolve().relative_to(root.resolve())
        except ValueError:
            raise SystemExit(f"레포 밖의 경로다: {raw}") from None
    elif path.anchor:
        raise SystemExit(f"레포 기준 경로로 준다: {raw}")
    parts = [part for part in path.parts if part != "."]
    if not parts or ".." in parts:
        raise SystemExit(f"쓸 수 없는 경로다: {raw}")
    return "/".join(parts)


def select(
    paths: Sequence[str], sync_dirs: Sequence[str], root: Path
) -> dict[str, tuple[str, ...]]:
    """옮길 것을 동기화 목록의 폴더별로 묶는다.

    값은 그 폴더 안에서 고른 경로이고, 빈 튜플이면 폴더 전체다. 경로를 주지 않으면 목록 전체다.
    """
    if not paths:
        return dict.fromkeys(sync_dirs, ())
    whole: set[str] = set()
    parts: dict[str, set[str]] = {}
    for raw in paths:
        rel = to_repo_path(raw, root)
        base = next((d for d in sync_dirs if rel == d or rel.startswith(d + "/")), None)
        if base is None:
            listed = ", ".join(sync_dirs) or "비어 있음"
            raise SystemExit(f"동기화 목록 밖의 경로다: {raw} (목록: {listed})")
        sub = rel[len(base) + 1 :]
        if sub:
            parts.setdefault(base, set()).add(sub)
        else:
            whole.add(base)
    return {
        d: () if d in whole else tuple(sorted(parts[d]))
        for d in sync_dirs
        if d in whole or d in parts
    }


def _escape_glob(path: str) -> str:
    """rclone 필터에서 글롭 문자를 글자 그대로 읽게 한다."""
    return "".join("\\" + ch if ch in "\\*?[]{}" else ch for ch in path)


def copy_args(
    source: str,
    dest: str,
    subpaths: Sequence[str],
    *,
    max_age: str | None = None,
    dry_run: bool = False,
) -> list[str]:
    """rclone copy 인자. copy는 지우지 않고, --update는 받는 쪽 파일이 더 새것이면 건너뛴다."""
    args = ["copy", source, dest, "--update", "--verbose"]
    for sub in subpaths:
        pattern = "/" + _escape_glob(sub)
        # 고른 경로가 파일이어도 폴더여도 맞게 둘 다 넣는다.
        args += ["--include", pattern, "--include", pattern + "/**"]
    if max_age is not None:
        args += ["--max-age", max_age]
    if dry_run:
        args.append("--dry-run")
    return args


def exists(rclone: Rclone, path: str) -> bool:
    result = rclone.output("lsjson", "--stat", path)
    if result.returncode == 0:
        return True
    if result.returncode in NOT_FOUND_CODES:
        return False
    raise SystemExit(f"rclone이 실패했다: {result.stderr.strip()}")


def push(
    rclone: Rclone,
    selection: Mapping[str, Sequence[str]],
    *,
    root: Path = ROOT,
    remote_root: str = REMOTE_ROOT,
    dry_run: bool = False,
) -> None:
    for base, subpaths in selection.items():
        local = root / base
        if not local.is_dir():
            print(f"{base}: 이 PC에 없어서 건너뛴다")
            continue
        print(f"{base} → 드라이브")
        rclone.run(*copy_args(str(local), f"{remote_root}/{base}", subpaths, dry_run=dry_run))


def pull(
    rclone: Rclone,
    selection: Mapping[str, Sequence[str]],
    *,
    root: Path = ROOT,
    remote_root: str = REMOTE_ROOT,
    max_age: str | None = None,
    dry_run: bool = False,
) -> None:
    for base, subpaths in selection.items():
        remote = f"{remote_root}/{base}"
        if not exists(rclone, remote):
            print(f"{base}: 드라이브에 없어서 건너뛴다")
            continue
        print(f"드라이브 → {base}")
        args = copy_args(remote, str(root / base), subpaths, max_age=max_age, dry_run=dry_run)
        rclone.run(*args)


def remote_settings(rclone: Rclone) -> dict[str, str] | None:
    """이 PC의 드라이브 연결 설정. 로그인하지 않았으면 None."""
    if not rclone.config.exists():
        return None
    # 설정에는 토큰이 들어 있어서 화면에 보이지 않게 받는다.
    result = rclone.output("config", "dump")
    if result.returncode != 0:
        raise SystemExit(f"rclone 설정을 읽지 못했다: {result.stderr.strip()}")
    return json.loads(result.stdout).get(REMOTE)


def read_client(path: Path) -> tuple[str, str]:
    """구글 클라우드에서 받은 OAuth 클라이언트 파일(데스크톱 앱)에서 ID와 보안 비밀을 꺼낸다."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise SystemExit(f"앱 파일을 읽지 못했다: {error}") from None
    client = data.get("installed") if isinstance(data, dict) else None
    if not isinstance(client, dict) or "client_id" not in client or "client_secret" not in client:
        raise SystemExit(f"데스크톱 앱 유형의 OAuth 클라이언트 파일이 아니다: {path}")
    return client["client_id"], client["client_secret"]


def login(rclone: Rclone, client_file: Path | None) -> None:
    """구글 로그인을 하고 드라이브 연결을 설정한다.

    앱 파일은 PC마다 처음 한 번만 준다. 다시 로그인할 때는 설정에 남은 앱을 쓴다.
    """
    settings = remote_settings(rclone) or {}
    if client_file is not None:
        client_id, client_secret = read_client(client_file)
    elif settings.get("client_id") and settings.get("client_secret"):
        client_id, client_secret = settings["client_id"], settings["client_secret"]
    else:
        raise SystemExit("구글 클라우드에서 받은 앱 파일을 준다: login --client <파일>")
    rclone.config.parent.mkdir(parents=True, exist_ok=True)
    if settings:
        rclone.run("config", "delete", REMOTE)
    print("브라우저가 열리면, 데이터를 둘 구글 계정으로 로그인해 허용한다.")
    create = rclone.command(
        "config",
        "create",
        REMOTE,
        "drive",
        "scope=drive",
        f"client_id={client_id}",
        f"client_secret={client_secret}",
    )
    # 끝에 표준 출력으로 토큰이 든 설정을 보여 주므로 버린다. 로그인 주소는 표준 오류로 나온다.
    code = subprocess.run(create, check=False, stdout=subprocess.DEVNULL).returncode
    if code != 0:
        raise SystemExit(f"로그인하지 못했다 (종료 코드 {code})")
    print("로그인했다. 드라이브 사용량:")
    rclone.run("about", f"{REMOTE}:")
    folder = REMOTE_ROOT.split(":", 1)[1]
    there = exists(rclone, REMOTE_ROOT)
    print(f"드라이브의 {folder} 폴더: " + ("있다" if there else "없다. 처음 올릴 때 생긴다"))


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="uv run tools/sync.py",
        description="git에서 뺀 데이터 폴더를 구글 드라이브에 레포와 같은 경로로 올리고 받는다 (0022).",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    login_parser = commands.add_parser("login", help="PC마다 처음 한 번: 구글 로그인")
    login_parser.add_argument(
        "--client",
        type=Path,
        metavar="파일",
        help="구글 클라우드에서 받은 OAuth 클라이언트 파일 (PC마다 처음 한 번)",
    )
    for name, text in (("push", "새로 생기거나 바뀐 파일을 올린다"), ("pull", "받는다")):
        command = commands.add_parser(name, help=text)
        command.add_argument(
            "paths", nargs="*", metavar="경로", help="이 경로 아래만 옮긴다. 없으면 목록 전체"
        )
        command.add_argument(
            "--dry-run", action="store_true", help="옮기지 않고 옮길 파일만 보여 준다"
        )
        if name == "pull":
            command.add_argument(
                "--max-age",
                metavar="기간",
                help="이 기간 안에 바뀐 파일만 받는다 (예: 3d, 12h, 2026-11-01)",
            )
    args = parser.parse_args(argv)
    # 출력을 파일로 받을 때도 이 스크립트의 안내가 rclone 출력보다 먼저 보이게 한다.
    if isinstance(sys.stdout, io.TextIOWrapper):
        sys.stdout.reconfigure(line_buffering=True)

    rclone = Rclone(ensure_rclone(), CONFIG, CACHE)
    if args.command == "login":
        login(rclone, args.client)
        return
    if remote_settings(rclone) is None:
        raise SystemExit(
            "이 PC는 아직 로그인하지 않았다: uv run tools/sync.py login --client <파일>"
        )
    if not SYNC_DIRS:
        print("동기화 목록(tools/sync.py의 SYNC_DIRS)이 비어 있다. 표준 경로를 정하면 넣는다.")
        return
    selection = select(args.paths, SYNC_DIRS, ROOT)
    if args.command == "push":
        push(rclone, selection, dry_run=args.dry_run)
    else:
        pull(rclone, selection, max_age=args.max_age, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
