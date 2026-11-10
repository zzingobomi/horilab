"""tools/sync.py 검사 (docs/decisions/0022-records-and-sync.md).

rclone으로 실제로 올리고 받는 테스트는 구글 드라이브 대신 임시 폴더를 쓴다.
rclone은 tools/local/에 처음 쓸 때 받으므로, 받기 전에는 건너뛴다.
"""

import os
import subprocess
import time
from pathlib import Path

import pytest

from tools import sync

ROOT = Path(__file__).resolve().parents[1]
DIRS = ("logs", "records/joint")  # 테스트용 동기화 목록


def _ignored(path: str) -> bool:
    result = subprocess.run(["git", "check-ignore", "-q", path], cwd=ROOT, check=False)
    return result.returncode == 0


def test_login_settings_stay_out_of_git() -> None:
    # 설정 파일에는 구글 로그인 토큰이 들어 있다.
    assert _ignored(sync.CONFIG.relative_to(ROOT).as_posix())
    assert _ignored(sync.rclone_path().relative_to(ROOT).as_posix())


def test_sync_dirs_are_repo_paths_and_git_ignored() -> None:
    assert len(set(sync.SYNC_DIRS)) == len(sync.SYNC_DIRS)
    for d in sync.SYNC_DIRS:
        assert sync.to_repo_path(d, ROOT) == d, f"{d}: 레포 기준 '/' 경로로 쓴다"
        assert not any(d.startswith(o + "/") for o in sync.SYNC_DIRS), f"{d}: 다른 목록 폴더 안이다"
        assert _ignored(f"{d}/probe"), f"{d}: .gitignore에도 넣는다"


def test_select_everything_without_paths(tmp_path: Path) -> None:
    assert sync.select([], DIRS, tmp_path) == {"logs": (), "records/joint": ()}


def test_select_groups_paths_by_listed_dir(tmp_path: Path) -> None:
    got = sync.select(["records/joint/b", "records\\joint\\a", "logs"], DIRS, tmp_path)
    assert got == {"logs": (), "records/joint": ("a", "b")}


def test_select_whole_dir_wins_over_its_paths(tmp_path: Path) -> None:
    assert sync.select(["logs/x", "logs/"], DIRS, tmp_path) == {"logs": ()}


def test_select_accepts_absolute_path_inside_repo(tmp_path: Path) -> None:
    assert sync.select([str(tmp_path / "logs" / "x")], DIRS, tmp_path) == {"logs": ("x",)}


@pytest.mark.parametrize(
    "raw", ["records", "data/measured.toml", "../logs", "logs/../data", "/logs"]
)
def test_select_rejects_paths_outside_list(tmp_path: Path, raw: str) -> None:
    with pytest.raises(SystemExit):
        sync.select([raw], DIRS, tmp_path)


def test_select_rejects_absolute_path_outside_repo(tmp_path: Path) -> None:
    with pytest.raises(SystemExit):
        sync.select([str(tmp_path.parent / "logs")], DIRS, tmp_path)


needs_rclone = pytest.mark.skipif(
    not sync.rclone_path().exists(), reason="rclone을 아직 받지 않았다 (uv run tools/sync.py login)"
)


@pytest.fixture
def rclone(tmp_path: Path) -> sync.Rclone:
    config = tmp_path / "rclone.conf"
    config.write_text("", encoding="utf-8")
    return sync.Rclone(sync.rclone_path(), config, tmp_path / "cache")


def _write(path: Path, text: str, *, days_ago: float = 0) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    if days_ago:
        stamp = time.time() - days_ago * 86400
        os.utime(path, (stamp, stamp))


def _files(root: Path) -> set[str]:
    return {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file()}


@needs_rclone
def test_push_then_pull_mirrors_listed_dirs_without_deleting(
    rclone: sync.Rclone, tmp_path: Path
) -> None:
    home, office, drive = tmp_path / "home", tmp_path / "office", tmp_path / "drive"
    _write(home / "logs/run1/a.csv", "1")
    _write(home / "records/joint/run1/b.csv", "2")
    _write(home / "build/c.txt", "목록에 없다")
    sync.push(rclone, sync.select([], DIRS, home), root=home, remote_root=drive.as_posix())
    sync.pull(rclone, sync.select([], DIRS, office), root=office, remote_root=drive.as_posix())
    assert _files(office) == {"logs/run1/a.csv", "records/joint/run1/b.csv"}

    (home / "logs/run1/a.csv").unlink()
    sync.push(rclone, sync.select([], DIRS, home), root=home, remote_root=drive.as_posix())
    assert (drive / "logs/run1/a.csv").exists()


@needs_rclone
def test_pull_picks_by_path_and_age(rclone: sync.Rclone, tmp_path: Path) -> None:
    home, office, drive = tmp_path / "home", tmp_path / "office", tmp_path / "drive"
    _write(home / "logs/run[1]/a.csv", "1")  # 글롭 문자도 글자 그대로 고른다
    _write(home / "logs/run2/old.csv", "2", days_ago=10)
    _write(home / "logs/run2/new.csv", "3")
    sync.push(rclone, sync.select([], DIRS, home), root=home, remote_root=drive.as_posix())

    picked = sync.select(["logs/run[1]"], DIRS, office)
    sync.pull(rclone, picked, root=office, remote_root=drive.as_posix())
    assert _files(office) == {"logs/run[1]/a.csv"}

    recent = sync.select(["logs/run2"], DIRS, office)
    sync.pull(rclone, recent, root=office, remote_root=drive.as_posix(), max_age="1d")
    assert _files(office) == {"logs/run[1]/a.csv", "logs/run2/new.csv"}


@needs_rclone
def test_pull_keeps_newer_local_file(rclone: sync.Rclone, tmp_path: Path) -> None:
    home, office, drive = tmp_path / "home", tmp_path / "office", tmp_path / "drive"
    _write(home / "logs/a.csv", "home", days_ago=1)
    sync.push(rclone, sync.select([], DIRS, home), root=home, remote_root=drive.as_posix())
    _write(office / "logs/a.csv", "office")  # 받는 쪽에서 고친 더 새 파일
    sync.pull(rclone, sync.select([], DIRS, office), root=office, remote_root=drive.as_posix())
    assert (office / "logs/a.csv").read_text(encoding="utf-8") == "office"


@needs_rclone
def test_missing_dirs_are_skipped(
    rclone: sync.Rclone, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    drive = (tmp_path / "drive").as_posix()
    selection = sync.select([], DIRS, tmp_path)
    sync.push(rclone, selection, root=tmp_path / "home", remote_root=drive)
    sync.pull(rclone, selection, root=tmp_path / "office", remote_root=drive)
    assert capsys.readouterr().out.count("건너뛴다") == 2 * len(DIRS)


@needs_rclone
def test_remote_settings_reads_saved_login(rclone: sync.Rclone) -> None:
    assert sync.remote_settings(rclone) is None
    rclone.config.write_text(
        f"[{sync.REMOTE}]\ntype = drive\nclient_id = id\nclient_secret = secret\n",
        encoding="utf-8",
    )
    settings = sync.remote_settings(rclone)
    assert settings is not None
    assert (settings["client_id"], settings["client_secret"]) == ("id", "secret")


def test_read_client_takes_desktop_app_file(tmp_path: Path) -> None:
    # 구글 클라우드의 "JSON 다운로드" 파일 모양
    path = tmp_path / "client.json"
    path.write_text(
        '{"installed": {"client_id": "id.apps.googleusercontent.com", "client_secret": "s"}}',
        encoding="utf-8",
    )
    assert sync.read_client(path) == ("id.apps.googleusercontent.com", "s")


@pytest.mark.parametrize("text", ['{"web": {"client_id": "a", "client_secret": "b"}}', "[]", "{"])
def test_read_client_rejects_other_files(tmp_path: Path, text: str) -> None:
    path = tmp_path / "client.json"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(SystemExit):
        sync.read_client(path)
