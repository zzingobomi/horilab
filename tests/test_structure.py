"""레포 구조 검사: 패키지 경계와 의존 방향 (docs/decisions/0007-repo-structure.md).

uv 워크스페이스에서는 모든 패키지가 한 가상환경에 깔려서, 경계를 어기는 import도
PC에서는 그냥 돈다. 그래서 소스를 읽어 import를 확인한다.
"""

import ast
import importlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ROOT / "packages"

# 패키지마다 불러와도 되는 다른 horilab 패키지
ALLOWED_INTERNAL: dict[str, set[str]] = {
    "horilab_design": set(),
    "horilab_motor": set(),
    "horilab_control": {"horilab_motor"},
    "horilab_camera": set(),
}

# 파이에서도 돌아야 하는 패키지와, 그 패키지가 불러오면 안 되는 무거운 라이브러리
LIGHT_PACKAGES = ("horilab_motor", "horilab_camera")
HEAVY_LIBRARIES = {"build123d", "OCP", "bd_warehouse", "ocp_vscode", "mujoco", "mink", "eaik"}


def _top_level_imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.add(node.module.split(".")[0])
    return names


def _source_files(package: str) -> list[Path]:
    src = PACKAGES / package.removeprefix("horilab_") / "src" / package
    return sorted(src.rglob("*.py"))


def test_every_package_is_listed() -> None:
    found = {p.name for p in PACKAGES.iterdir() if (p / "pyproject.toml").exists()}
    assert {f"horilab_{name}" for name in found} == set(ALLOWED_INTERNAL)


@pytest.mark.parametrize("package", sorted(ALLOWED_INTERNAL))
def test_package_imports(package: str) -> None:
    importlib.import_module(package)


@pytest.mark.parametrize("package", sorted(ALLOWED_INTERNAL))
def test_internal_imports_follow_dependency_direction(package: str) -> None:
    allowed = ALLOWED_INTERNAL[package] | {package}
    for path in _source_files(package):
        internal = {n for n in _top_level_imports(path) if n.startswith("horilab_")}
        assert internal <= allowed, f"{path.relative_to(ROOT)}: {sorted(internal - allowed)}"


@pytest.mark.parametrize("package", LIGHT_PACKAGES)
def test_light_packages_avoid_heavy_libraries(package: str) -> None:
    for path in _source_files(package):
        heavy = _top_level_imports(path) & HEAVY_LIBRARIES
        assert not heavy, f"{path.relative_to(ROOT)}: {sorted(heavy)}"
