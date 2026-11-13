"""pytest 공통 설정. 실물 서보·카메라를 쓰는 테스트(hardware)는 --hardware를 줘야 돈다."""

import pytest


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--hardware",
        action="store_true",
        default=False,
        help="실물 서보·카메라를 쓰는 테스트도 돌린다. 실물이 움직이므로 확인을 받은 뒤에만 쓴다",
    )


def pytest_configure(config: pytest.Config) -> None:
    # 실물 테스트는 한 프로세스에서만 돈다. 여러 프로세스가 같은 포트를 열어 서보를 같이 움직이지 않게 한다
    workers = getattr(config.option, "numprocesses", None)
    if config.getoption("--hardware") and workers not in (None, 0):
        raise pytest.UsageError(
            "--hardware는 -n 0과 같이 준다 (실물 테스트는 한 프로세스에서만 돈다)"
        )


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    if config.getoption("--hardware"):
        return
    skip = pytest.mark.skip(reason="실물 테스트는 --hardware를 줘야 돈다")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)
