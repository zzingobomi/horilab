"""pytest 공통 설정. 실물 서보·카메라를 쓰는 테스트(hardware)는 --hardware를 줘야 돈다."""

import pytest


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--hardware",
        action="store_true",
        default=False,
        help="실물 서보·카메라를 쓰는 테스트도 돌린다. 실물이 움직이므로 확인을 받은 뒤에만 쓴다",
    )


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    if config.getoption("--hardware"):
        return
    skip = pytest.mark.skip(reason="실물 테스트는 --hardware를 줘야 돈다")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)
