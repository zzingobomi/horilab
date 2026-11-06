# MuJoCo는 형식 정보(.pyi)를 주지 않고, 이름을 컴파일된 모듈에서 별표 import로 가져온다.
# 그래서 pyright가 mujoco의 이름을 하나도 못 보고 오류를 낸다. 모든 이름을 Any로 보게 해서
# 잘못된 오류만 막는다. mujoco를 바르게 쓰는지는 테스트가 확인한다.
from typing import Any

def __getattr__(name: str) -> Any: ...
