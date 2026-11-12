# 0008. 개발 환경과 패키지

- 상태: 채택
- 날짜: 2026-11-07

## 배경

- [0001](0001-inherited-decisions.md)은 Windows 네이티브를 골랐다. 그 근거 중 "WSL에서 RealSense가 어렵다"는 D405를 파이에 연결하기로 하면서([0025](0025-controller-on-pi.md)) 더는 해당하지 않아 다시 비교했다.
- 버전 정보는 2026-11 조사 기준이다.

## 개발 환경

| 대안 | 장점 | 단점 |
|---|---|---|
| Windows 네이티브 | 서보 어댑터가 COM 포트로 바로 잡힌다. MuJoCo 뷰어가 GPU로 바로 그린다. 다른 PC에 설치가 간단하다 | 일부 로봇 라이브러리의 Windows 지원이 약하다 (Pinocchio는 conda만). 기본 인코딩이 cp949다 |
| WSL2 | 로봇 라이브러리 대부분이 리눅스를 1순위로 지원한다 | 서보 어댑터를 usbipd로 넘겨야 하고, USB가 가상 네트워크를 거쳐 지연이 늘 수 있다. MuJoCo 뷰어가 WSLg를 거친다. PC마다 설치할 것이 는다 |

결정: Windows 네이티브를 유지한다.

- 파일은 항상 UTF-8로 읽고 쓰고, `PYTHONUTF8=1`을 켠다.
- conda로만 설치되는 라이브러리는 쓰지 않는다.
- 쓰려는 IK 라이브러리(EAIK, mink)는 Windows에서 동작을 확인했다 ([software.md](../software.md#패키지)).
- 리눅스가 꼭 필요한 일은 라즈베리파이가 맡는다.
- 컨트롤러는 파이(리눅스)에서 돌지만 Windows에서 짜고 파이용으로 교차 빌드한다 ([0026](0026-pi-runtime-deploy-records.md)). 리눅스에서만 도는 부분(실시간 우선순위, 게임패드, systemd)은 여분 파이(hori3)에서 돌려 본다. WSL2는 PC의 커널이라 파이의 커널, 실시간 커널, USB를 대신하지 못한다. Windows에서 파이용으로 빌드되지 않는 크레이트가 나오면 WSL보다 GitHub의 arm64 러너를 먼저 본다.

USB 지연과 WSLg 성능은 일반적으로 알려진 내용이고 따로 측정하지 않았다. 결정이 이 두 가지에만 기대지는 않는다.

## Python 버전

| 대안 | 장점 | 단점 |
|---|---|---|
| 3.11 | 파이의 이전 빌드와 같다 | 최신 numpy·scipy는 3.12부터다. 지원이 2027-10에 끝난다 |
| 3.12 | 쓰려는 패키지가 모두 지원한다. 닫힌 해 IK 라이브러리(EAIK, 3.13까지)와도 맞는다 | 특별한 단점 없음 |
| 3.13 | 모두 지원한다 | EAIK 휠의 마지막 버전이라 여유가 적다 |
| 3.14 | build123d가 허용하는 최신이다 | 작은 패키지가 늦게 따라올 수 있다 |

결정: PC는 Python 3.12로 고정한다 (`.python-version`, requires-python `>=3.12,<3.15`). 파이 카메라 프로그램의 버전은 [0007](0007-repo-structure.md)대로 따로 정한다.

## 패키지

| 역할 | 선택 | 검토한 대안 | 근거 |
|---|---|---|---|
| 코드 CAD | build123d 0.13 | | 0001 |
| 표준 부품 | bd_warehouse 0.4 | 직접 모델링 | ISO 렌치볼트·너트·와셔, 너트 홈을 내는 구멍(`captive_nut`, [0003](0003-fastening-nut-traps.md)), 베어링과 베어링 자리가 있다 |
| 뷰어 | ocp_vscode 4.2 + VS Code 확장 4.2 | | 0001. 패키지와 확장의 major.minor를 맞춰야 한다 |
| 값 스키마 | pydantic 2 | dataclass + 직접 검사 | 잘못된 값을 읽는 순간 잡아 준다 |
| TOML 쓰기 | tomlkit | tomli-w | 손으로 단 주석과 순서를 지킨 채 값을 고친다. tomllib로 읽어 tomli-w로 쓰면 주석이 사라진다 |
| 시뮬레이션, MJCF 작성 | mujoco 3.17 | dm_control의 PyMJCF | MjSpec이 공식 방식이고 훨씬 빠르다 |
| URDF 작성 | 표준 라이브러리 xml.etree | urchin, odio_urdf | 의존성이 없다. 다른 라이브러리는 관리가 약하다 |
| URDF 검사 (테스트) | yourdfpy | | URDF를 읽고 FK를 따로 계산해 MJCF와 대조한다 |
| IK | 해석적 IK는 EAIK, 수치 IK는 mink. 골격 설계부터 둘 다 쓴다 | Pinocchio, ikpy, py-opw-kinematics, roboticstoolbox | EAIK는 축 배치에서 구조를 판별해 구형 손목과 UR형을 모두 풀고, 가능한 해를 전부 빠르게 준다. 그래서 손목 후보의 작업 범위 비교와 해 고르기에 쓴다. mink는 생성된 MJCF를 그대로 쓰고 관절 한계와 충돌을 지키며 풀어서, 이어지는 동작(직선 이동, 조그)과 보정 모델에 쓴다. 시작점은 EAIK의 해로 준다. Pinocchio는 Windows에서 conda가 필요하고, ikpy는 느리다. py-opw-kinematics는 구형 손목만 풀고, EAIK 하나로 두 형식을 다 풀어서 roboticstoolbox도 필요 없다 |
| 서보 SDK | 컨트롤러는 Rust로 직접 짠 서보 버스(`crates/servo`, rustypot은 평가해 보고 참고만 했다, [0024](0024-servo-bus.md)). Feetech 공식 ftservo-python-sdk는 Python 기준 구현에만 쓴다 | LeRobot, feetech-servo-sdk | 컨트롤러가 Rust라서다 ([0025](0025-controller-on-pi.md)). 공식 SDK는 동기 읽기·쓰기를 지원하고 pyserial만 필요해 기준 구현에 맞다. LeRobot은 torch 등이 딸려 와 무겁고, feetech-servo-sdk는 2022년 이후 멈췄다 |
| 영상 전송 | pyzmq (PUB/SUB) | Zenoh, gRPC | 파이 한 대에서 PC 한 대로 보내는 단순한 구조라 자동 탐색이 필요 없다 |

- 버전은 uv.lock으로 고정하고, 올릴 때는 따로 검토한 뒤 올린다. MuJoCo 4.0이 곧 나오는데 패키지가 나뉘는 큰 변경이라 특히 그렇다.
- 포맷, 린트, 타입 검사, 테스트 도구는 기반 결정 6~7번에서 정한다.

## 영향

- IK는 골격 설계 때 생성한 모델로 다시 확인하고, EAIK와 mink의 해를 서로 대조하는 테스트를 둔다.
- 서보 통신 속도는 서보 7개를 연결해 잰다. 한 주기(동기 쓰기 + 위치·부하·온도 동기 읽기)에 걸리는 시간과 흔들림을 보고 (조사로는 100~200 Hz 예상), 이 값으로 실행 루프의 주기를 정한다 ([0009](0009-control-architecture.md)). 컨트롤러는 Rust로 짜고 컨트롤러 파이에서 돌리므로, 통신 시간도 파이에서 잰다 ([0025](0025-controller-on-pi.md)).
- 영상 압축과 시계 동기화는 카메라를 구현할 때 정한다. 848×480 30 fps면 깊이 영상은 압축 없이 기가비트 랜에 들어간다 (약 195 Mbit/s).
