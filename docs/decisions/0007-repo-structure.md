# 0007. 레포 구조: 패키지 경계와 폴더

- 상태: 채택
- 날짜: 2026-11-07

## 배경

- PC(Windows, x86_64)에서는 설계, 시뮬레이션, 모터 제어, 영상 처리가 돌고, 로봇 옆 라즈베리파이(ARM, Ubuntu 22.04)에서는 카메라 프로그램만 돈다 ([0005](0005-connection-architecture.md)).
- 모터 프로세스는 나중에 파이로 옮길 수 있어야 한다 ([0009](0009-control-architecture.md)). 그러려면 무거운 패키지(build123d, OCP, MuJoCo, IK) 없이 설치돼야 한다.
- 파이의 pyrealsense2는 설치가 특수하다. PyPI의 공식 ARM 휠은 Python 3.9/3.10/3.12용 Jetson 빌드이고, 파이 4에서 되는지는 확인되지 않았다. 이전 프로젝트에서 소스 빌드한 것은 Python 3.11용이다 (2026-11 조사).
- uv 워크스페이스는 잠금 파일 하나를 쓰고 멤버 하나만 골라 설치할 수 있다. requires-python은 모든 멤버의 교집합이고, 플랫폼 전용 로컬 휠을 쓰려면 그 파일을 레포에 커밋해야 한다.
- 레포에 다른 프로젝트를 담을지는 아직 정하지 않았다.

## 검토한 대안

| 대안 | 장점 | 단점 |
|---|---|---|
| A. 패키지 하나 + 역할별 선택 설치(extras) | 가장 단순하다 | 경계가 관례에만 기댄다. 파이에 설치할 때 무거운 의존성을 따로 걸러야 한다 |
| B. PC용 uv 워크스페이스(역할별 패키지) + 파이 카메라 프로그램은 별도 uv 프로젝트 | 패키지마다 의존성이 분명해서 경계가 강제된다. 파이의 Python 버전과 pyrealsense2 설치 방법이 PC 쪽에 영향을 주지 않는다 | pyproject 파일이 여러 개다 |
| C. 파이 카메라까지 한 워크스페이스 | 잠금 파일이 하나다 | 파이 때문에 PC 쪽도 Python 3.11을 지원해야 하거나, ARM용 휠을 레포에 커밋해야 한다 |

## 결정

B로 한다.

```
horilab/
├── README.md, CLAUDE.md, .gitignore
├── pyproject.toml, uv.lock, .python-version   PC 워크스페이스 루트
├── packages/
│   ├── design/    치수 값 → 부품·조립 → 출력 파일·URDF·MJCF·부품표 → 검사
│   ├── motor/     모터 프로세스: 실행 루프, 안전 검사, 서보 드라이버, 로봇 설정, 명령·상태 메시지
│   ├── control/   상위 제어: IK, 궤적, 시뮬레이션 백엔드, 작업 코드
│   └── camera/    영상 메시지 형식, PC 쪽 수신. 의존성이 가벼워 파이에서도 쓴다
├── pi/
│   └── camera-service/   파이 카메라 프로그램. 별도 uv 프로젝트, camera 패키지를 경로로 가져다 쓴다
├── params/        설계 값 (TOML)
├── data/          실측 값, 로봇 설정·캘리브레이션, 구매 데이터 (TOML)
├── docs/          (docs-local/은 git 제외)
└── build/         생성물 (git 제외)
```

- import 이름은 `horilab_design`, `horilab_motor`, `horilab_control`, `horilab_camera`다.
- 의존 방향은 아래와 같고, 테스트로 확인한다.
  - design, motor, camera는 다른 패키지를 불러오지 않는다.
  - control은 motor를 불러온다 (메시지 형식, 실행 루프). design은 불러오지 않고, 생성된 MJCF와 data/의 로봇 설정만 읽는다.
- motor 패키지는 pyserial, 서보 SDK, pyzmq, pydantic처럼 가벼운 의존성만 쓴다. 모터 프로세스를 파이로 옮기면 같은 워크스페이스에서 motor만 설치한다.
- 파이 카메라 프로그램의 Python 버전과 pyrealsense2 설치 방법은 파이 준비 단계에서 실제로 시험해 정한다. 후보는 이전 소스 빌드 재사용, conda-forge의 librealsense, 서드파티 휠(dreamscale-pyrealsense2), 공식 ARM 휠이다. camera 패키지는 그 Python 버전에서도 돌아야 한다.
- 다른 프로젝트가 들어오면 packages/ 아래에 그 프로젝트의 패키지를 더하고, params/와 data/에 프로젝트별 폴더를 만든다.
- 테스트 폴더 위치와 실행 방법은 기반 결정 7번에서 정한다.

## 근거

- 패키지 경계로 의존 방향이 강제돼서, 모터 프로세스를 파이로 옮길 때 설계·시뮬레이션·IK 패키지 없이 설치된다.
- 모터 프로세스를 control 안에 두고 MuJoCo와 IK만 선택 설치로 빼는 방법도 있지만, 그러면 경계가 다시 관례에 기댄다 (A의 단점). 따로 두면 실물 서보로 명령을 보내는 코드가 motor 한 패키지에만 있어서, 안전 검사를 거치지 않는 길이 생기지 않는다 ([0009](0009-control-architecture.md)).
- 파이 카메라 프로그램을 별도 프로젝트로 두면 ARM, Python 3.11, 특수한 pyrealsense2가 PC 쪽 버전 선택을 묶지 않는다. 두 쪽이 많이 다르면 별도 프로젝트로 두라는 게 uv 문서의 권고이기도 하다.
- 메시지 형식은 한 곳에만 두고 보내는 쪽과 받는 쪽이 같은 코드를 쓴다 (영상은 camera, 모터는 motor).
- 값 파일(params/, data/)을 패키지 밖에 두면 코드를 고치지 않고 값을 바꿀 수 있고, 여러 패키지가 같은 파일을 읽는다.

## 영향

- pyproject 파일이 여섯 개다 (루트, design, motor, control, camera, 파이 카메라 프로젝트).
- .gitignore에 `build/`를 더한다.
- 모터 프로세스를 파이로 옮기게 되면, 파이에 uv로 Python 3.12를 설치해 motor가 도는지 그때 확인한다.
