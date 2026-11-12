# 0007. 레포 구조: 패키지 경계와 폴더

- 상태: 채택
- 날짜: 2026-11-07

## 배경

- 이 레포는 로봇을 만든다: 부품 설계, 컨트롤러, 펜던트 화면, 시뮬레이션, 응용이 쓰는 클라이언트. 응용(첫 응용은 팔레타이징, 나중에 다른 작업)은 컨트롤러 API 위에 올라가고, API가 자리 잡으면 다른 레포로 옮길 수 있어야 한다 ([0023](0023-robot-controller.md)).
- 컨트롤러는 Rust로 짜서 로봇 옆 컨트롤러 파이(ARM, 리눅스)에서 돌린다. 개발할 때는 같은 컨트롤러가 PC에서도 돈다 (맨 서보 확인, 시뮬레이션). 카메라 파이에서는 카메라 프로그램(Python)이 돈다. PC(Windows, x86_64)에서는 설계, 시뮬레이션, 응용, 영상 처리가 돈다 ([0025](0025-controller-on-pi.md)).
- 파이에 올라가는 것은 무거운 의존성(build123d, OCP, MuJoCo) 없이 만들어져야 한다.
- 파이의 pyrealsense2는 설치가 특수하다. PyPI의 공식 ARM 휠은 Python 3.9/3.10/3.12용 Jetson 빌드이고, 파이 4에서 되는지는 확인되지 않았다. 이전 프로젝트에서 소스 빌드한 것은 Python 3.11용이다 (2026-11 조사).
- uv 워크스페이스는 잠금 파일 하나를 쓰고 멤버 하나만 골라 설치할 수 있다. requires-python은 모든 멤버의 교집합이고, 플랫폼 전용 로컬 휠을 쓰려면 그 파일을 레포에 커밋해야 한다.

## 검토한 대안

### Python 패키지 나누기

| 대안 | 장점 | 단점 |
|---|---|---|
| A. 패키지 하나 + 역할별 선택 설치(extras) | 가장 단순하다 | 경계가 관례에만 기댄다. 파이에 설치할 때 무거운 의존성을 따로 걸러야 한다 |
| B. PC용 uv 워크스페이스(역할별 패키지) + 파이 카메라 프로그램은 별도 uv 프로젝트 | 패키지마다 의존성이 분명해서 경계가 강제된다. 파이의 Python 버전과 pyrealsense2 설치 방법이 PC 쪽에 영향을 주지 않는다 | pyproject 파일이 여러 개다 |
| C. 파이 카메라까지 한 워크스페이스 | 잠금 파일이 하나다 | 파이 때문에 PC 쪽도 Python 3.11을 지원해야 하거나, ARM용 휠을 레포에 커밋해야 한다 |

### 맨 위 나누기와 이름

| 정할 것 | 대안 | 판단 |
|---|---|---|
| 맨 위 나누기 | 도는 기계별 (`pc/`, `pi/`) / 언어 도구별 (`crates/`는 Cargo, `packages/`는 uv, `pendant/`는 웹) | 언어 도구별. 같은 컨트롤러가 PC와 파이에서 돌아서 기계별로는 자리가 둘이 된다. Cargo, uv, 편집기 도구가 레포 뿌리에서 바로 돈다. 파이에만 올리는 설정과 프로그램은 `pi/`에 모은다 |
| 이름 | 처음 구조의 이름 (motor, control) / 하는 일의 이름 | 하는 일. motor와 control은 [0009](0009-control-architecture.md)의 "모터 프로세스"와 "상위 제어"에서 왔는데, 둘은 파이의 컨트롤러 하나가 됐다. motor 패키지에는 서보에 직접 명령하는 기준 구현과 컨트롤러를 거치는 클라이언트라는 반대 성격 둘이 들어가게 됐다 |

## 결정

Python은 B로 하고, 맨 위는 언어 도구별로 나눈다. 이름은 하는 일을 따른다.

```
horilab/
├── README.md, CLAUDE.md, .gitignore
├── Cargo.toml, rust-toolchain.toml   Rust 워크스페이스 (로봇 쪽 프로그램)
├── crates/
│   ├── servo/        서보 버스: 메모리 표, 동기 읽기·쓰기, EEPROM 절차, 가짜 서보 (0024)
│   ├── controller/   컨트롤러: 실행 루프, 안전 검사, 조종권, API 서버(펜던트 화면, WSS), 기록, 서보 준비·맨 서보 확인 명령
│   └── bench/        파이에서 미리 재는 시험 프로그램: 교차 빌드 확인, 실행 루프 부하 시험 (0026)
├── pendant/          펜던트 화면 (웹, R3F). 컨트롤러가 HTTPS로 내보낸다 (0023)
├── pyproject.toml, uv.lock, .python-version   Python 워크스페이스 (PC)
├── packages/
│   ├── design/       치수 값 → 부품·조립 → 출력 파일·URDF·MJCF·부품표 → 검사
│   ├── servo/        서보 버스의 Python 기준 구현, 첫 연결 명령(찾기, 덤프, 통신 시간). Rust 버스가 실물에서 확인되면 지운다
│   ├── client/       컨트롤러 API의 Python 클라이언트. 응용과 PC 도구가 쓴다
│   ├── sim/          시뮬레이션 서버 (MuJoCo). 컨트롤러가 실물 서보 대신 붙는다 (0025)
│   └── camera/       영상 메시지 형식, PC 쪽 수신. 의존성이 가벼워 파이에서도 쓴다
├── apps/             응용. 응용마다 패키지 하나 (첫 응용은 palletizing)
├── pi/
│   ├── controller/   컨트롤러 파이 설정: 처음 설정 스크립트, systemd 유닛, 부팅 인자 (0026)
│   └── camera/       카메라 파이 프로그램. 별도 uv 프로젝트, camera 패키지를 경로로 가져다 쓴다
├── params/           설계 값 (TOML). 응용에 딸린 값은 params/<응용>/
├── data/             실측 값, 로봇 설정·캘리브레이션, 구매 데이터 (TOML)
├── docs/             (docs-local/은 git 제외)
├── tools/            레포 도구 (시험 기록 올리고 받기, 파이 배포). tools/local/은 PC마다 받는 실행 파일과 로그인 설정 (git 제외)
├── tests/            패키지 경계와 레포 도구 검사
├── typings/          타입 정보가 없는 라이브러리의 형식 정보
└── build/, target/   생성물 (git 제외)
```

- 폴더는 첫 코드를 넣을 때 만든다. 지금 있는 것은 servo, bench 크레이트, design, servo, camera 패키지와 params, data, docs, tools, tests, typings다.
- 이름: Python 패키지의 import 이름은 `horilab_<폴더>`, Rust 크레이트 이름은 `horilab-<폴더>`, 응용은 `horilab_<응용>`이다.
- 의존 방향:
  - design, servo, camera, client는 다른 horilab 패키지를 불러오지 않는다.
  - sim은 camera만 불러온다 (흉내 낸 카메라 영상을 같은 형식으로 보낸다). design은 불러오지 않고 생성된 MJCF만 읽는다.
  - 응용은 client와 camera만 불러온다. 로봇 안쪽(design, servo, sim)을 부르지 않아서, 응용을 다른 레포로 옮길 때 폴더만 옮기면 된다. 응용도 패키지와 같은 꼴(`apps/<이름>/src/horilab_<이름>/`)로 둔다.
  - 서보에 직접 닿는 코드(시리얼 포트, 서보 SDK, servo 패키지)는 `crates/servo`와 Python 기준 구현(`packages/servo`)에만 있다. 응용과 클라이언트는 컨트롤러를 거친다.
  - 파이에서도 도는 servo, camera와 다른 레포·환경에서 쓸 client는 무거운 라이브러리를 불러오지 않는다.
  - 따로 설치되는 것(servo, camera, client, 응용)은 불러오는 것을 모두 자기 pyproject에 적는다. 한 환경에 다 깔린 PC에서는 빠진 의존성이 드러나지 않기 때문이다.
  - Rust: controller와 bench는 servo를 부른다. servo는 다른 horilab 크레이트를 부르지 않는다. 시리얼 크레이트는 servo만 쓴다.
- 경계는 문서에만 두지 않고 코드 검사로 막는다.
  - Python: `tests/test_structure.py`가 커밋 전 검사에서 위 규칙을 모두 본다. 규칙의 원본은 그 파일이고, 아직 없는 패키지(client, sim)와 응용의 규칙도 미리 들어 있다. 규칙이 없는 새 패키지는 테스트를 통과하지 못한다. 서보에 직접 닿는 import는 ruff(banned-api)가 편집기에서도 바로 표시한다.
  - Rust: 크레이트는 Cargo.toml에 적은 크레이트만 부를 수 있고, 크레이트 안의 경계(움직이는 명령은 joints만, [0024](0024-servo-bus.md))는 모듈 공개 범위로 컴파일러가 막는다. 크레이트 사이 방향과 시리얼 크레이트 자리는 같은 테스트가 `cargo metadata`로 본다.
- 경로 계획(IK, 궤적)을 어느 언어로 어디에 둘지는 [0025](0025-controller-on-pi.md)의 "팔 움직이기 전에 정할 것"에서 정하고, 그때 자리를 이 기록에 더한다.
- 응용은 컨트롤러 API가 자리 잡을 때까지 이 레포에 둔다. 무거운 의존성(AI 모델 등)이 들어오면 다른 레포로 빼거나 별도 환경으로 둔다 ([0023](0023-robot-controller.md)).
- 응용에 딸린 값(박스 규격, 팔레트 자리)은 `params/<응용>/`에 둔다. 그 출력물(박스, 팔레트 틀)은 design이 같은 값으로 만든다.
- 파이 카메라 프로그램의 Python 버전과 pyrealsense2 설치 방법은 파이 준비 단계에서 실제로 시험해 정한다. 후보는 이전 소스 빌드 재사용, conda-forge의 librealsense, 서드파티 휠(dreamscale-pyrealsense2), 공식 ARM 휠이다. camera 패키지는 그 Python 버전에서도 돌아야 한다.
- 테스트 폴더 위치와 실행 방법은 기반 결정 7번에서 정한다.

## 근거

- 패키지 경계로 의존 방향이 강제돼서, 파이에 올라가는 것은 설계·시뮬레이션 패키지 없이 만들어진다.
- 서보에 직접 명령하는 코드가 servo에만 있으면 안전 검사를 거치지 않는 길이 생기지 않는다 ([0009](0009-control-architecture.md)). 응용이 쓰는 클라이언트를 servo와 따로 두는 것도 그래서다.
- 서보 버스를 컨트롤러와 다른 크레이트로 두면, 서보 버스만 가짜 서보로 시험하고 hori3의 부하 시험 프로그램도 같은 코드를 쓴다 ([0026](0026-pi-runtime-deploy-records.md)).
- 파이 카메라 프로그램을 별도 프로젝트로 두면 ARM, Python 3.11, 특수한 pyrealsense2가 PC 쪽 버전 선택을 묶지 않는다. 두 쪽이 많이 다르면 별도 프로젝트로 두라는 게 uv 문서의 권고이기도 하다.
- 메시지 형식은 한 곳에서 정한다. 영상은 보내는 쪽과 받는 쪽이 camera 패키지를 같이 쓴다. 컨트롤러 API는 Rust 컨트롤러와 Python 클라이언트가 같은 형식을 쓰는지 테스트로 확인한다.
- 값 파일(params/, data/)을 패키지 밖에 두면 코드를 고치지 않고 값을 바꿀 수 있고, 여러 패키지와 컨트롤러가 같은 파일을 읽는다.
- 이름과 자리는 코드가 거의 없을 때 바꾸는 비용이 가장 작다.

## 영향

- pyproject 파일은 루트, 패키지마다 하나, 카메라 파이 프로그램에 하나다. 응용마다 하나씩 는다.
- .gitignore에 `build/`를 더한다. Rust 워크스페이스를 만들 때 `target/`을 더한다.
- 컨트롤러는 PC에서 파이용으로 교차 컴파일해 올린다 ([0026](0026-pi-runtime-deploy-records.md)).

## 바뀐 점

- 2026-11-10: 패키지 밖의 레포 도구를 두는 `tools/`를 더했다. 첫 도구는 시험 기록 올리고 받기다 ([0022](0022-records-and-sync.md)).
- 2026-11-10: 모터 프로세스를 Rust로 짜고 컨트롤러 파이에서 돌리기로 해서([0025](0025-controller-on-pi.md)) Rust 워크스페이스와 `crates/motor`를 더했다. Python motor 패키지는 메시지 쪽과 서보 버스 기준 구현을 맡는다.
- 2026-11-11: [0023](0023-robot-controller.md)과 [0025](0025-controller-on-pi.md)에 맞춰 이름과 자리를 다시 정했다. 처음 이름(motor, control)은 0009의 "모터 프로세스"와 "상위 제어"를 따랐는데, 둘이 파이의 컨트롤러 하나가 돼서다. `crates/motor`는 `crates/controller`와 `crates/servo`로 나누고, `packages/motor`(서보 버스 기준 구현)는 `packages/servo`로 바꿨다. 컨트롤러 API 클라이언트는 `packages/client`로 따로 둔다. 비어 있던 `packages/control`은 지우고, 경로 계획의 자리는 팔 움직이기 전에 정한다. 펜던트 화면(`pendant/`), 시뮬레이션 서버(`packages/sim`), 파이 설정(`pi/controller/`)의 자리를 정했고, `pi/camera-service/`는 `pi/camera/`로 바꿨다. 응용에 딸린 값은 `params/<응용>/`에 둔다.
- 2026-11-11: 경계를 문서에만 두면 구현하다 어길 수 있어서 코드 검사로 막는다. 테스트가 응용과 아직 없는 패키지의 규칙, 서보에 직접 닿는 import, 따로 설치되는 것의 의존성 적기까지 보고, ruff가 서보에 직접 닿는 import를 편집기에서 표시한다. 일부러 어긴 파일을 넣어 검사가 잡는 것을 확인했다.
- 2026-11-11: Rust 워크스페이스를 만들고(servo, bench), 파이에서 미리 재는 시험 프로그램을 둘 `crates/bench`를 더했다. controller 크레이트는 첫 코드를 넣을 때 만든다. 크레이트 사이 방향과 시리얼 크레이트 자리도 테스트로 막는다.
