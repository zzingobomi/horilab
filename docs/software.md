# 소프트웨어

결정 기록: 값의 형식과 생성 흐름 [0006](decisions/0006-data-and-pipeline.md), 레포 구조 [0007](decisions/0007-repo-structure.md), 개발 환경과 패키지 [0008](decisions/0008-environment-and-packages.md), 제어 구조 [0009](decisions/0009-control-architecture.md), 시험 기록과 데이터 넘기기 [0022](decisions/0022-records-and-sync.md), 로봇 컨트롤러와 응용의 경계 [0023](decisions/0023-robot-controller.md), 서보 버스 [0024](decisions/0024-servo-bus.md), 컨트롤러 자리와 하드웨어 연결 [0025](decisions/0025-controller-on-pi.md), 파이의 실행 환경·배포·기록 [0026](decisions/0026-pi-runtime-deploy-records.md).

## 하나의 치수 값에서 모든 결과물을

```
치수 값 (링크 길이, 서보 종류, 베어링, 손목 구조 …)
   │  build123d 코드
   ├─▶ 부품 STL/3MF → 출력 → 실물 조립
   ├─▶ 조립 화면 (최종 모습 확인, 관절 움직여 보기)
   ├─▶ URDF / MuJoCo MJCF → 시뮬레이션
   ├─▶ 부품표 (베어링, 볼트, 너트 종류와 개수)
   └─▶ 관절별 토크·오차 계산
```

- 값 파일은 TOML이고 네 종류(설계 값, 실측 값, 로봇 설정·캘리브레이션, 구매 데이터)로 나눈다. 파일의 단위는 mm와 도이고 키 이름에 단위를 붙인다. 코드 안에서는 SI(m, rad)다.
- 지금 있는 값 파일은 `params/arm.toml`(팔의 치수, 관절 범위, 관절마다 서보), `params/servos.toml`(서보 명목 사양), `data/measured.toml`(실측 값)이다. `horilab_design.values`가 읽어서 검사하고 로봇 기술(SI)로 바꾼다. 모르는 키가 있으면(단위를 빠뜨린 키 이름 등) 읽을 때 실패한다. 구매 데이터는 `data/purchases.toml`이고, `uv run python -m horilab_design.bom`이 [bom.md](bom.md)를 만든다. 로봇 설정 파일은 필요해질 때 만든다.
- 설계 값은 명목값이고 잰 값으로 덮어쓰지 않는다. 출력할 때마다 같은 차이는 부품 형상으로 보정하고, 로봇마다 다른 차이(영점 등)는 로봇 설정에 두고 실물을 움직일 때만 적용하고, 처짐·백래시 같은 움직임 차이는 시뮬레이션 파라미터로 맞춘다. 표는 [0006](decisions/0006-data-and-pipeline.md)에 있다.
- 무게는 출력 전에는 출력 설정(벽 두께, 채움 비율)으로 근사하고, 출력한 뒤에는 저울로 잰 값을 쓴다. 부피에 필라멘트 밀도를 그냥 곱하면 속이 꽉 찬 것으로 계산돼 실제보다 무겁게 나온다. 서보와 베어링 무게를 더해 URDF와 MJCF의 inertial에 넣는다.
- 축은 정확히 직교하거나 평행하게 만든다. 그래서 모델을 만드는 과정의 오차(CAD에서 URDF를 뽑을 때 생기는 오차)는 없어진다. 하지만 출력과 조립 오차는 남으므로 캘리브레이션은 여전히 필요하다. 마스터링 핀으로 영점을 물리적으로 정해서 캘리브레이션할 변수를 줄인다.
- URDF와 MJCF는 같은 로봇 기술에서 따로 작성하고, 둘과 설계 값에서 직접 계산한 FK를 서로 대조하는 테스트를 둔다.
- 부품표: 설계에 필요한 수량은 치수 값에서 자동으로 뽑는다. 주문 상태, 가격, 예산처럼 코드로 알 수 없는 것은 data/의 구매 데이터로 관리하고, 사람이 보는 docs/bom.md는 스크립트가 만든다 ([0002](decisions/0002-docs-system.md)).
- 생성물은 build/에 만들고 git에 넣지 않는다. docs/bom.md만 예외다.

## 레포 구조

```
horilab/
├── Cargo.toml, rust-toolchain.toml   Rust 워크스페이스
├── crates/
│   ├── servo/       서보 버스 (라이브러리)
│   ├── controller/  컨트롤러: 실행 루프, 안전 검사, 조종권, API 서버(펜던트 화면, WSS), 기록, 서보 준비·관절 시험 명령
│   └── bench/       파이에서 미리 재는 시험 프로그램
├── pendant/         펜던트 화면 (웹, R3F)
├── pyproject.toml, uv.lock, .python-version   Python 워크스페이스
├── packages/
│   ├── design/      치수 값 → 부품·조립 → 출력 파일·URDF·MJCF·부품표 → 검사
│   ├── servo/       서보 버스의 Python 기준 구현, 첫 연결 명령
│   ├── client/      컨트롤러 API의 Python 클라이언트
│   ├── sim/         시뮬레이션 서버 (MuJoCo)
│   └── camera/      영상 메시지 형식, PC 쪽 수신
├── apps/            응용 (팔레타이징 등): 작업 순서, 비전. 응용마다 패키지 하나
├── pi/              파이에 올리는 것: 컨트롤러 파이 설정(controller/), 카메라 파이 프로그램(camera/)
├── params/          설계 값 (TOML). 응용에 딸린 값은 params/<응용>/
├── data/            실측 값, 로봇 설정·캘리브레이션, 구매 데이터 (TOML)
├── tools/           레포 도구 (시험 기록 올리고 받기, 파이 배포)
└── build/, target/  생성물 (git 제외)
```

- 폴더는 첫 코드를 넣을 때 만든다. 지금 있는 패키지는 design, servo, camera다. 자리와 이름을 고른 이유는 [0007](decisions/0007-repo-structure.md)에 있다.
- 의존 방향: design, servo, camera, client는 다른 패키지를 불러오지 않고, sim은 camera만 불러온다. 응용은 client와 camera만 불러온다. 서보에 직접 명령하는 코드는 servo(Rust 크레이트와 Python 기준 구현)뿐이다. 문서에만 두지 않고 테스트(`tests/test_structure.py`)와 ruff로 막는다 ([0007](decisions/0007-repo-structure.md)).
- 응용은 컨트롤러 API가 자리 잡을 때까지 이 레포에 두고, 무거운 의존성(AI 모델 등)이 들어오면 다른 레포로 빼거나 별도 환경으로 둔다 ([0023](decisions/0023-robot-controller.md)).

## 설계 도구

- **build123d**: Python 코드 CAD다. FreeCAD와 같은 OpenCascade 커널이라 원하는 모서리만 골라 필렛할 수 있고, STEP/STL/3MF로 내보낼 수 있고, 조인트로 조립할 수 있다.
- **OCP CAD Viewer**: VS Code 확장이다. 코드를 실행하면 VS Code 안에 3D로 보여 준다.
- **bd_warehouse**: 나사, 너트, 베어링 같은 표준 부품 라이브러리다. 너트 홈을 내는 구멍도 있다 ([0003](decisions/0003-fastening-nut-traps.md)).

## 시뮬레이션 (MuJoCo)

- MJCF를 같은 치수 값에서 MuJoCo의 MjSpec으로 만든다. 그리퍼는 따로 만들어 플랜지 사이트에 붙인다.
- 충돌 형상은 치수 값에서 만든 기본 도형(상자, 원기둥, 캡슐)이고, 튀어나온 볼트 머리까지 부품을 다 감싸는 크기로 만든다. CAD 부품이 그 안에 들어가는지 테스트로 확인한다. 보이는 모양만 CAD 메시를 쓴다.
- 출력하기 전에 시뮬레이션에서 움직여 보고 작업 범위, IK, 특이점, 충돌을 확인한다. 이전 프로젝트의 손목 문제 같은 걸 만들기 전에 잡으려는 것이다.
- 박스 쌓기도 MuJoCo에서 먼저 돌려 본다. 쌓는 순서와 패턴, 쌓은 박스가 무너지지 않는지를 실물 전에 확인한다.
- 백래시와 처짐은 MuJoCo에 기본으로 나오지 않는다. 실물에서 잰 값(서보 강성, 백래시, 처짐)으로 시뮬레이션 파라미터를 맞춘다.

## 제어 구조

```
[응용: 작업 순서, 비전]                                    PC
     │  이동 명령 ↓  ↑ 상태, 이벤트 (컨트롤러 API, WSS)
[컨트롤러 (Rust)]                                          컨트롤러 파이
 ├─ API 서버, 조종권, 펜던트 화면
 ├─ 경로 계획: IK, 블렌드, 시간 매기기 (팔 움직이기 단계에서는 PC에 있다)
 │     │  관절 궤적, 정지, 토크 켜기·끄기 ↓  ↑ 관절 위치, 부하, 온도, 전압, 오류
 └─ 실행 루프 + 안전 검사
       ├─ 실물 백엔드: 서보 버스 → Feetech 서보
       └─ 시뮬레이션 백엔드: MuJoCo 시뮬레이션 서버 (PC)
```

- 응용은 이동 명령을 보내고, 컨트롤러가 IK와 블렌드로 관절 궤적을 만들어 실행 루프가 일정한 주기로 실행한다. 응용은 상대가 시뮬레이션인지 실물인지 모른다. 경계, API, 조종권, 펜던트는 [0023](decisions/0023-robot-controller.md), 실행 루프와 안전은 [0009](decisions/0009-control-architecture.md)에 있다.
- 관절은 이름으로 부르고, 단위는 rad와 s다. 서보 값과 rad 사이 변환은 실물 백엔드 안에서만 한다.
- 컨트롤러는 Rust로 짜고(`crates/controller`, 서보 버스는 `crates/servo`) 로봇 옆 컨트롤러 파이에서 돈다. 개발과 시뮬레이션은 PC에서 같은 프로그램으로 한다 ([0025](decisions/0025-controller-on-pi.md)). 로봇 설정은 data/에서만 읽는다.
- 안전 검사(시작 절차, 한계, 통신 오류, 과부하, 정지)는 실행 루프에 있다. 실물을 처음 움직이기 전에 통과할 시험은 [0009](decisions/0009-control-architecture.md)에 있다.
- 서보 버스는 메모리 표, 서보마다 확인하는 동기 읽기, EEPROM 쓰기 절차를 둔다. 실물이 없는 PC에서는 가짜 서보 버스로 시험한다 ([0024](decisions/0024-servo-bus.md)). 공식 SDK 위에 Python으로 먼저 짠 것(`horilab_servo.feetech`)은 기준 구현이다.
- 프로그램은 기본으로 시뮬레이션만 움직이고, 실물은 명시적으로 켰을 때만 움직인다.

## 하드웨어 연결

```
[PC (Windows)]  개발, 경로 계획, 시뮬레이션, 영상 처리, 응용
 └─ 유선 랜 ─┬─ [컨트롤러 파이, 로봇 옆] ── USB ── [Waveshare 버스 서보 어댑터] ── 서보 7개
             └─ [카메라 파이, 로봇 옆] ── USB 3 ── [D405]
```

- 컨트롤러는 컨트롤러 파이에서 돈다. 그래서 PC가 꺼져 있어도 폰·게임패드로 로봇을 쓸 수 있고, PC 부하가 실행 루프에 닿지 않는다 ([0025](decisions/0025-controller-on-pi.md)). 파이는 로봇을 쓸 때만 켜고, 안 쓸 때는 멀티탭까지 끈다 ([0026](decisions/0026-pi-runtime-deploy-records.md)). 서보와 카메라는 USB 컨트롤러를 나누려고 다른 파이에 꽂는다. 파이는 라즈베리파이 4B다.
- 개발할 때는 어댑터를 PC에 꽂고(5 m USB 2) 같은 컨트롤러를 PC에서 돌려도 된다. 관절 하나 시험이 그렇다.
- 카메라 파이는 D405 영상을 PC로 보내는 일만 하고, 켜지면 자동으로 실행된다.
- 펜던트: 게임패드는 컨트롤러 파이에 꽂고, 안드로이드 폰은 집 와이파이로 컨트롤러의 웹 화면에 들어간다 ([0023](decisions/0023-robot-controller.md)).
- 경로 계획(IK)은 지금은 PC에 있다. PC 없이 쓰는 단계에서 파이로 옮긴다 ([0025](decisions/0025-controller-on-pi.md)).

## 시험 기록과 데이터

[0022](decisions/0022-records-and-sync.md)에서 정했다.

- 실물 실행의 기록은 그것만 보고 실물이 없는 다른 PC에서 분석할 수 있게 남긴다. 분석은 레포 안 스크립트로 한다.
- 데이터는 종류마다 표준 경로에 둔다. git에서 뺀 데이터 폴더는 rclone으로 구글 드라이브에 레포와 같은 경로로 비춘다. 명령 한 번으로 올리고 받는다.
- 올리고 받는 명령은 `tools/sync.py`다 (명령은 [CLAUDE.md](../CLAUDE.md#자주-쓰는-명령), PC 준비는 [setup.md](setup.md#7-시험-기록-올리고-받기)). 동기화 목록은 그 파일의 `SYNC_DIRS`이고 아직 비어 있다. 표준 경로(`records/`, `videos/`)는 [0024](decisions/0024-servo-bus.md)에서 정했고, 기록을 처음 만들 때 목록과 `.gitignore`에 같이 넣는다.

## 개발 환경

- Windows 네이티브에서 한다 (WSL 아님). 서보 어댑터가 COM 포트로 바로 잡히고, OCP CAD Viewer와 MuJoCo 뷰어가 GPU로 바로 그린다. 리눅스가 꼭 필요한 일은 라즈베리파이가 맡는다. 파이에서 도는 컨트롤러도 Windows에서 짜고 파이용으로 빌드하며, 리눅스에서만 도는 부분은 여분 파이(hori3)에서 돌려 본다 ([0008](decisions/0008-environment-and-packages.md)).
- Python은 3.12로 고정하고 uv로 관리한다. 패키지 버전은 uv.lock으로 고정하고, 올릴 때는 따로 검토한다.
- Windows의 기본 인코딩이 cp949라서 파일은 항상 UTF-8로 읽고 쓰고, `PYTHONUTF8=1`을 켠다.
- conda로만 설치되는 라이브러리는 쓰지 않는다.
- 컨트롤러는 Rust로 짠다 ([0025](decisions/0025-controller-on-pi.md)). PC마다 Windows용 Rust를 두고([setup.md](setup.md#8-rust)), 파이용 실행 파일은 PC에서 교차 컴파일한다 ([0026](decisions/0026-pi-runtime-deploy-records.md)).
- 작업하는 PC는 모두 Windows이고 NVIDIA GPU가 있다 ([hardware.md](hardware.md#가진-것)). GPU는 기본 전제다. 도구와 방법을 고를 때 GPU로 할 수 있는 방법을 먼저 따진다.
  - 주로 쓸 곳: 영상 처리(물체 인식, 파지 검출, 포인트 클라우드), 많은 후보를 한꺼번에 돌리는 시뮬레이션.
  - CAD 커널(OpenCascade)과 서보 통신은 CPU 일이고, 라즈베리파이에는 CUDA가 없다.
  - Windows에서 CUDA를 지원하는지 확인한다 (PyTorch는 지원한다). 꼭 필요한 GPU 도구가 리눅스 전용이면, 그 부분만 WSL2로 돌리는 방법과 같이 따져서 정한다.
  - MuJoCo Warp(GPU용 MuJoCo)는 지금은 쓰지 않는다. Windows와 RTX 3060에서 돌지만, 세계 수만 개를 한꺼번에 돌릴 때만 CPU보다 빠르고(약 25배) 세계 하나는 훨씬 느리다 (2026-11-07 측정). 강화학습처럼 대량 병렬 시뮬레이션이 필요해지면 먼저 본다. float32로 계산하므로 써도 탐색용으로만 쓰고, 무언가를 정하는 근거는 CPU MuJoCo(float64)로 확인한다.
- 맥은 지원 대상에 넣지 않는다. 경로를 `pathlib`으로 다뤄 OS에 묶이지 않게만 하고, 맥에서 시험하지는 않는다.

## 패키지

| 역할 | 패키지 |
|---|---|
| 코드 CAD, 뷰어, 표준 부품 | build123d, ocp_vscode (VS Code 확장과 major.minor를 맞춘다), bd_warehouse |
| 값 파일 | tomllib (읽기), tomlkit (쓰기), pydantic (검사) |
| 시뮬레이션, MJCF | mujoco |
| URDF | xml.etree (작성), yourdfpy (테스트에서 읽기) |
| IK | EAIK (해석적, 가능한 해를 전부 준다), mink (수치, 관절 한계와 충돌을 지키며 이어지는 동작) |
| 서보 | 컨트롤러는 Rust (rustypot을 평가해 쓰거나 직접 짠다, [0024](decisions/0024-servo-bus.md)). Feetech ftservo-python-sdk는 Python 기준 구현에만 |
| 영상 전송 | pyzmq |
| 카메라 (파이) | pyrealsense2. 설치 방법은 파이 준비 때 정한다 |

- 버전과 고른 이유는 [0008](decisions/0008-environment-and-packages.md)에 있다.
- IK 라이브러리는 Windows에서 동작을 확인했다 (2026-11-07, Python 3.12, MuJoCo 3.17.0, 예제 6축 모델).
  - EAIK 1.2.2: 축 배치만 넣으면 구조를 알아서 판별했다 (구형 손목 예제, UR5e 치수의 UR형). 목표 50개를 모두 풀고, 목표마다 가능한 해를 전부(2~8개) 줬다. 한 번에 약 10 µs.
  - mink 1.4.0 (QP 풀이기 daqp 0.11.0): 관절 한계를 지키고, 충돌 회피 제한을 걸면 정한 최소 거리(5 mm)에서 멈췄다. 한 번 풀이에 보통 1 ms 안쪽. 다만 고정된 시작 자세에서 먼 목표 50개 중 12개는 국소해에 걸렸다.
  - 그래서 해석적 IK로 후보 해를 구하고, 그 해를 시작점으로 mink가 관절 한계·충돌·보정 모델까지 반영해 다듬는다.
- React(R3F) 화면은 펜던트 화면으로, 컨트롤러가 HTTPS로 준다 ([0023](decisions/0023-robot-controller.md)). 관절 조그, 티칭, 실물 상태를 3D로 보는 디지털 트윈, 카메라 화면 같은 것이다. 같은 URDF와 메쉬(glTF)를 불러와서, 치수 값 하나에서 나온 결과물을 그대로 쓴다.

## 구현할 때 알아 둘 것

2026-11 조사 기준이다.

- build123d: STEP으로 내보낼 때 한글 라벨이 깨지는 열린 이슈가 있다. 부품 이름은 영어로 쓴다.
- build123d의 3MF 내보내기는 여러 물체와 이름, 색을 담지만 면별 색은 안 된다.
- bd_warehouse 0.4는 build123d 0.12 이상 0.14 미만을 요구한다. build123d를 올릴 때 같이 확인한다.
- OCP CAD Viewer: VS Code의 Python Environments 확장 때문에 뷰어가 멈추면, VS Code 설정에 `"python-envs.terminal.autoActivationType": "shellStartup"`을 넣는다.
- MuJoCo 메시는 바이너리 STL이나 OBJ로 넣는다. STL은 면 20만 개 이하다. 관성은 계산한 값을 명시적으로 넣는다.
- MuJoCo가 MJCF를 XML로 쓸 때 숫자를 유효숫자 6자리로 줄인다 (0.0123456789 → 0.0123457, 2026-11-07 확인). 위치 1 µm, 각도 1 µrad 안쪽이라 쓰는 데는 문제없지만, 다시 읽은 모델과 비교할 때는 그만큼 허용 오차를 둔다.
- MJCF, URDF, FK가 서로 어긋나는 흔한 원인: mm와 m 혼동, MJCF 각도의 기본 단위가 도(°)인 것, URDF rpy와 MJCF 오일러 순서(`eulerseq="XYZ"`)의 대응, 쿼터니언 순서(MuJoCo는 w가 앞, SciPy는 뒤), 합쳐진 고정 링크(테스트에서 URDF를 MuJoCo로 읽을 때는 `fusestatic`을 끈다), 메시를 출력 방향으로 내보내는 실수, 서보 영점과 방향.
- pyzmq PUB/SUB: 멀티파트 메시지에는 CONFLATE(최신 메시지만 남기기)가 안 된다. 받는 쪽 큐(RCVHWM)를 작게 잡아 지연이 쌓이지 않게 한다.
- D405 타임스탬프는 `RS2_OPTION_GLOBAL_TIME_ENABLED`로 호스트 시간에 맞춘다. 파이와 PC 시계는 chrony로 맞추거나 코드에서 차이를 잰다.

## 참고

- build123d: https://github.com/gumyr/build123d (문서 https://build123d.readthedocs.io)
- bd_warehouse: https://github.com/gumyr/bd_warehouse
- OCP CAD Viewer: https://github.com/bernhard-42/vscode-ocp-cad-viewer
- MuJoCo: https://github.com/google-deepmind/mujoco (문서 https://mujoco.readthedocs.io)
