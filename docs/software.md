# 소프트웨어

결정 기록: 하드웨어 연결 [0005](decisions/0005-connection-architecture.md), 값의 형식과 생성 흐름 [0006](decisions/0006-data-and-pipeline.md), 레포 구조 [0007](decisions/0007-repo-structure.md), 개발 환경과 패키지 [0008](decisions/0008-environment-and-packages.md), 제어 구조 [0009](decisions/0009-control-architecture.md).

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
- 설계 값은 명목값이고 잰 값으로 덮어쓰지 않는다. 출력할 때마다 같은 차이는 부품 형상으로 보정하고, 로봇마다 다른 차이(영점 등)는 로봇 설정에 두고 실물을 움직일 때만 적용하고, 처짐·백래시 같은 움직임 차이는 시뮬레이션 파라미터로 맞춘다. 표는 [0006](decisions/0006-data-and-pipeline.md)에 있다.
- 무게는 출력 전에는 출력 설정(벽 두께, 채움 비율)으로 근사하고, 출력한 뒤에는 저울로 잰 값을 쓴다. 부피에 필라멘트 밀도를 그냥 곱하면 속이 꽉 찬 것으로 계산돼 실제보다 무겁게 나온다. 서보와 베어링 무게를 더해 URDF와 MJCF의 inertial에 넣는다.
- 축은 정확히 직교하거나 평행하게 만든다. 그래서 모델을 만드는 과정의 오차(CAD에서 URDF를 뽑을 때 생기는 오차)는 없어진다. 하지만 출력과 조립 오차는 남으므로 캘리브레이션은 여전히 필요하다. 마스터링 핀으로 영점을 물리적으로 정해서 캘리브레이션할 변수를 줄인다.
- URDF와 MJCF는 같은 로봇 기술에서 따로 작성하고, 둘과 설계 값에서 직접 계산한 FK를 서로 대조하는 테스트를 둔다.
- 부품표: 설계에 필요한 수량은 치수 값에서 자동으로 뽑는다. 주문 상태, 가격, 예산처럼 코드로 알 수 없는 것은 data/의 구매 데이터로 관리하고, 사람이 보는 docs/bom.md는 스크립트가 만든다 ([0002](decisions/0002-docs-system.md)).
- 생성물은 build/에 만들고 git에 넣지 않는다. docs/bom.md만 예외다.

## 레포 구조

```
horilab/
├── pyproject.toml, uv.lock, .python-version   PC 워크스페이스 루트
├── packages/
│   ├── design/    치수 값 → 부품·조립 → 출력 파일·URDF·MJCF·부품표 → 검사
│   ├── motor/     모터 프로세스: 실행 루프, 안전 검사, 서보 드라이버, 로봇 설정, 명령·상태 메시지
│   ├── control/   상위 제어: IK, 궤적, 시뮬레이션 백엔드, 작업 코드
│   └── camera/    영상 메시지 형식, PC 쪽 수신
├── pi/camera-service/   파이 카메라 프로그램 (별도 uv 프로젝트)
├── params/        설계 값 (TOML)
├── data/          실측 값, 로봇 설정·캘리브레이션, 구매 데이터 (TOML)
└── build/         생성물 (git 제외)
```

- 의존 방향: design, motor, camera는 다른 패키지를 불러오지 않는다. control은 motor만 불러온다. 테스트로 확인한다.
- 코드 폴더는 레포 준비 단계(제작 로드맵 0단계)에서 만든다.

## 설계 도구

- **build123d**: Python 코드 CAD다. FreeCAD와 같은 OpenCascade 커널이라 원하는 모서리만 골라 필렛할 수 있고, STEP/STL/3MF로 내보낼 수 있고, 조인트로 조립할 수 있다.
- **OCP CAD Viewer**: VS Code 확장이다. 코드를 실행하면 VS Code 안에 3D로 보여 준다.
- **bd_warehouse**: 나사, 너트, 베어링 같은 표준 부품 라이브러리다. 너트 홈을 내는 구멍도 있다 ([0003](decisions/0003-fastening-nut-traps.md)).
- Rust는 필수가 아니다. 나중에 제어나 펌웨어 쪽에서 쓸 수 있다.

## 시뮬레이션 (MuJoCo)

- MJCF를 같은 치수 값에서 MuJoCo의 MjSpec으로 만든다. 그리퍼는 따로 만들어 플랜지 사이트에 붙인다.
- 충돌 형상은 치수 값에서 만든 기본 도형(상자, 원기둥, 캡슐)이고, 튀어나온 볼트 머리까지 부품을 다 감싸는 크기로 만든다. CAD 부품이 그 안에 들어가는지 테스트로 확인한다. 보이는 모양만 CAD 메시를 쓴다.
- 출력하기 전에 시뮬레이션에서 움직여 보고 작업 범위, IK, 특이점, 충돌을 확인한다. 이전 프로젝트의 손목 문제 같은 걸 만들기 전에 잡으려는 것이다.
- 박스 쌓기도 MuJoCo에서 먼저 돌려 본다. 쌓는 순서와 패턴, 쌓은 박스가 무너지지 않는지를 실물 전에 확인한다.
- 백래시와 처짐은 MuJoCo에 기본으로 나오지 않는다. 실물에서 잰 값(서보 강성, 백래시, 처짐)으로 시뮬레이션 파라미터를 맞춘다.

## 제어 구조

```
[상위 제어: 작업, IK, 궤적]              시뮬레이션이든 실물이든 같은 코드
     │  명령: 궤적, 정지, 토크 켜기·끄기
     │  상태: 관절 위치, 부하, 온도, 전압, 오류
     ▼  (pyzmq. PC 안에서도, 파이로도)
[모터 프로세스: 실행 루프 + 안전 검사]
     ├─ 실물 백엔드: Feetech 서보
     └─ 시뮬레이션 백엔드: MuJoCo (PC에서만)
```

- 상위 제어는 시간이 붙은 관절 궤적을 보내고, 모터 프로세스가 일정한 주기로 실행한다. 상위 제어는 상대가 시뮬레이션인지 실물인지 모른다.
- 관절은 이름으로 부르고, 단위는 rad와 s다. 서보 값과 rad 사이 변환은 실물 백엔드 안에서만 한다.
- 모터 프로세스는 data/의 로봇 설정만 읽는다. 그래서 파이로 옮겨도 그대로 돈다.
- 안전 검사(시작 절차, 한계, 통신 오류, 과부하, 정지)는 모터 프로세스에 있다. 실물을 처음 움직이기 전에 통과할 시험은 [0009](decisions/0009-control-architecture.md)에 있다.
- 프로그램은 기본으로 시뮬레이션만 움직이고, 실물은 명시적으로 켰을 때만 움직인다.

## 하드웨어 연결

```
[PC (Windows)]
 ├─ USB 2 (5 m) ── [Waveshare 버스 서보 어댑터] ── 서보 7개
 └─ 유선 랜 ────── [라즈베리파이 4, 로봇 옆] ── USB 3 ── [D405]
```

- 모터 제어, IK, 경로 계산, 영상 처리는 PC에서 한다. 파이는 D405 영상을 PC로 보내는 일만 하고, 파이가 켜지면 자동으로 실행된다.
- 모터 프로세스는 영상 처리와 다른 프로세스다. PC 부하 때문에 모터 명령 간격이 크게 흔들리면 모터 프로세스를 파이로 옮긴다.

## 개발 환경

- Windows 네이티브에서 한다 (WSL 아님). 서보 어댑터가 COM 포트로 바로 잡히고, OCP CAD Viewer와 MuJoCo 뷰어가 GPU로 바로 그린다. 리눅스가 꼭 필요한 일은 라즈베리파이가 맡는다.
- Python은 3.12로 고정하고 uv로 관리한다. 패키지 버전은 uv.lock으로 고정하고, 올릴 때는 따로 검토한다.
- Windows의 기본 인코딩이 cp949라서 파일은 항상 UTF-8로 읽고 쓰고, `PYTHONUTF8=1`을 켠다.
- conda로만 설치되는 라이브러리는 쓰지 않는다.
- 맥에서도 작업할 수 있게 한다. 설계(build123d, MuJoCo)는 맥에서도 돈다 (MuJoCo 휠은 Apple Silicon 맥용만 있다). 경로는 `pathlib`으로 다룬다.

## 패키지

| 역할 | 패키지 |
|---|---|
| 코드 CAD, 뷰어, 표준 부품 | build123d, ocp_vscode (VS Code 확장과 major.minor를 맞춘다), bd_warehouse |
| 값 파일 | tomllib (읽기), tomlkit (쓰기), pydantic (검사) |
| 시뮬레이션, MJCF | mujoco |
| URDF | xml.etree (작성), yourdfpy (테스트에서 읽기) |
| IK | EAIK (해석적, 가능한 해를 전부 준다), mink (수치, 관절 한계와 충돌을 지키며 이어지는 동작) |
| 서보 | Feetech ftservo-python-sdk |
| 영상 전송 | pyzmq |
| 카메라 (파이) | pyrealsense2. 설치 방법은 파이 준비 때 정한다 |

- 버전과 고른 이유는 [0008](decisions/0008-environment-and-packages.md)에 있다.
- IK 라이브러리는 Windows에서 동작을 확인했다 (2026-11-07, Python 3.12, MuJoCo 3.17.0, 예제 6축 모델).
  - EAIK 1.2.2: 축 배치만 넣으면 구조를 알아서 판별했다 (구형 손목 예제, UR5e 치수의 UR형). 목표 50개를 모두 풀고, 목표마다 가능한 해를 전부(2~8개) 줬다. 한 번에 약 10 µs.
  - mink 1.4.0 (QP 풀이기 daqp 0.11.0): 관절 한계를 지키고, 충돌 회피 제한을 걸면 정한 최소 거리(5 mm)에서 멈췄다. 한 번 풀이에 보통 1 ms 안쪽. 다만 고정된 시작 자세에서 먼 목표 50개 중 12개는 국소해에 걸렸다.
  - 그래서 해석적 IK로 후보 해를 구하고, 그 해를 시작점으로 mink가 관절 한계·충돌·보정 모델까지 반영해 다듬는다.
- React(R3F) 화면은 실물 로봇을 다루는 단계에서 필요해지면 붙인다. 관절 조그, 티칭, 실물 상태를 3D로 보는 디지털 트윈, 카메라 화면 같은 것이다. 그때도 같은 URDF와 메쉬(glTF)를 불러와서, 치수 값 하나에서 나온 결과물을 그대로 쓴다.

## 구현할 때 알아 둘 것

2026-11 조사 기준이다.

- build123d: STEP으로 내보낼 때 한글 라벨이 깨지는 열린 이슈가 있다. 부품 이름은 영어로 쓴다.
- build123d의 3MF 내보내기는 여러 물체와 이름, 색을 담지만 면별 색은 안 된다.
- bd_warehouse 0.4는 build123d 0.12 이상 0.14 미만을 요구한다. build123d를 올릴 때 같이 확인한다.
- OCP CAD Viewer: VS Code의 Python Environments 확장 때문에 뷰어가 멈추면, VS Code 설정에 `"python-envs.terminal.autoActivationType": "shellStartup"`을 넣는다.
- MuJoCo 메시는 바이너리 STL이나 OBJ로 넣는다. STL은 면 20만 개 이하다. 관성은 계산한 값을 명시적으로 넣는다.
- MJCF, URDF, FK가 서로 어긋나는 흔한 원인: mm와 m 혼동, MJCF 각도의 기본 단위가 도(°)인 것, URDF rpy와 MJCF 오일러 순서(`eulerseq="XYZ"`)의 대응, 쿼터니언 순서(MuJoCo는 w가 앞, SciPy는 뒤), 합쳐진 고정 링크(테스트에서 URDF를 MuJoCo로 읽을 때는 `fusestatic`을 끈다), 메시를 출력 방향으로 내보내는 실수, 서보 영점과 방향.
- pyzmq PUB/SUB: 멀티파트 메시지에는 CONFLATE(최신 메시지만 남기기)가 안 된다. 받는 쪽 큐(RCVHWM)를 작게 잡아 지연이 쌓이지 않게 한다.
- D405 타임스탬프는 `RS2_OPTION_GLOBAL_TIME_ENABLED`로 호스트 시간에 맞춘다. 파이와 PC 시계는 chrony로 맞추거나 코드에서 차이를 잰다.

## 참고

- build123d: https://github.com/gumyr/build123d (문서 https://build123d.readthedocs.io)
- bd_warehouse: https://github.com/gumyr/bd_warehouse
- OCP CAD Viewer: https://github.com/bernhard-42/vscode-ocp-cad-viewer
- MuJoCo: https://github.com/google-deepmind/mujoco (문서 https://mujoco.readthedocs.io)
