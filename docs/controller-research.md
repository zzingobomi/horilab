# 로봇 컨트롤러 리서치

[0023](decisions/0023-robot-controller.md)의 리서치 계획 가운데 어디서 해도 결과가 같은 네 주제를 2026-11-10에 조사했다: 산업용 컨트롤러, 데스크톱·오픈소스 팔, 전송 방식, 시뮬레이션 연결과 파이용 계획 라이브러리. 파이 실행 환경(실시간, 배포, 자동 시작)과 NAS는 집에서 실제 장비 정보를 확인한 뒤 조사한다 ([0025](decisions/0025-controller-on-pi.md)).

- 이 문서는 조사한 사실이다. 정하는 것은 0023과 0025에 있다.
- 사실마다 출처를 붙였다. 출처를 직접 확인하지 못한 것은 "(확인 안 됨)", 프로젝트나 업체가 스스로 밝힌 것은 "(주장)"으로 적었다.
- 남의 결론은 그대로 믿지 않는다. 우리 결정에 쓰기 전에 계산이나 실측으로 다시 본다.

## 1. 산업용 컨트롤러

### 컨트롤러 안과 밖

- UR: 제어 상자 안의 PC에서 컨트롤러(URControl)가 부팅 때 뜨는 서비스로 돈다. 펜던트 화면(PolyScope)도 로컬 TCP로 붙는 손님이다 ([URControl](https://www.universal-robots.com/manuals/EN/HTML/SW5_26/Content/prod-scriptmanual/all_scripts/Connecting_to_URControl.htm)). IK, 블렌드, 한계, 안전은 컨트롤러가 한다 ([movej](https://www.universal-robots.com/manuals/EN/HTML/SW5_26/Content/prod-scriptmanual/all_scripts/movej_qa14v105t0r.htm)).
- UR에서 PC가 하는 일: 원격 모드에서 프로그램 보내기, 실행 중인 프로그램에 문장 넣기(Interpreter mode), RTDE로 입출력과 상태 주고받기, 대시보드 서버로 감독하기 ([원격 제어](https://www.universal-robots.com/articles/ur/remote-control-via-tcpip/)).
- ABB: RAPID 프로그램과 경로 계획은 컨트롤러에서 돌고, PC는 Robot Web Services(REST)로 감독하고 데이터를 읽고 쓴다 ([RWS](https://developercenter.robotstudio.com/api/rwsApi/msh_request_page.html)).
- FANUC: PC에서 움직이는 길은 둘이다. 8칸 버퍼에 J/L/C/S 명령을 넣는 RMI와, 서보 수준 명령을 흘려보내는 Stream Motion이다 ([사용법](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/controller_usage.html)).
- ros-controls는 업체 공통 이동 명령(PTP/LIN/CIRC + 블렌드 반지름)을 로봇 자체 계획기에 넘기는 메시지를 더했다 ([MotionPrimitive](https://raw.githubusercontent.com/ros-controls/control_msgs/master/control_msgs/msg/MotionPrimitive.msg)).

### 이동 명령과 블렌드

| | 명령과 값 | 블렌드를 맞출 수 없을 때 |
|---|---|---|
| UR | `movej`(관절 공간에서 곧게, 목표가 자세면 IK), `movel`(공구 공간에서 곧게), `movep`(공구 속도 일정, 원호 블렌드), `movec`. 가속 a, 속도 v, 시간 t(주면 a·v보다 우선), 블렌드 반지름 r | 모션 버전 1은 블렌드가 겹치면 그 이동을 건너뛰고 경고한다. 권장하는 버전 2는 반지름을 줄여 건너뛰지 않는다 ([모션 버전](https://www.universal-robots.com/manuals/EN/HTML/SW5_26/Content/prod-scriptmanual/all_scripts/motion_version_desc.htm)). 반지름은 웨이포인트 간격의 절반 미만을 권한다 |
| ABB | MoveJ/MoveL/MoveC, 속도(speeddata), 구역(zonedata: `fine`은 정지점, `z10`은 반지름 10 mm) ([zonedata](https://developercenter.robotstudio.com/api/fpsdk/html/fb6df07f-0116-77ed-cd7f-7a5aab872b19.htm)) | 로봇이 구역에 들어설 때 다음 이동이 준비되지 않았으면 50024 Corner path failure를 내고 그 점에서 멈춘다 ([커뮤니티 글](https://tech-community.robotics.abb.com/t/50024-cornerpathfailure/4966)) |
| FANUC | J/L/C/S, FINE / CNT0~100 | (확인 안 됨) |
| KUKA | PTP/LIN/CIRC, C_PTP/C_DIS 등 | 미리 읽어 두는 실행(advance run)이 있어야 블렌드한다 (확인 안 됨) |

### 상태 흐름과 스트리밍

- UR RTDE: 클라이언트가 1~500 Hz 가운데 고르면 서버가 솎아 보낸다(e-Series). 한 입력 변수는 한 클라이언트만 쓴다. 밀리면 묵은 패킷을 버리고 최신을 보낸다 ([RTDE](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html)).
- UR 외부 제어: 매 메시지에 다음 메시지를 기다릴 시간이 들어 있다(기본 200 ms). 연결이 끊기면 외부 제어 프로그램이 멈춘다 ([reverse interface](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/kilted/doc/ur_client_library/doc/architecture/reverse_interface.html), [동작 모드](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/operation_modes.html)).
- ABB EGM은 약 4 ms(타사 자료), KUKA RSI는 4 ms다 ([RSI](https://www.kuka.com/en-de/products/robot-systems/software/application-software/kuka_robotsensorinterface)). FANUC Stream Motion은 PC가 바빠 보낼 때를 놓치면 로봇이 고장 상태가 된다 ([문제 해결](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/troubleshooting/troubleshooting.html)).
- 업체들의 공통 지침: 센서가 고리 안에 있을 때만 흘려보내고, 정해진 경로는 이동 명령으로 보낸다.

### 정지와 복구

- UR 정지 범주: 비상 정지는 Cat 1(제어해 멈춘 뒤 전원 끔), 보호 정지는 Cat 2(전원을 유지한 채 정지 감시), 고장과 한계 위반은 Cat 0(바로 전원 끔)이다 ([안전 FAQ](https://www.universal-robots.com/articles/ur/safety-faq)). 전원을 꺼도 브레이크가 팔을 잡는다는 전제다.
- UR 복구: 보호 정지는 팝업 닫기 → 잠금 풀기 → 재생이고 멈춘 곳부터 잇는다. 비상 정지는 브레이크 풀기 → 재생이고 처음부터 한다 ([복구](https://docs.universal-robots.com/tutorials/controlling-robot-externally/stop-recovery.html)).

### 모드와 조종권

- UR: 로컬 모드에서는 펜던트만 조종하고, 원격 스크립트 포트는 연결을 끊는다. 원격 모드에서는 펜던트가 잠겨 움직임을 시작할 수 없다. 상태 보기는 두 모드 모두 열려 있다. 수동 모드는 3단 인에이블 스위치를 누르는 동안만 움직이고 공구·팔꿈치 속도가 250 mm/s 이하다 ([3단 스위치](https://www.universal-robots.com/manuals/EN/HTML/SW10_15/Content/prod-usr-man/complianceUR15/PolyX_SW/polyx-Safety-Three-Position.htm)).
- ABB: 컨트롤러 자원은 한 번에 한 클라이언트만 쥔다 ([마스터십](https://developercenter.robotstudio.com/api/pcsdk/articles/Manual/Run-time-environment/Running-PC-SDK-Applications/Mastership.html)). RobotWare 8의 "제어 스테이션"에서는 로컬 펜던트가 모드와 상관없이 언제든 쓰기 권한을 가져간다. 원격 스테이션은 펜던트에서 외부 제어를 허용받은 뒤 권한을 요청하고, 다 쓰면 돌려준다 ([제어 스테이션](https://developercenter.robotstudio.com/api/pcsdk/articles/Manual/Run-time-environment/Running-PC-SDK-Applications/Control-Station-Write-Access.html)). 모드는 자동 ↔ 수동 ↔ 수동 고속 순으로만 바뀌고, 바뀔 때 모터가 꺼진다.
- FANUC: 경보가 나면 움직임 권한이 컨트롤러로 돌아가고, 외부 드라이버가 다시 받아야 한다 ([권한](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/motion_control_authority.html)).

### 상태와 이벤트

- UR은 세 상태를 따로 둔다: robot mode(POWER_OFF, IDLE, RUNNING …), safety status(NORMAL, PROTECTIVE_STOP, FAULT …), program state(STOPPED, PLAYING, PAUSED). 오류 코드는 `C171A4`처럼 코드와 부코드다. 안전·오류 메시지는 상태 흐름과 따로 밀어 준다 ([대시보드](https://www.universal-robots.com/manuals/EN/HTML/SW5_23/Content/prod-dashboard/Dashboard_table.htm)).
- UR 제어 API의 반환 값: 0 성공, -1 원격 조종 상태가 아님, -2 로봇 상태가 맞지 않음, -100 그 밖 ([Control API](https://www.universal-robots.com/manuals/EN/HTML/SW10_15/Content/prod-scriptmanual/all_scripts/Module_Control_API_control_methods.htm)).

### 시뮬레이터

- URSim, ABB 가상 컨트롤러, KUKA OfficeLite 모두 같은 컨트롤러 프로그램을 따로 하나 더 띄우고, 응용은 같은 인터페이스로 붙는다 ([URSim](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/ursim_docker.html), [ABB 가상](https://developercenter.robotstudio.com/api/pcsdk/articles/Manual/Installation-and-development-environment/Two-development-models---virtual-and-real.html), [OfficeLite](https://www.kuka.com/products/robotics-systems/software/simulation-planning-optimization/kuka_officelite)). 실행 중인 컨트롤러 안의 스위치가 아니다. URSim은 비상 정지, 입력 I/O, 충돌 감지, 힘 모드를 못 한다.

### 조그와 티칭

- UR: 화살표를 누르는 동안 기준 좌표계(베이스, 공구, 사용자)로 움직이고, 떼면 멈춘다. 사용자 좌표계는 컨트롤러에 저장한다 ([조그](https://www.universal-robots.com/articles/ur/programming/how-to-jog-the-robot/)).
- ABB: 스텝 크기를 소·중·대·사용자 중에 고르고, 1초 넘게 누르면 초당 10스텝으로 반복한다.

## 2. 데스크톱·오픈소스 팔

| | 실행 루프 (어디, 언어, 주기) | 움직임 API | 조그와 화면 | 호스트나 클라이언트가 멈추면 | 같은 API 뒤의 시뮬레이터 |
|---|---|---|---|---|---|
| [PAROL6](https://github.com/PCrnjak/PAROL6-python-API) | PC의 Python, 100 Hz. STM32는 스텝 펄스만 | UDP + msgpack. 블렌드하는 이동(TOPP-RA)과 조그·서보 흐름(Ruckig) | Tk 앱, 커뮤니티 웹 화면 | 흐름은 0.25 s 뒤 감속해 버팀 (PC 쪽, 주장). 펌웨어는 마지막 속도로 계속 돈다 ([main.cpp](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm/blob/main/PAROL6%20control%20board%20main%20software/src/main.cpp)) | 있다: 가짜 시리얼 + 별도 시뮬레이션 프로세스 |
| xArm, Lite 6 | 제어 상자 (자세히 공개 안 됨). 서보 흐름 250 Hz까지 | TCP SDK. 반지름 블렌드 이동, 서보 흐름, 속도, 지금 상태에서 다시 계획(모드 6, 7) ([모드](https://docs.ufactory.cc/user_manual/ufactoryStudio/10.motion_mode_and_state.html)) | 브라우저 화면(설치 없음). 게임패드는 ROS 2로만 | 공개 안 됨. 속도 명령의 자동 정지는 고를 때만 | 있다: 실제 펌웨어를 Docker로 ([Docker](https://docs.supportarticle.ufactory.cc/support_articles/software/how-to-install-ufactory-studio-in-docker.html)) |
| AR4 | Teensy가 스텝, PC는 Python(공식) 또는 ros2_control(커뮤니티) | 시리얼 ASCII, 관절 위치·속도 | Tk 앱, Xbox (옛 문서) | 커뮤니티 펌웨어는 속도 명령이 계속 돈다 ([드라이버](https://github.com/ycheng517/ar4_ros_driver)) | ROS 드라이버로 Gazebo |
| Reachy 2 | 로봇 안의 Rust EtherCAT 1 kHz, 위는 ROS 2 | gRPC + Python SDK | (보지 않음) | 보드는 100 ms, 마스터는 500 ms 안에 응답이 없으면 멈춤. 5 ms보다 묵은 명령은 버림 ([안전](https://pollen-robotics.github.io/poulpe_ethercat_controller/safety_features)) | Docker + Gazebo |
| Reachy Mini | Pi CM4 또는 PC의 Python 50 Hz + Rust 모터 라이브러리 | REST + WebSocket + WebRTC ([개념](https://huggingface.co/docs/reachy_mini/main/SDK/core-concept)) | 웹·JS 앱 | 끊겨도 아무것도 하지 않음 (버팀) | 있다: MuJoCo 또는 물리 없는 모드 ([시뮬레이션](https://huggingface.co/docs/reachy_mini/platforms/simulation/get_started)) |
| LeRobot | 사용자 Python 프로세스 안 (LeKiwi는 파이 호스트 30 Hz) | 같은 프로세스. LeKiwi는 pyzmq | 폰(데드맨), 게임패드 | LeKiwi는 500 ms 뒤 바퀴만 멈춤 ([host](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/lekiwi/lekiwi_host.py)) | 찾지 못함 |
| ros2_control | C++ 실시간 스레드 100 Hz ([manager](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html)) | 궤적 토픽·액션 | 없음 | 궤적 제어기는 버팀. 시간 제한은 기본 꺼짐 ([JTC](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)) | 가짜 또는 Gazebo 플러그인 |
| MoveIt Servo | C++ 노드 100 Hz | 관절 조그, twist, 자세 흐름 | 키보드 (게임패드는 붙이는 쪽에서) | 0.1 s보다 묵으면 부드럽게 멈춤 ([튜토리얼](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)) | ros2_control로 |

- 경계: 로봇 옆에 컨트롤러를 둔 곳(xArm, Reachy 둘, PAROL6 서버)은 모두 IK, 보간, 블렌드, 한계를 컨트롤러 안에 둔다. Reachy Mini는 로봇 모델(URDF)도 컨트롤러가 내준다.
- 움직임 종류는 셋으로 나뉜다: 줄 세워 블렌드하는 이동, 최신 목표만 따르는 흐름(xArm 서보 모드, LeKiwi, PAROL6), 지금 상태에서 다시 계획하는 이동(xArm 모드 6, 7).
- 조그: 컨트롤러 안의 흐름으로 두고, Ruckig 같은 것으로 부드럽게 하고, 0.1~0.25 s 안에 새 명령이 없으면 부드럽게 멈추고, 특이점과 한계 근처에서 느려진다 (PAROL6, MoveIt Servo).
- LeRobot의 폰 조작은 누르는 동안만 움직이고 놓으면 멈추며, 누를 때마다 기준을 다시 잡아 튀지 않게 한다. 안드로이드 페이지는 HTTPS가 있어야 한다 ([폰 조작](https://huggingface.co/docs/lerobot/phone_teleop)).
- 감시는 모터를 실제로 멈출 수 있는 가장 낮은 층에 겹겹이 둔다 (Reachy 2). PAROL6과 커뮤니티 AR4 펌웨어는 호스트가 죽어도 마지막 속도로 계속 돈다. 피할 예다.
- Reachy Mini의 조종권 잠금은 SDK 클라이언트가 그냥 지나간다 (코드 주석). 권고일 뿐 막지 못한다.
- 시뮬레이터는 가장 낮은 층에서 바꿔 끼운다: xArm은 실제 펌웨어를 Docker로, PAROL6은 가짜 시리얼 + 시뮬레이션 프로세스, Reachy Mini는 백엔드 교체, ros2_control은 플러그인 교체.
- 배포: Reachy Mini는 systemd 서비스와 데몬의 업데이트 API, Reachy 2는 Docker, xArm은 업체 패키지와 재부팅, LeKiwi는 SSH로 손으로 한다.
- 파이 성능: Reachy Mini 무선판은 CM4(파이 4와 같은 칩)에서 Python 데몬, Rust 모터 라이브러리, IK, 웹·WebRTC 서버를 50 Hz로 돌린다 (주장).

### Feetech 서보를 다루는 코드

- [rustypot](https://github.com/pollen-robotics/rustypot) (Pollen, Apache-2.0): Rust로 짠 서보 라이브러리다.
  - PyO3로 Python에 붙고, 시리얼 통신 동안 GIL을 놓는다.
  - Feetech 모델: STS3215, STS3250, SM8512BL, SCS0009, SCS0043 (README에는 STS3235도). ST-3120은 목록에 없다.
  - 동기 읽기·쓰기, 상태 바이트, 재시도, 타임아웃, EEPROM 잠금을 지키는 토크 제어, 한 버스에 섞인 모델을 다룬다.
  - 0.1.0(2023-01)부터 1.13.1(2026-10-14)까지 27번 냈다. Reachy Mini와 LeRobot이 쓴다.
- LeRobot은 v0.7.0(2026-11-05)에서 공식 SDK 대신 rustypot을 쓰기 시작했다 ([발표](https://github.com/huggingface/lerobot/releases)). 아래는 LeRobot 코드에 적힌 그들의 구현이다 ([motors_bus.py](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/motors_bus.py)). 서보가 정말 그런지는 우리가 확인한다.
  - 동기 읽기는 묶어 읽지만 한순간의 스냅샷이 아니다. 동기 쓰기는 응답이 없어서 잃은 패킷을 모른다.
  - 타임아웃은 USB 어댑터의 지연 타이머(16 ms)보다 길어야 한다.
  - 보정은 범위 가운데가 2047로 읽히게 위치 오프셋을 쓴다. 오프셋이 부호-크기 표현이라 잘려서 한 칸 어긋날 수 있다.
  - Return_Delay_Time을 0으로 둔다 (공장값 500 µs). Waveshare 메모리 표에는 그 자리(7번)가 "예약"으로 적혀 있다.
  - STS3215의 Phase 비트 4를 지워 위치를 범위 안에 둔다.
  - v0.7.0 전에는 PyPI SDK의 패킷 타임아웃 버그를 실행 중에 고쳐 썼다.
- [cu-feetech](https://docs.rs/cu-feetech) (Copper 프로젝트, Apache-2.0, 1.3.0): 같은 STS/SCS 서보용 Rust 드라이버다.

## 3. 전송 방식

| | Zenoh | ZeroMQ | 컨트롤러가 여는 WebSocket + HTTP | gRPC | MQTT |
|---|---|---|---|---|---|
| 최신 (2026-11) | 1.10.2 ([발표](https://github.com/eclipse-zenoh/zenoh/releases)) | libzmq 4.3.5 (2023-10), pyzmq 27.3.0 | axum 0.8.9, tokio-tungstenite 0.30, websockets 17.4 | tonic 0.14.9, grpcio 1.85.1 | Mosquitto 2.1.2, paho 2.2.0, rumqttc 0.25.1 |
| Rust | 본체가 Rust | libzmq 바인딩은 2022년이 마지막, 순수 Rust판은 기능이 다 없다 ([zmq.rs](https://github.com/zeromq/zmq.rs)) | 그대로 | 그대로 | 클라이언트 |
| 브라우저 | zenoh-ts + remote-api 플러그인. REST + SSE는 TLS가 없다 | 중계기가 필요하다 | 그대로 | 단방향과 서버 흐름만 ([tonic-web](https://docs.rs/tonic-web/latest/tonic_web/)) | MQTT.js over WSS |
| 파이에 더 띄울 것 | Rust↔Python은 없음, 브라우저용 zenohd나 브리지 | 중계기 | 없음 | 없음 | 브로커 |
| 연결 확인 | liveliness 토큰, 임대 10 s ([기본 설정](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5)) | ZMTP 하트비트(ms) | 서버 ping뿐 | 1분 이상 권장 ([keepalive](https://grpc.io/docs/guides/keepalive/)) | 초 단위 × 1.5, 유언 메시지 |
| 기록 | 저장소는 최신 값만 | 프록시 복사 소켓 | 컨트롤러 안에서 | 컨트롤러 안에서 | `#` 구독 |

- 부하는 결정 요인이 아니다. 초당 100번 × 수백 바이트면 초당 수십 kB이고, 모두 넉넉하다 (추정).
- 데드맨은 어느 전송 방식의 연결 확인에도 맡길 수 없다. 가장 짧은 것도 초 단위다 (Zenoh 10 s, Python websockets ping 20 s). 브라우저는 WebSocket ping에 네트워크 층에서 답하고, JavaScript는 ping을 보낼 수 없다 ([RFC 6455](https://www.rfc-editor.org/rfc/rfc6455#section-5.5.2), [websockets](https://websockets.readthedocs.io/en/stable/topics/keepalive.html)). 그래서 누르는 동안 정해진 간격으로 보내는 응용 메시지를 컨트롤러가 직접 재야 한다.
- 브라우저 규칙:
  - 화면 켜 두기(Wake Lock)와 Gamepad API는 HTTPS 같은 보안 컨텍스트에서만 된다 ([MDN](https://developer.mozilla.org/en-US/docs/Web/Security/Secure_Contexts/features_restricted_to_secure_contexts)). Wake Lock은 페이지가 숨으면 풀려서 다시 요청해야 한다.
  - HTTPS 페이지에서는 WSS와 HTTPS만 쓸 수 있다 ([혼합 콘텐츠](https://developer.mozilla.org/en-US/docs/Web/Security/Mixed_content)). 카메라 화면도 HTTPS여야 한다.
  - Chrome은 공개 주소에서 사설 주소로 가는 요청에 권한을 묻는다 ([로컬 네트워크 접근](https://developer.chrome.com/docs/modern-web-guidance/guides/security/local-network-access)). 화면을 파이가 직접 주면 경계를 넘지 않는다 (추론).
  - 숨은 페이지의 타이머는 1초 단위로 느려져서 데드맨이 걸린다 ([타이머](https://developer.chrome.com/blog/timer-throttling-in-chrome-88)). 안전한 쪽이다.
- 집 랜에서 HTTPS를 쓰는 길:
  1. 사설 인증 기관(mkcert)을 만들고 기기마다 뿌리 인증서를 설치한다 ([mkcert](https://github.com/FiloSottile/mkcert)). iOS는 프로필을 설치한 뒤 완전 신뢰를 켠다. 인증서에는 DNS 이름이 있어야 하고 유효 기간은 825일 이하다 ([애플](https://support.apple.com/en-us/103769)). 안드로이드 Chrome이 사용자가 넣은 인증서를 믿는지는 확인하지 못했다. 폰에서 시험한다.
  2. 도메인을 사고 Let's Encrypt의 DNS-01로 인증서를 받는다 ([방식](https://letsencrypt.org/docs/challenge-types/)). 기기 설정이 없지만 도메인 비용과 갱신 자동화가 든다. 공유기의 DNS 재바인딩 보호가 막을 수 있다 (확인 안 됨).
  3. 개발할 때만 Chrome 플래그로 HTTP를 보안 출처로 취급한다. iOS에는 없다.
- 기록과 재생: MCAP(MIT)는 Rust·Python 라이브러리와 명령줄 도구가 있다 ([mcap](https://github.com/foxglove/mcap)).
- Reachy Mini는 v1.0.0에서 Zenoh를 썼다가 지금은 websockets와 zeroconf를 쓴다. 이유는 찾지 못했다 ([pyproject](https://github.com/pollen-robotics/reachy_mini/blob/main/pyproject.toml)).
- 게임패드를 파이에 꽂으면 Rust의 gilrs(리눅스 evdev, 핫플러그)로 컨트롤러 프로세스가 직접 읽을 수 있다 ([gilrs](https://docs.rs/gilrs/latest/gilrs/)). 그러면 데드맨이 와이파이를 건너지 않는다.

### 조사가 추천한 설계

- A (지금): 컨트롤러가 유일한 서버다. 파이의 Rust 프로세스 하나가 펜던트 화면(HTTPS)과 WSS API 하나를 연다. 브라우저, Python 응용, 게임패드가 모두 거기에 붙는다.
  - 메시지는 `{type, id, seq, t, body}` 모양의 JSON이다. 명령은 id로 답하고, 이벤트와 상태는 밀어 준다. 클라이언트마다 받을 주제와 주기를 고른다.
  - 조종권은 연결에 묶는다. 명시적으로 받고 돌려주며, 연결이 끊기거나 데드맨이 걸리면 저절로 풀린다.
  - 클라이언트마다 최신 상태 한 칸만 두어, 와이파이가 밀려도 묵은 상태가 쌓이지 않게 한다.
  - 들어오고 나가는 모든 메시지를 컨트롤러가 MCAP로 기록한다. 재생은 그 파일을 같은 API로 다시 흘려보내는 작은 서버다.
  - 카메라 파이 → PC 영상은 pyzmq 그대로 둘 수 있다 ([0008](decisions/0008-environment-and-packages.md)).
- B (자랄 길): 여러 기계가 서로 얽힐 때(카메라를 여러 응용이 같이 쓰기, 응용 여럿이 동시에) 컨트롤러 안쪽에 Zenoh를 더하고, 브라우저용 출입구는 A를 그대로 둔다. Zenoh 조각들은 모두 같은 버전으로 묶어야 하고, 1.x 판끼리의 호환은 보장된 문서를 찾지 못했다.

## 4. 시뮬레이션 연결과 파이용 계획 라이브러리

### MuJoCo를 Rust에서

- [mujoco-rs](https://github.com/davidhozic/mujoco-rs) 7.0.0+mj-3.17.0 (2026-10-26, MIT 또는 Apache-2.0)
  - 공식 MuJoCo 공유 라이브러리에 동적으로 링크하고, Rust 뷰어가 있다 ([설치](https://mujoco-rs.readthedocs.io/en/latest/installation.html)).
  - 5.0(06-13), 6.0(08-31), 7.0(10-26)마다 API가 깨졌다 ([변경 기록](https://mujoco-rs.readthedocs.io/en/latest/changelog.html)). 최소 Rust 1.95다.
- [rusty_mujoco](https://github.com/rust-control/rusty_mujoco) 0.4.0은 MuJoCo 3.10을 묶는다. 그 밖의 크레이트는 MuJoCo 3 전에 멈췄다.
- 공식 MuJoCo는 3.17.0(2026-10-22)과 4.0.0(2026-11-06) 모두 linux-aarch64와 windows-x86_64 바이너리를 낸다. 레포의 Python mujoco는 3.17.0이다. 4.0.0을 묶는 Rust 크레이트는 아직 없다 ([4.0.0](https://github.com/google-deepmind/mujoco/releases/tag/4.0.0)).

### 시뮬레이션을 컨트롤러 뒤에 두는 예

| 예 | 시뮬레이션과 제어기가 붙는 방식 | 시간 |
|---|---|---|
| [mujoco_ros2_control](https://control.ros.org/jazzy/doc/mujoco_ros2_control/doc/index.html) | MuJoCo를 하드웨어 인터페이스 플러그인으로 감싸 제어 노드 안에서 돌린다 | 물리 시간을 내보낸다. 실제보다 빠르게, 정해진 수만큼 진행할 수 있다 |
| [PX4 SITL](https://docs.px4.io/main/en/simulation/) | 별도 프로세스, TCP로 액추에이터 명령과 센서 값을 주고받는다 | 함께 한 걸음씩(lockstep). 실제보다 빠르게도 느리게도 된다 |
| [ArduPilot JSON](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md) | UDP. 서보 출력과 프레임 번호를 보내면 시뮬레이터가 상태로 답한다 | 시뮬레이터가 시계를 쥔다 |
| [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) | 별도 MuJoCo 프로세스가 실물과 같은 DDS 메시지를 주고받는다 | 고정 주기 |
| Reachy Mini | 같은 데몬의 백엔드를 MuJoCo, 실물, 물리 없는 모드로 바꾼다 | 실시간 |
| PAROL6 | 가짜 시리얼 + 공유 메모리로 붙는 시뮬레이션 프로세스 | 실시간 |

| | A: Rust가 MuJoCo를 직접 부른다 | B-txn: Python 시뮬레이션 서버, 서보 거래 단위, TCP로 한 걸음씩 | B-bytes: Python이 Feetech 바이트를 흉내 낸다 |
|---|---|---|---|
| 같은 결과가 나오는가 | 자연스럽다 (한 스레드가 매 박자 진행) | 루프가 시뮬레이션 시간을 시계로 쓰고 매 박자 기다리면 된다 (PX4 방식) | 바이트 흐름에는 박자가 없어서 따로 신호가 필요하다 |
| 시험 속도 | 가장 빠르다 | 매 박자 로컬 왕복 한 번. 100 Hz면 실시간보다 훨씬 빠를 것 (추정) | 더 느리다 |
| 시험되는 Rust 코드 | 루프와 안전 검사 | 거기에 ID 대응, 방향·영점 변환, 상태 기준, 오류 길까지 | 거기에 프로토콜 코드까지 |
| Python 재사용 | 없다 (장면, 카메라, 뷰어가 Rust로 와야 한다) | 모두 (공식 뷰어, 설계에서 만든 MJCF, 비전용 카메라 그림) | 모두, 지금의 가짜 서보(`fake.py`)까지 |
| MuJoCo 판 맞추기 | Rust와 Python이 같이 올려야 하고 크레이트가 자주 깨진다 | Python만 | Python만 |
| 파이 | 기능 플래그로 빼야 한다 | Rust 쪽에 MuJoCo가 없다. 파이의 모터 프로세스를 랜 너머 PC 시뮬레이션에 붙일 수도 있다 | 같지만 Windows에 가상 시리얼(pty)이 없어서 어차피 TCP가 필요하다 |

### IK

우리 팔은 J2·J3·J4가 평행이라 IK-Geo의 "평행 축 셋" 꼴이고, UR5처럼 닫힌 해가 있다 ([IK-Geo](https://github.com/rpiRobotics/ik-geo), [논문](https://arxiv.org/abs/2211.05737)).

| 후보 | 판, 날짜 | 라이선스 | 파이 | 비고 |
|---|---|---|---|---|
| [ik-geo 크레이트](https://github.com/Verdant-Evolution/ik-geo-rust) | 0.1.2, 2024-07 이후 멈춤 | BSD-3 | nlopt를 cmake로 빌드한다. NLopt는 LGPL ([라이선스](https://nlopt.readthedocs.io/en/latest/NLopt_License_and_Copyright/)) | 평행 셋 풀이가 있고 모든 해를 준다 |
| [EAIK](https://pypi.org/project/EAIK/) | 1.2.2, 2026-06 | BSD-3 | Python 3.9~3.13용 aarch64 휠이 있다 (3.14는 없다) | 이미 `packages/design`에서 쓴다 |
| [mink](https://github.com/kevinzakka/mink) | 1.4.0, 2026-11 | Apache-2.0 | aarch64 휠이 있다 (mujoco, daqp 포함) | 컨트롤러에 MuJoCo와 MJCF가 따라온다 |
| [k](https://github.com/openrr/k) | 0.32.0, 2024-09 | Apache-2.0 | 순수 Rust | 야코비안 IK, 관절 한계 처리는 문서에 없다 |
| [rs-opw-kinematics](https://github.com/bourumir-wyngs/rs-opw-kinematics) | 3.0.0, 2026-09 | - | 순수 Rust | 구형 손목만 풀어서 우리 팔에 맞지 않는다 |

- 파이의 Ubuntu 22.04는 Python 3.10이다. 레포는 3.12 이상이라 uv가 관리하는 Python을 쓴다. 파이 4에서 EAIK와 mink가 얼마나 빠른지는 재지 않았다.

### 궤적과 시간 매기기

| 후보 | 판, 날짜 | 라이선스 | aarch64 | 비고 |
|---|---|---|---|---|
| [Ruckig](https://github.com/pantor/ruckig) (C++ 커뮤니티판) | 0.19.4 | MIT | 소스로 빌드 (C++20) | 500 µs 주기까지 된다. 중간 웨이포인트는 유료 Pro판이나 실시간이 아닌 클라우드 API로만 |
| Ruckig 공식 크레이트 | 0.19.4, 2026-07 | MIT | C++를 같이 빌드해서 교차 컴파일에 aarch64 C++ 도구가 필요하다 | |
| [rsruckig](https://github.com/petrikosk/rsruckig) | 3.0.1, 2026-08 | MIT | 순수 Rust | 커뮤니티판을 옮기고 자체 위치 한계와 웨이포인트 풀이를 더했다. 웨이포인트 풀이는 시간 최적이 아니다 (Pro판보다 평균 1.4%, 최악 7.3% 길다, 주장) |
| [TOPP-RA](https://github.com/hungpham2511/toppra) | Python 0.6.11, 2026-11 | MIT | Python 휠은 x86_64뿐, C++는 CMake | Python 지원은 그만둘 예정이라고 한다 |
| [copp](https://github.com/TOPP-THU/copp) | 0.2.3, 2026-09 | MIT (유료 PRO판 따로) | 순수 Rust | 속도, 가속, 토크, 저크 한계. 아직 젊다 |

- 모서리를 둥글게 잇는(블렌드) Rust 크레이트는 찾지 못했다.
