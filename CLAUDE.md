# CLAUDE.md

horilab: 3D 프린터로 출력한 부품과 Feetech 서보로 만드는 6축 로봇팔 호리봇(horibot). 이전 프로젝트도 레포 이름이 horibot이어서, 그쪽은 문서에서도 말할 때도 늘 "이전 프로젝트"라고 부른다. 문서는 모두 한국어로 쓴다.

## 세션을 시작하면

1. [docs/status.md](docs/status.md)를 읽고 지금 단계와 다음 할 일을 확인한다.
2. [docs/bom.md](docs/bom.md)의 "지금 주문할 것"과 "배송 중"을 보고, 아직 주문하지 않은 것과 도착했을 때가 지난 것을 사용자에게 먼저 짧게 알린다.
3. 작업에 필요한 주제 문서만 읽는다. 문서 목록은 [README.md](README.md)에 있다.
4. `docs-local/`이 있으면 개인 참고 자료다. 이 폴더가 없는 PC도 있으니, 작업에 꼭 필요한 정보는 여기에 두지 않는다.
5. 처음 쓰는 PC면 [docs/setup.md](docs/setup.md)대로 준비한다.

## 작업 방식

- 구현보다 결정이 먼저다. 작업이 기대는 결정이 docs에 없으면 먼저 정한다.
- 설계(골격, 오차 예산, 팔 부품 형상)는 최신 Fable 모델의 세션에서 한다. 다른 모델의 세션에서 설계를 시작할 차례가 되면, 시작하기 전에 한 줄로 알려 모델을 바꿀 수 있게 한다. 설계가 아닌 일(시험 분석, 기록 정리, 문서 고침)은 그대로 한다.
- 고민이 필요한 선택은 대안과 트레이드오프를 따져 근거와 함께 하나를 고른다. 비용과 이미 가진 장비도 같이 따진다. 당연한 관례는 그냥 따른다.
  - 설계는 로봇이 할 일과 그 자리에 필요한 것에서 시작한다. 가진 것(책상 크기, 가진 볼트 등)에 맞추려고 설계를 낮추지 않는다. 가진 것이 맞지 않으면 바꾸거나 맞는 것을 산다.
  - 치수, 배치, 구조처럼 계산과 검토로 답이 나오는 기술 결정은 Claude가 꼼꼼히 따져서 정한다. `docs/decisions/`에 기록하고 결과와 근거를 알린다.
  - 돈이 드는 것, 외형 취향, 목표와 범위처럼 사용자가 정할 것은 추천과 함께 묻고, 승인을 받으면 기록한다.
  - 설계 대안은 한 지표(무게, 되는 자세의 몫 하나)만 보고 고르지 않는다. 정밀도, 기구학과 동역학, 외형, 조립, 케이블과 자잘한 것, 다시 뽑지 않기를 같이 놓고 따진다.
  - 맞춤 정확도, 강도, 목표(정밀한 팔)가 출력 편의(서포트 없이 뽑기 등) 같은 기본 방침보다 먼저다. 기본 방침이 이들과 부딪히면 기본 방침을 접는다. 기본 방침 때문에 형상이나 방법을 바꿀 때는 정확도와 강도를 깎지 않는지 따로 확인해 알린다.
- 이어받은 내용(이전 문서, 이전 프로젝트)도 그대로 믿지 않고 따져 본다. 이전 프로젝트에서는 사실(무엇을 썼고 무슨 일이 있었나)만 참고한다. 결론(원인 진단, 무엇이 낫다)은 그때의 설계나 시험이 잘못됐을 수 있어서 검증되지 않은 가설로 보고, 이 프로젝트의 계산, 시뮬레이션, 실측으로 다시 따진다.
- 규칙과 절차(이 문서와 conventions.md 포함)는 목적에 맞게 쓴다. 상황에 맞지 않으면 그대로 따르지 말고, 더 나은 방법을 제안하거나 규칙을 고친다.
- 확인하지 않은 사실(부품 사양, 커넥터, 호환성, 가격)은 단정하지 않는다. 사진이나 조사로 확인한 뒤에 말한다. 사용자가 말하지 않은 규칙(특히 돈과 범위)이나 계산하지 않은 기술 주장은 문서에 넣지 않는다. 돈과 범위의 규칙은 사용자가 말한 뜻 그대로 적고, 근거가 어림이면 어림이라고 밝힌다.
- 부품 조사(제품, 품질, 사는 곳, 값, 배송 기간)는 묻지 않고 바로 한다. 조사 에이전트에는 필요한 도구(웹 검색, 페이지·사진 내려받기 등)를 다 쓰게 한다. 사용자가 허용 창을 누르지 않도록 권한은 설정에 미리 둔다. 한 사이트에 자동 요청을 몰아 보내지 않는다 (사용자 인터넷 주소가 그 사이트에서 막힐 수 있다). 그 밖의 웹 검색과 조사는 무엇을 왜 찾을지 말하고 허락을 받은 뒤에만 한다. 레포 문서와 [이전 프로젝트](https://github.com/zzingobomi/horibot) 기록은 묻기 전에 먼저 찾아본다.
- 에이전트(서브에이전트)는 부품 조사가 아니면 띄우기 전에 무엇을 왜 몇 개 맡기는지 말하고 허락을 받는다. 웹을 쓰지 않는 조사도 마찬가지다.
- 질문은 한 번에 1~2개만, 답이 필요한 시점에 한다. 답은 짧고 직접적으로 한다.
  - 사용자가 질문하면 그 답만 한다. 답하면서 시키지 않은 파일 수정이나 설계 변경을 시작하지 않고, 묻지 않은 분석(가진 것과 견주기, 조건, 권고)을 덧붙이지 않는다. 목록을 달라고 하면 목록만 준다. 답하다가 문제를 찾으면 고치지 말고 한 줄로 알린 뒤 지시를 기다린다.
  - 설명은 전문 용어 없이 쉬운 말로, 지금 할 일 위주로 한다. 꼭 필요하지 않은 확인 단계는 꺼내지 않는다. 부품 번호가 든 문장은 무엇을 하는지(뽑는다, 끼워 본다, 산다)를 먼저 쓰고, 살 것이 없으면 그렇게 밝힌다.
- Claude는 실물을 볼 수 없다. 맞는지는 측정값과 사진으로 확인한다. 다시 설계하고 다시 출력하는 일을 계획에 넣지 않고, 출력 전에 확인할 수 있는 것은 모두 확인한다 ([docs/design.md](docs/design.md#출력-전-확인)).
  - 실물 값(측정, 사진)을 기다리며 설계를 멈추지 않는다. 공개 자료(사양서, 도면, 공식 CAD)와 레포 문서, 이전 프로젝트 기록으로 값을 가정해 먼저 그리고, 실물 값이 오면 숫자를 바꿔 검사를 다시 돌린다. 실물이 없는 PC에서 할 수 있는 일을 최대한 해 두려는 것이다.
  - 가정한 값은 코드와 문서에 가정이라고 표시하고, [status.md](docs/status.md)의 사용자 순서에 뽑기 전에 확인할 것으로 모은다. 실물 값이 꼭 있어야 하는 때는 뽑기 직전이다.
  - 가정이 틀리면 숫자가 아니라 배치가 바뀌는 곳은, 그 부분만 고칠 수 있게 그린다.
- 장비와 프로그램이 버티는지(부하, 지연, 통신 시간, 서보 힘과 열)도 같다. 만들기 전에 잴 수 있는 것은 미리 잰다. 재기 전에 통과 기준과, 모자랄 때 고를 길과 그 순서를 정해 둔다. 다 만든 뒤에 장비 한계나 최적화 문제로 되돌아가는 시행착오를 줄이려는 것이다.
- 제작과 조립은 기본 공구만으로 쉽게 할 수 있는 것을 고른다. 대안을 비교할 때 조립 난이도도 같이 따진다 ([docs/design.md](docs/design.md#설계-원칙)).
- 모든 작업 PC에 NVIDIA GPU가 있다. 도구와 방법을 고를 때 GPU를 쓰는 방법을 먼저 따진다 ([docs/software.md](docs/software.md#개발-환경)).
- 실물 로봇을 움직이는 명령은 확인을 받은 뒤에 실행한다 ([docs/hardware.md](docs/hardware.md#안전)).
- 도구 설치 같은 환경 작업은 Claude가 직접 한다.
- 살 것을 놓치지 않게 챙긴다. 설계하다 살 것이 생기면 [0004](docs/decisions/0004-purchasing.md)의 원칙(품질 우선, 국내 배송 우선)대로 조사해 고르고, 고른 까닭(품질, 설계, 값, 배송, 조립)을 사용자에게 보고한다. 승인을 받으면 [부품표](docs/bom.md)에 확정하고 주문했는지 묻는다. 다음 단계로 넘어갈 때도 그 단계에 필요한데 아직 손에 없는 것을 알린다. 주문했다거나 받았다는 말을 들으면 부품표를 고친다.
- 작업 단위가 끝나면 `docs/status.md`를 고친다.
- 사용자가 손으로 하는 일(출력, 측정, 배선, 실물 시험)은 Claude가 무엇이 무엇에 기대는지와 급한 정도를 따져 순서를 정하고, 한 번에 한 단계씩 짧게 이끈다: 무엇을, 어떻게, 끝나면 무엇을 알려 줄지. 조립 순서, 사진 찍는 법, 캘리퍼스로 재는 자리 같은 안내는 글로만 하지 않고 3D 페이지로 보여 준다. 순서는 [status.md](docs/status.md)의 "사용자 순서"에 두고, 단계가 끝나거나 사정이 바뀌면 고친다.
  - 출력할 부품은 한글 이름 옆에 STL 파일 이름과 바꿀 설정(프리셋, 채움)을 같이 쓰고, 3D 페이지에도 부품마다 파일 이름을 보인다. 알리기 직전에 파일을 다시 만들어 지금 코드와 같은지 본다.
  - 측정이나 실물 시험을 하는 자리에서는 시작하기 전에, 뒤의 설계와 분석에 쓸 값(쓰기 전 상태, 조립 전에만 보이는 것, 화면 값, 사진, 무게)을 빠짐없이 정해 그 자리의 순서에 넣는다. 실물을 다시 준비하는 일이 가장 비싸다.
- 커밋은 바로 하지 않는다. 무엇을 왜 바꿨는지 사용자에게 보고하고, 최종 승인을 받으면 커밋하고 push한다. 커밋은 의미 있는 단위(결정 하나와 그 코드·시험·문서, 또는 기능 하나)로 한다 ([conventions.md](docs/conventions.md#git)). 정하는 중인 것(살 것, 고를 것)이 있으면 정해진 뒤에 관련 변경을 한 번에 보고한다. 커밋하기 전에 git 작성자(user.name, user.email)가 기존 커밋과 같은지 확인한다.

## 자주 쓰는 명령

- 환경 맞추기: `uv sync`
- 커밋 전 검사. 모두 통과해야 커밋한다:
  - `uv run ruff format --check .`
  - `uv run ruff check .`
  - `uv run pyright`
  - `uv run pytest` (CPU 수만큼의 프로세스로 나눠 돌린다, 1분 40초쯤. 파일 하나만 볼 때는 `-n 0`을 붙인다)
  - `cargo fmt --all --check`
  - `cargo clippy --workspace --all-targets -- -D warnings`
  - `cargo test --workspace`
- 팔 설계 검토와 3D 페이지 ([0029](docs/decisions/0029-joint-parts.md)):
  - `uv run python -m horilab_design.parts.wrist` (첫 관절: 겹침, 틈, 무게, 걸침), `... parts.shoulder` (어깨), `... parts.base` (J1), `... parts.flange` (J6 플랜지), `... parts.camera_mount` (D405 받침), `... parts.gripper` (평행 그리퍼, [0030](docs/decisions/0030-parallel-gripper.md)), `... parts.links` (링크)
  - `uv run python -m horilab_design.studies.clamp_plate` (허브 판이 베어링 안쪽 링을 누르는 힘), `... studies.error_budget` (오차 예산)
  - `uv run python -m horilab_design.studies.ur_arm [compare|zone|lengths|view|calib|accel 60]` (팔 전체: 쌓임과 무게, UR3e와 같은 자세 시험, 주 작업 영역, 링크 길이 후보, D405 화면, 손목 카메라의 캘리브레이션 자세, 관절의 가속 상한)
  - `uv run python -m horilab_design.studies.wiring` (배선: 관절을 넘는 선 길이, 구간마다 쓸 선, D405 선, [0031](docs/decisions/0031-wiring.md))
  - `uv run python -m horilab_design.pages.joint_page`, `... pages.arm_page` (`build/pages/`에 3D 페이지를 만든다: `joint.html`, `shoulder.html`, `base.html`, `wrist_end.html`, `gripper.html`, `arm.html`. 올려 둔 페이지 "STS 관절 구조", "어깨 관절 구조", "J1 관절 구조", "손목 끝 구조", "평행 그리퍼 구조", "horilab 팔 전체"를 고칠 때는 아티팩트 목록에서 제목으로 찾아 같은 주소에 다시 올린다)
- 시뮬레이션 서버 ([0033](docs/decisions/0033-planning-and-simulation.md)): 모델을 먼저 `uv run python -m horilab_design.mjcf`로 쓰고(`build/sim/horibot.xml`) `uv run python -m horilab_sim [--port 5600] [--realtime]`로 띄운다. 컨트롤러가 실물 서보 대신 TCP로 붙는다. `cargo test`의 시뮬레이션 통합 시험은 `.venv`의 Python으로 서버를 띄우므로 `uv sync`가 먼저다.
- 파이용 실행 파일 만들기 ([0026](docs/decisions/0026-pi-runtime-deploy-records.md)): `uv run tools/pi_build.py -p <크레이트> --bin <실행 파일>`
- 경로 계획과 실행 루프 계산 시간 재기 (0026의 미리 재기 3): `cargo run --release -p horilab-controller --example plan_timing`. 파이에서는 `uv run tools/pi_build.py -p horilab-controller --example plan_timing`로 빌드해 `target/aarch64-unknown-linux-musl/release/examples/plan_timing`을 돌린다
- 실물 서보·카메라 테스트는 `uv run pytest --hardware -n 0`이다 (한 프로세스로만 돈다). 실물이 움직이므로 확인을 받은 뒤에만 돌린다.
- 서보 첫 연결(찾기, 덤프, 통신 시간, ID 바꾸기): `uv run python -m horilab_servo ports|scan|dump|timing|set-id` ([docs/servo-setup.md](docs/servo-setup.md)). `timing`과 `set-id`는 서보에 값을 쓰므로(움직이지는 않는다) 확인을 받은 뒤 `--yes`로 돌린다.
- 로봇 설정 확인과 값 적용: `cargo run -p horilab-controller -- settings|ports|apply` ([docs/servo-setup.md](docs/servo-setup.md#값-적용)). `apply --yes`는 서보 EEPROM에 쓰므로(움직이지는 않는다) 확인을 받은 뒤에 돌린다.
- 맨 서보 확인: `cargo run -p horilab-controller -- check|check-state --port <포트> --id 20` ([docs/servo-setup.md](docs/servo-setup.md#맨-서보-확인)). `check --yes`는 서보를 움직이므로 확인을 받은 뒤, 사용자가 보는 창으로 띄운다 (손으로 하는 단계에서 사용자가 Enter를 누른다). `check-state`는 `--situation <번호>`로 Claude가 돌린다. 한 단계만 다시 할 때는 `--only <단계>`다.
- 시험 기록 올리고 받기 ([0022](docs/decisions/0022-records-and-sync.md)). 실물 시험이 끝나면 올리고, 분석을 시작할 때 받는다:
  - `uv run tools/sync.py push`
  - `uv run tools/sync.py pull` (경로를 주거나 `--max-age 3d`로 골라 받는다)

## 문서 쓰는 법

- 결정, 근거, 사실만 쓴다. 공개 레포라서 대화 내용을 옮기거나 개인에 대한 서술을 넣지 않는다.
- 정보는 한 곳에만 쓰고 다른 곳에서는 링크한다.
- 문서에는 지금 유효한 것만 둔다. 지난 내용과 바뀐 이력은 git 기록과 커밋 메시지에 있다. 바깥에서 가져온 사실은 원문을 확인했는지 같이 적는다.
- 문서, 코드, 테스트, git의 자세한 규칙은 [docs/conventions.md](docs/conventions.md)에 있다.
