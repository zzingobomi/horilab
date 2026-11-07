# 참고 설계

다른 데스크톱 로봇팔에서 가져올 점과, 만든 사람들이 겪은 문제를 모았다. 설계를 따라 하려는 것이 아니다. 부품을 설계하기 전에 시행착오를 줄이려고 봤다.

조사는 2026-11에 했다.
- GitHub 문서와 이슈, 제조사 문서, MakerWorld 모델 67개와 그 댓글을 봤다.
- MakerWorld는 웹 API로 읽었다. Printables는 페이지가 막혀 검색 결과만 봤다.
- 숫자는 출처에 적힌 값이고, 직접 재 본 것은 아니다.

## 참고한 팔

| 팔 | 구동 | 가져올 점 |
|---|---|---|
| [SO-ARM100 / SO-101](https://github.com/TheRobotStudio/SO-ARM100) | STS3215 6개 (5축 + 그리퍼) | 서보 자리 시험편(게이지)을 먼저 출력한다. 관절마다 출력 혼과 뒷면 혼을 다 쓴다 (손목 롤만 한쪽). 케이블 클립, 손목 D405 마운트 |
| [SO-ARM-102](https://github.com/roboninecom/SO-ARM-102) | STS3235, STS3250, STS3215 | 서보 대신 출력한 서보 모형을 넣어 프레임 휨만 따로 쟀다. 팔을 다 펴고 300 g을 달면 SO-101 프레임은 7.5 mm, 위상 최적화한 PET-CF 링크는 3.4~3.9 mm 처졌다 ([사양](https://github.com/roboninecom/SO-ARM-102/blob/main/docs/specifications.md)). 알려진 문제: 드라이버가 닿지 않는 M2 나사가 있다. 관절을 닫기 전에 케이블 여유를 확인해야 한다 |
| [ElRobot](https://github.com/norma-core/norma-core/tree/main/hardware/elrobot) | ST3215 8개 (7축 + 그리퍼) | 나사를 두 종류만 쓴다. 배선할 때마다 관절이 끝까지 돌아가는지 확인한 뒤 덮개를 나사로 덮는다. 부품마다 글자를 붙이고, 서보 ID는 조립 전에 정한다. 그림 설명서가 75단계다 |
| [AM-ARM200](https://github.com/liyiteng/AM-ARM) | STS3215 등 (6축 + 그리퍼) | 나사는 M3×10 한 종류다. 나사를 모두 느슨하게 끼워 맞춘 뒤에 조인다. 손목 케이블은 덮기 전에 넣고, 케이블이 당기지 않는지 관절을 범위 끝까지 돌려 본다 ([조립](https://github.com/liyiteng/AM-ARM/blob/main/am-arm200/hardware_assembly.md)) |
| [ArmDojo](https://github.com/miraclewhite-tech/armdojo-hardware) | STS3215 6개 (5축) | 모든 관절에서 두 혼에 판을 대고 두 판 사이를 가로대로 잇는다. M3 너트를 홈에 넣는다. 맞춤에 중요한 출력 설정(구멍 보정, 첫 층 퍼짐 보정)을 적어 둔다. 너트 홈과 혼 면에는 서포트를 두지 않는다. 출력해 만든 사람은 아직 없다 |
| [so101-toolchanger](https://github.com/chaseab/so101-toolchanger) | SO-101용 도구 교체 | 3×12 mm 강철 핀 세 개가 V홈에 앉고, 손목 롤을 45° 돌려 잠근다. 도구 거치대에서 200번 꽂아 199번 됐다 |
| [Koch v1.1](https://github.com/jess-moss/koch-v1-1) | Dynamixel | 플라스틱에 바로 박던 나사를 너트로 바꿨다. 스냅 끼움은 프린터마다 맞춤이 달라 나사로 바꿨다 |
| [Niryo One](https://niryo.com/niryo-one-upgraded-version/) | J1~J3 스테퍼, J4~J6 Dynamixel | 팔 안으로 지나가는 케이블을 두 가닥으로 줄여 연결 문제를 줄였다. 영점은 출력한 표시를 맞춰 잡는다 |
| [PAROL6](https://docs.source-robotics.com/PAROL-docs/latest/) | 스테퍼와 감속기 | 공압 튜브가 케이블을 잡아 주는 등뼈 노릇을 한다. 영점을 잡은 뒤, 핀이 모든 관절의 표시 구멍을 지나는지 본다. J1에 멈춤을 둬서 한 바퀴 넘게 돌다가 선이 뜯기지 않게 한다. 모터 옆 부품은 60~70°C까지 올라 PETG로 뽑는다 |
| [SO-101 책상 클램프](https://makerworld.com/en/models/1796562) | 부속 | 공구 없이 손으로 돌리는 출력 나사다. 핀 두 개로 베이스 자리를 잡고, 미끄럼 방지 TPU 패드를 댄다 |

서보를 쓰는 팔 중에 J1에 크로스 롤러 베어링을 쓰거나 ST-3120을 쓰는 예는 찾지 못했다.

## 외형

협동로봇처럼 보이게 하는 방법이다.

- **UR식** ([UR3e](https://www.universal-robots.com/products/ur3e/))
  - 관절마다 둥근 집이 있고, 양쪽 끝에 색 뚜껑이 있다. 뚜껑은 나사로 고정하고, 관절을 손볼 때 연다.
  - 이음매마다 검은 띠(실링 고리)가 있고, 관절 사이는 민무늬 관이다. 베이스는 원뿔이고, 플랜지는 ISO 9409다.
  - 요소마다 한 색 부품으로 나눠 뽑을 수 있다.
  - [UR5e 1:5 출력 모형](https://makerworld.com/en/models/3408581)은 몸체를 한 색으로 뽑고 캡만 따로 뽑는다. 검은 띠 대신 얕은 홈을 팠는데, 사진에서는 그림자 선 정도로만 보인다.
- **서보로 움직이는 팔**
  - [myCobot 280](https://robotsguide.com/robots/mycobot)은 서보를 굽은 관 조각 안에 넣어서, 관절이 분할선으로만 보인다.
  - [RIG-Arm](https://makerworld.com/en/models/2913469)은 끝이 둥근 사각 단면 링크로 서보를 감싼다. 벽은 1.2 mm다.
  - [Kinova Gen3 lite](https://www.roscomponents.com/product/gen3-lite/)는 링크 끝을 관절 축 중심의 반원으로 만든다.
- **매끈한 한 덩어리 껍데기**(Franka, Kinova, Lite 6)는 큰 곡면이라 서포트가 많이 들고, 열기 어렵다.
- **주의할 것**
  - 열:
    - STS3215는 덮개 없이도 110분 뒤 71°C까지 올랐다 ([hardware.md](hardware.md#다른-사람이-잰-값)).
    - 뱀부랩 PLA Basic은 57°C, PETG-CF는 74°C부터 변형된다 (판매처가 옮긴 뱀부랩 자료: [PLA](https://www.crea3d.com/en/filaments/1276-bambu-pla-basic.html), [PETG-CF](https://www.crea3d.com/en/filaments/1308-bambu-petg-cf.html)).
    - 덮개가 서보 온도를 얼마나 올리는지는 확인하지 못했다.
  - 무게: 지름 50 mm, 길이 170 mm, 벽 1.2 mm인 덮개는 약 40 g이다(계산). 팔을 수평으로 폈을 때 J2에 약 1.1 kg·cm가 더 걸린다.
  - 끼워 맞추거나 붙인 뚜껑은 다시 열 수 없다 ([895746 댓글](https://makerworld.com/en/models/895746)).
  - 케이블이 밖으로 지나가면 협동로봇처럼 보이지 않는다 ([mockway](https://makerworld.com/en/models/3068526), [PRIMO](https://makerworld.com/en/models/2473047)).

## 피할 것

만든 사람들이 겪은 문제다.

- **서보 자리가 맞지 않는다.** 도면 치수나 한 판매처 부품에 맞춰 그리면 안 맞는 경우가 MakerWorld에서 가장 흔한 불평이다. 실물을 재고 시험편으로 맞춘다 ([design.md](design.md#출력-전-확인)).
- **PLA 프레임이 휜다.** SO-101은 서보를 빼고 프레임만으로 300 g에 7.5 mm 처진다. 이 팔은 링크가 더 길다.
- **서보가 뜨거워진다.** 시험값은 [hardware.md](hardware.md#다른-사람이-잰-값)에 있다. PLA는 55~60°C부터 무르다.
- **긴 링크가 층 사이로 갈라지고, 첫 층이 실패한다** (SO-101).
- **플라스틱 나사산과 핀 축이 헐거워진다** (Koch, MakerWorld 여러 모델). 너트 홈을 쓴다 ([0003](decisions/0003-fastening-nut-traps.md)).
- **케이블이 짧거나 끼이거나 감겨서 뜯긴다** (SO-ARM-102, AM-ARM, PAROL6, reBot). 끌리는 배선이 커넥터를 닳게 한 예도 있다.
- **나사에 드라이버가 닿지 않는다.** 판이 서보 나사를 가리는 경우도 있다 (SO-ARM-102).
- **그리퍼가 물체를 쥔 채 멈춰 있으면 서보가 과열되거나 부서진다** (ElRobot, LeRobot). 닫는 쪽 힘을 전류로 제한한다.
- **버스에 다른 서보를 섞으면 문제가 난다.** 전압판이 다르면 과전압 경고가 나고, 펌웨어가 다르면 통신 오류가 난다 ([lerobot#1010](https://github.com/huggingface/lerobot/issues/1010)).
- **끝까지 밀어 영점을 잡는 자동 캘리브레이션이 위험하다.** 시작 자세에 따라 팔을 책상에 눌렀다 (ElRobot). 영점은 핀으로 잡는다.
- **베이스가 흔들리거나 넘어진다** (PAROL6, myCobot). PAROL6은 베이스를 옆으로 밀어 들리거나 밀리지 않는지 본다.
- **D405 커넥터가 떨어진다.** USB 2로 떨어지거나 끊기는 문제가 있다. 나사 고정 케이블을 세게 조여 커넥터 납땜이 떨어진 예도 있다 ([librealsense#14406](https://github.com/realsenseai/librealsense/issues/14406)).
- **출력한 감속기와 휘는 프레임은 반복 정밀도를 망친다** (Faze4 저자). 이 팔은 감속을 서보 안에서만 한다.
- **부품표, 나사 목록, 설명서가 없다.** 가장 흔한 불평이다. 부품표와 조립 순서를 같이 만든다 ([design.md](design.md#설계-방식)).
