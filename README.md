# crazyflie-imo

Crazyswarm2 기반 Crazyflie 비행 테스트 레포.

- 단일 / 군집 예제 모두 제공
- Flow deck(opticalflow) / mocap(Qualisys) 예제 모두 제공
- IMU raw · 모터 PWM · pose 를 rosbag 으로 기록

## 구성

```
ros2_ws/src/crazyflie-imo
├── crazyswarm2/               # Crazyflie ROS 2 스택 (서버·드라이버)
├── motion_capture_tracking/   # 모션 캡쳐 패키지
└── crazyflie_test/            # 비행 테스트 패키지
```

## 실행 환경

- Ubuntu 22.04 + ROS 2 Humble
- Ubuntu 24.04 + ROS 2 Jazzy

## 설치

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/sungho2574/crazyflie-imo.git
```

이미 `--recursive` 없이 클론했다면

```bash
git submodule update --init --recursive
```

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # 최초 1회 crazyswarm2 의존성 설치
colcon build --symlink-install
source install/setup.bash
```

## 실행

### 기본 예제 (시뮬레이션 + rviz2)

- Terminal 1: 시뮬레이션 서버 가동

```bash
ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=sim
```

- Terminal 2: 알고리즘 코드 실행

```bash
ros2 run crazyflie_test hello_world
```

- Terminal 3: 시각화
  - Global Options → Fixed Frame → `world` 선택
  - Add 버튼 클릭 → TF 선택

```bash
rviz2
```

### 확장 버전 (launch/logging/test)

```bash
# 터미널 1 — 서버 (config 적용 지점)
ros2 launch crazyflie_test launch.py mode:=opticalflow   # 기본값 (Flow deck 단일)
# optical flow 멀티 기체 (편대)
ros2 launch crazyflie_test launch.py mode:=opticalflow_multi
# mocap (QTM 세팅 필요)
ros2 launch crazyflie_test launch.py mode:=mocap
# 하드웨어 없이 로직 검증
ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=sim

# 터미널 2 — 로깅 (비행 직전 시작). 멀티는 기체별 토픽으로 교체 (예: /cf1/... /cf2/... /cf3/...)
# /poses = mocap 원본 = ground truth (mocap 모드에서만 존재)
# /cf231/pose = 드론 온보드 추정값 (GT 와 비교용)
ros2 bag record /poses /cf231/pose /cf231/imu_raw /cf231/motor_pwm \
  -o ~/flight_logs/$(date +%Y%m%d_%H%M%S)

# 터미널 3 — 테스트 스크립트 (서버 유지한 채 여러 번 실행 가능)
ros2 run crazyflie_test hello_world
ros2 run crazyflie_test goto_square
ros2 run crazyflie_test figure8            # 8자
ros2 run crazyflie_test multi_square       # 편대 3기체
ros2 run crazyflie_test gate_flight        # 게이트 코스
```

- `launch.py` 인자:
  - `mode`: `opticalflow`|`opticalflow_multi`|`mocap` (default `opticalflow`)
  - `backend`: `cflib`|`cpp`|`sim` (default `cflib`)

### 8자 궤적

```bash
ros2 run crazyflie_test figure8_continuous --laps 3 --period 8.0 --a 1.0 --b 0.5
```

| 옵션          | 기본      | 설명                                |
| ------------- | --------- | ----------------------------------- |
| `--a` / `--b` | 1.0 / 0.5 | x(전진) / y(좌우) 진폭 [m]          |
| `--period`    | 8.0       | 한 바퀴 주기 [s]. **작을수록 빠름** |
| `--laps`      | 3         | 바퀴 수                             |
| `--height`    | 1.0       | 비행 고도 [m]                       |
| `--rate`      | 50        | setpoint 스트리밍 주파수 [Hz]       |
| `--ramp`      | 2.0       | 시작/종료 가감속 시간 [s]           |

- 리사주 8자(`x=A·sin φ`, `y=B·sin 2φ`)는 어느 위상에서도 속도가 0이 아니라 **원점에서 멈추지 않는다.**
  호버(속도 0)에서 매끄럽게 진입하려고 위상 속도 φ̇ 를 0→최대→0 으로 램프시킨다(시간 워핑) —
  경로는 그대로 두고 시작/종료만 부드럽게 만드는 방식이라 `t=0` 에서 속도·가속도가 정확히 0이다.
- ⚠️ `cmdFullState` 는 **low-level 제어**라 high-level commander 를 우회한다. 상태추정이 튼튼해야 하므로
  **mocap 모드 권장**, opticalflow 라면 `--period` 를 크게(느리게) 잡을 것.
- 실행 시 형상·최대 속도·최대 가속도를 출력하니 확인 후 날릴 것.

### 게이트 코스

`crazyflie_test/config/gates.yaml` 에 적힌 게이트를 **번호 순으로 전부 통과**한다.
경로는 **TOGT-Planner(FSC-Lab) 로 오프라인에서 시간최적 궤적**을 만들어 다항식
CSV(`config/gate_trajectory.csv`)로 저장해 두고, 비행은 **크플 펌웨어의 온보드
궤적 추종기**(High-Level Commander: `uploadTrajectory` + `startTrajectory`)가
실행한다. crazyswarm2 의 `figure8.py` 와 같은 표준 방식이라, 펌웨어가 온보드에서
궤적을 추종하고 통신이 끊겨도 계속 따라간다(직접 setpoint 를 스트리밍하지 않는다).

    gates.yaml ─(오프라인: TOGT)─► gate_trajectory.csv ─► 펌웨어 uploadTrajectory ─► 비행

동작: `start` 좌표에서 이륙 → 게이트 1..N 통과 → `start` 상공 복귀 → 착륙.
이륙 지점과 착륙 지점은 같은 곳이다.

#### 1. 궤적 계획 (오프라인, 게이트를 바꿨을 때만)

기본 `gates.yaml` 로 만든 `config/gate_trajectory.csv` 가 이미 커밋돼 있어 **평소
비행에는 이 단계가 필요 없다.** 게이트 배치를 바꿨을 때만 다시 만든다.

TOGT-Planner 소스는 `crazyflie_test/togt_tools/TOGT-Planner/` **git 서브모듈**로
들어 있다. `--recursive` 로 클론했으면 이미 받아져 있고, 아니면 한 번만:

```bash
git submodule update --init --recursive     # TOGT-Planner 서브모듈 받기 (최초 1회)

# gates.yaml → TOGT → config/gate_trajectory.csv (togt_plan 을 서브모듈에서 자동 빌드)
ros2 run crazyflie_test plan_gate_trajectory
# 또는: python3 -m crazyflie_test.plan_gate_trajectory
# 다른 TOGT 경로를 쓰려면: --togt-dir /path  (또는 환경변수 TOGT_DIR)
```

출력에서 **조각 수 ≤ 32**(펌웨어 궤적 메모리 한계), 방 이탈 0, 게이트 프레임 간섭
없음을 확인한다. 조각 수가 넘치면 `togt_tools/params/refine/cf_planning.yaml` 의
`piecesPerSegment` 를 줄인다. 크플용 동역학 파라미터(질량 33 g, 추력·각속도 한계,
방 8×11 경계)는 `togt_tools/params/` 에 있다.

#### 2. 계획 확인 (기체 불필요)

```bash
ros2 run crazyflie_test gate_flight --dry-run
```

통과 순서·궤적 시간·조각 수·방 경계·게이트 프레임 여유를 출력한다.

#### 3. 시뮬레이션 (터미널 4개)

게이트 비행은 기체의 `initial_position` 이 `gates.yaml` 의 `start` 와 같아야 한다
(기체 추정 원점 = 게이트 world 좌표 원점). 그래서 `initial_position` 을 start 로
맞춘 전용 robot 설정 `config/crazyflies_gate.yaml` 을 패키지에 두었고, launch 의
**`mode:=gate`** 가 이 파일을 자동으로 고른다(launch 코드는 그대로 — `mode` 값으로
`crazyflies_<mode>.yaml` 을 선택하는 기존 방식 그대로다). `gates.yaml` 의 start 를
바꾸면 이 파일의 `initial_position` 도 같이 바꾼다(다르면 gate_flight 가 이륙 거부).

⚠️ 노드는 **각각 별도 터미널에서 포그라운드로** 실행하고, 끝낼 땐 `Ctrl+C` 로 확실히
종료한다. `&` 로 백그라운드에 돌리면 잘 안 죽고 남아, 특히 `gate_markers` 가 겹치면
**rviz 에 경로가 두 개 왔다갔다 깜빡인다**(`/gate_course/path` 퍼블리셔가 2개가 됨).

```bash
# T1 — sim 서버 (mode:=gate 가 crazyflies_gate.yaml 을 자동 선택)
export PYTHONPATH=~/crazyflie/crazyflie-firmware/build:$PYTHONPATH   # sim 에 cffirmware 필요
ros2 launch crazyflie_test launch.py mode:=gate backend:=sim
```
```bash
# T2 — 게이트·궤적 마커 (별도 터미널, 하나만)
ros2 run crazyflie_test gate_markers
```
```bash
# T3 — rviz
rviz2 -d $(ros2 pkg prefix crazyflie_test)/share/crazyflie_test/config/gate_course.rviz
```
```bash
# T4 — 비행
ros2 run crazyflie_test gate_flight
```

> 경로가 두 개 겹쳐 깜빡이면 옛 `gate_markers` 가 안 죽고 남은 것이다:
> `pkill -f gate_markers` 후 하나만 다시 띄운다. 확인: `ros2 topic info /gate_course/path`
> 의 `Publisher count` 가 **1** 이어야 한다.

> **sim 이 `startTrajectory` 에서 `plan_start_trajectory() missing ... start_yaw` 로 죽으면**
> 설치된 `crazyflie_sim` 이 최신 `cffirmware`(7인자 시그니처)보다 오래된 것이다.
> `crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_sil.py` 의 `startTrajectory` 에서
> `firm.plan_start_trajectory(self.planner, traj, reverse, relative, relative, startfrom, self.cmdHl_yaw)`
> 로 `relative_yaw`·`start_yaw` 를 넘기면 된다(이 레포엔 이미 반영). symlink-install 이라
> 재빌드 불필요. 실기체(진짜 펌웨어)는 서명이 맞으므로 이 문제가 없다.

#### 4. 실기체 (모캡 권장)

위치를 어디서 얻느냐에 따라 방식이 다르다:

- **모캡(Qualisys)** — 트래커가 **절대 위치**를 준다. 원점은 `initial_position` 이
  아니라 **QTM 캘리브레이션 원점**이다. 그래서 `mode:=mocap` 을 그대로 쓰고,
  `gate_flight --mocap` 으로 `initial_position=start` 검사를 건너뛴다.
  ⚠️ **QTM world 원점(캘리브레이션 L프레임)을 방 중심(gates.yaml 원점)에 놓고 x/y
  축을 gates.yaml 과 정렬**해야 게이트 좌표가 맞는다. 기체는 mocap 볼륨 안 아무 데나
  놓아도 되고(트래커가 추적), 이륙 후 궤적 시작점으로 이동해 코스를 돈다.

  ```bash
  ros2 launch crazyflie_test launch.py mode:=mocap        # QTM + mocap 노드
  ros2 run crazyflie_test gate_flight --mocap --timescale 2   # 첫 비행은 느리게
  ```

- **Flow deck(외부 트래커 없음)** — 온보드 추정이라 원점 = `initial_position`.
  기체를 `start` 좌표에 기수 +x 로 놓고 `crazyflies_gate.yaml` 의 `initial_position`
  (= start)·`uri` 를 맞춘 뒤 `mode:=gate`. (sim 검증과 같은 설정.)

  ```bash
  ros2 launch crazyflie_test launch.py mode:=gate
  ros2 run crazyflie_test gate_flight --timescale 2
  ```

> `crazyflies_gate.yaml`(= `initial_position` 을 start 로 맞춘 설정)은 **외부 트래커가
> 없는 경우(sim·Flow deck)** 를 위한 것이다. 모캡은 절대 위치를 받으므로 필요 없다.

`gate_flight` 는 이륙 전에 `initial_position` 일치·배터리·tumble·lock 을 점검하고
문제가 있으면 **이륙하지 않는다**. 익숙해지면 `--timescale 1` 로 정상 속도.

| 옵션            | 기본  | 설명                                            |
| --------------- | ----- | ----------------------------------------------- |
| `--timescale`   | 1.0   | 궤적 시간 배율. >1 이면 느리게 (첫 비행 권장)    |
| `--trajectory`  | -     | 궤적 CSV (기본 `config/gate_trajectory.csv`)    |
| `--height`      | yaml  | 이륙 고도 [m]. 기본은 `start.takeoff_z`         |
| `--min-battery` | 3.85  | 이 전압 미만이면 이륙 안 함 [V]                  |
| `--mocap`       | -     | 모캡으로 절대 위치를 받는 경우 (아래 참고)       |
| `--force-start` | -     | initial_position 이 start 와 달라도 강행        |
| `--dry-run`     | -     | 계획만 출력하고 비행하지 않음                    |

#### 위치 출처: 모캡 vs 온보드 추정 (중요)

게이트 좌표(방 중심 = 원점)와 기체가 아는 자기 위치가 **같은 좌표계**여야 궤적이
맞는다. 위치를 어디서 얻느냐에 따라 원점을 맞추는 방법이 다르다:

| 방식              | 위치 출처         | world 원점            | 필요한 것                          |
| ----------------- | ----------------- | --------------------- | ---------------------------------- |
| **모캡(Qualisys)**| 트래커 절대좌표   | **QTM 캘리브레이션**  | QTM 원점 = 방 중심, 축 정렬        |
| Flow deck / sim   | 온보드 추정       | `initial_position`    | `initial_position = start`         |

- **모캡** — `initial_position` 은 원점이 아니다(트래커가 절대 위치를 준다). 따라서
  `mode:=mocap` + `gate_flight --mocap` 을 쓰고, `initial_position=start` 검사는
  건너뛴다. 대신 **QTM 캘리브레이션 원점을 방 중심(gates.yaml 원점)에 놓고 x/y 축을
  gates.yaml 과 정렬**해야 한다. 기체는 볼륨 안 아무 데나 놓아도 된다.
- **Flow deck / sim** — 온보드 추정이라 원점 = `initial_position`. 그래서 게이트용
  전용 설정 `crazyflies_gate.yaml`(`initial_position=start`)을 `mode:=gate` 로 쓴다.
  기체를 `start` 좌표에 놓고 그 값을 맞춰야 한다(다르면 gate_flight 가 이륙 거부).
  `crazyflies_gate.yaml` 은 **외부 트래커가 없는 이 경우 전용**이다.

#### 좌표·게이트 규약 (중요)

- `gates.yaml` 의 x, y 는 **world 절대 좌표**, 방 중심이 원점.
- **`yaw_deg` 는 위에서 봤을 때 시계방향(CW)** 기준이다. 진행 방향 =
  `(cos(-yaw_deg), sin(-yaw_deg))` — 게이트 맵 생성기 `gate_map.py` 의 `yaw_rad()`
  와 같다. **`gate_map.py` 헤더의 `yaw_deg ... CCW` 주석은 실제(`-yaw_deg`, CW)와
  반대이니 고칠 것.** CCW 로 읽으면 게이트 방향이 x축 대칭으로 뒤집혀 코스가 완전히
  달라진다.
- 이 코스는 게이트가 안전여유선(y=4.0)에 붙어 있어, 진입/탈출 구간이 여유 영역을
  조금 벗어난다(방 경계 y=5.5 까지는 넘지 않음). 그래서 `벽까지 여유 NG` 경고가
  뜨는데, 게이트 배치에서 오는 것이라 `gate_map.py` 기준 경로도 마찬가지다.

#### 게이트 통과 마진 (실기체 충돌 방지)

TOGT 는 게이트 물리 개구부(`inner_size`, 기본 0.5 m)를 그대로 쓰지 않고, **가운데로
좁힌 유효 창**으로 궤적(=기체 중심)을 통과시킨다. `gates.yaml` 의 `gate:` 에서 조절:

| 값               | 기본  | 뜻                                                    |
| ---------------- | ----- | ----------------------------------------------------- |
| `inner_size`     | 0.5   | 게이트 개구부 한 변 [m] (물리 크기)                   |
| `drone_radius`   | 0.06  | 크플 반경(프로펠러 끝까지) [m]. CF2.1 ≈ 0.06          |
| `frame_margin`   | 0.10  | 프레임 안쪽 ~ 프로펠러 끝 최소 여유 [m]               |

```
유효 창 반폭 = inner_size/2 − (drone_radius + frame_margin)
             = 0.25 − (0.06 + 0.10) = 0.09 m   → 유효 창 0.18 m
```

궤적 중심이 게이트 중앙 ±0.09 m 안으로 지나고, **프로펠러 끝이 프레임에서 0.10 m**
뜬다. 더 타이트하게(가운데로) 가려면 `frame_margin` 을 키우고, 더 빠르게(여유 있게)
가려면 줄인다. `--dry-run`·`plan_gate_trajectory` 출력의 `프레임 여유` 검사가 게이트별
프로펠러-프레임 여유를 찍어 준다. **rviz 의 주황 사각형**이 이 유효 창이다(초록 프레임
안쪽). 값을 바꾸면 `plan_gate_trajectory` 로 궤적을 다시 만들어야 반영된다.

#### TOGT 옵션 튜닝 (`togt_tools/params/`)

속도·민첩성·부드러움은 아래 파일로 조절한다. 바꾼 뒤 `plan_gate_trajectory` 재실행.

- **`params/cf_quad.yaml`** — 기체 모델: `mass`(0.033), `inertia`, `thrust_max`(추력
  상한 [m/s²]), `omega_max`(각속도 [rad/s]). 실제 기체 스펙에 맞춘다.
- **`params/{init,refine}/cf_planning.yaml`** — 계획 제약:
  - `maxVelNorm` — **최대 속도** [m/s]. 실내에선 보수적으로(기본 2.0).
  - `maxOmgXY` / `maxOmgZ` — 최대 각속도 [rad/s]. 클수록 민첩(코너를 빠르게).
  - `maxTiltedAngle` — 최대 기울기 [rad]. 클수록 공격적.
  - `maxThr` / `minThr` — 로터 추력 한계 [N].
  - `boundX/Y/Z` — 방 경계 [m]. 벽 여유를 정한다.
  - `refine/cf_planning.yaml` 의 **`piecesPerSegment`** — 세그먼트당 조각 수.
    세그먼트 8개 × 이 값 = 총 조각. **펌웨어 궤적 메모리(~32) 이하**로 유지(기본 3 → 24).
    키우면 궤적이 부드럽지만 조각이 늘고, 32 를 넘으면 업로드가 실패한다.
- **`params/cf_setups.yaml`** — 위 파일들을 묶는 진입점(보통 그대로 둔다).

계획 결과는 `TrajExtremum`(maxVel/maxAcc/maxOmg/최대 틸트)과 조각 수·비행 시간으로
출력되니, 실기체 한계를 넘지 않는지 확인하고 날린다.

#### rviz 시각화

`gate_markers` 는 gate_flight 가 실제로 실행하는 **같은 궤적 CSV** 를 샘플해 그린다
(→ 보이는 경로 = 나는 경로). 표시:

- 방 경계 · 게이트 프레임(초록, 라벨 `G1`..`G7`) · **유효 통과 창(주황 = TOGT 목표 마진)**
  · 통과 방향 화살표 · 이착륙 지점(`TAKEOFF`)
- **계획 궤적**(`/gate_course/path`, 하늘색) — 날기 전에 미리 보인다.
- **실제 비행 자취**(`/gate_course/flown`, 노랑) — 드론 TF 를 누적해 그린다. 비행 중
  노란 선이 하늘색 계획 위에 겹쳐 그려지므로, 계획 대비 실제 추종을 눈으로 비교할 수
  있다. sim 재시작이나 재배치처럼 위치가 크게 튀면 자취는 자동으로 초기화된다.

다른 궤적을 보려면 `-p trajectory:=...`, 기체 이름이 `cf231` 이 아니면
`-p robot_frame:=<이름>`, 자취를 끄려면 `-p show_trail:=false`.

### Blackbird 스타일 주기 궤적 (학습 데이터용)

Blackbird 데이터셋의 설계 철학(**주기성 · 속도 격리 · yaw 모드**)을 따라 5개 도형을
코드로 정의하고, 각 도형을 **지정 랩 수만큼 연속으로** 비행한다. 목적은 sim 에서
imu·pose·pwm 을 기록해 **ML 학습 데이터**를 만드는 것.

도형: `circle` · `oval` · `figure8` · `clover`(네잎, 약한 3D) · `star`(5각). 앞 넷은
파라메트릭 수식(해석적 도함수), star 는 주기 스플라인. 코드: `crazyflie_test/traj/`.

#### 비행 (도형별 진입점)

```bash
ros2 run crazyflie_test circle  --laps 5 --speed 1.5 --yaw forward
ros2 run crazyflie_test clover  --laps 3 --speed 2.0
ros2 run crazyflie_test star    --laps 3 --speed 1.0 --yaw constant
# figure8 / oval 도 동일
```

| 옵션        | 기본     | 설명                                              |
| ----------- | -------- | ------------------------------------------------- |
| `--laps`    | 3        | 바퀴 수 (연속 비행)                               |
| `--speed`   | 1.0      | 목표 **최대** 속도 [m/s]. 도형 고정, period 만 조절 |
| `--yaw`     | forward  | `forward`=진행 방향, `constant`=고정(0)           |
| `--height`  | 1.0      | 비행 고도 [m]                                     |
| `--scale`   | 1.0      | 도형 수평 크기 배율                               |
| `--rate`    | 50       | cmdFullState 스트리밍 [Hz]                        |
| `--ramp`    | 2.0      | 시작/종료 가감속 [s] (호버에서 매끄럽게 진입·이탈) |
| `--dry-run` | -        | 계획만 출력(실현 속도/가속도/bbox), 비행 안 함    |

**속도 격리**: 같은 도형을 그대로 두고 `period = max|dp/ds| / speed` 만 바꿔 목표 속도를
맞춘다(속도만 독립 변수로 실험 가능). 시작·종료만 램프시켜 t=0 에서 속도·가속도가 0이라
호버에서 이어붙여도 충격이 없다. `cmdFullState` 스트리밍이므로 상태추정이 튼튼해야 한다.

#### sim IMU/PWM 확장 (⚠ 서브모듈 패치)

sim(`crazyflie_sim`)은 원래 `firmware_logging` 을 무시해 **imu_raw/motor_pwm/pose 를
안 낸다**(pose 는 `/tf` 뿐). 학습 데이터를 sim 에서 모으려고 sim 을 확장해 **실기체와
동일 토픽·포맷**으로 내보내게 했다:

- `/<cf>/pose`      — `geometry_msgs/PoseStamped` (sim 은 ground truth)
- `/<cf>/imu_raw`   — `LogDataGeneric.values = [acc.x, acc.y, acc.z(g), gyro.x,y,z(deg/s)]`
- `/<cf>/motor_pwm` — `LogDataGeneric.values = [m1, m2, m3, m4]` (0..65535)

acc 는 SIL 이 안 채우므로 동역학에서 **비추력**(drag-free, 바디 `[0,0,총추력/mass]`,
호버≈+1g z)으로 합성하고, gyro 는 `sensors.gyro`, PWM 은 `motors_thrust_pwm`(컨트롤러가
계산한 값)을 그대로 쓴다. ~100 Hz 로 throttle. sim 파라미터 `sim.log_topics:=false` 로 끈다.

**이 변경은 `crazyswarm2` 서브모듈 내부 수정**이라, 직접 커밋하지 않고 **패치 파일**로
보관한다(`crazyflie_test/patches/`). `git submodule update` 로 원복되면 다시 적용한다.
같은 패치에 게이트 궤적용 `plan_start_trajectory` 시그니처 수정도 함께 들어 있다.

```bash
git submodule update --init --recursive         # sim 서브모듈 받기
crazyflie_test/patches/apply_sim_patch.sh        # 패치 적용 (imu/pwm/pose 발행)
# --revert 로 원복, --check 로 적용 여부 확인. 자세한 내용: crazyflie_test/patches/README.md
```

> **sim vs 실기체 데이터 차이**: sim pose 는 GT(온보드 추정 아님), imu 는 drag-free·무노이즈,
> pwm 은 컨트롤러 출력 그대로. 실기체 imu 는 필터 후 값·노이즈 포함, pose 는 온보드 Kalman
> 추정. 학습 시 도메인 차이를 고려할 것.

#### 데이터 수집

서버가 떠 있는 상태에서 도형×속도를 순회하며 rosbag 기록 → 비행 → (옵션)CSV 변환한다.

```bash
# 서버 (sim). 아무 crazyflies_*.yaml 로 backend:=sim
export PYTHONPATH=~/crazyflie/crazyflie-firmware/build:$PYTHONPATH
ros2 launch crazyflie launch.py backend:=sim teleop:=False mocap:=False \
     crazyflies_yaml_file:=$(ros2 pkg prefix crazyflie_test)/share/crazyflie_test/config/crazyflies_opticalflow.yaml

# 수집 (다른 터미널)
ros2 run crazyflie_test collect_traj_data --shapes clover circle star \
     --speeds 1.0 2.0 --laps 3 --yaw forward --to-csv --out traj_data
```

기록 토픽: `pose · imu_raw · motor_pwm · cmd_full_state`(= 레퍼런스 입력).
출력 폴더(Blackbird 미러): `traj_data/<shape>/<yawType>/<shape>_maxSpeed<V>/{bag, csv}`
(예 `clover/yawForward/clover_maxSpeed2p0/`). `--to-csv` 는 `scripts/bag_to_csv.py` 로
토픽별 CSV 를 만든다(약 100 Hz, 토픽별 timestamp 다르므로 오프라인 리샘플·정렬 필요).

#### 오프라인 CSV/플롯 (비행 없이)

다른 시뮬·분석용으로 **한 랩** 주기 레퍼런스 CSV(스키마
`t,x,y,z,vx,vy,vz,ax,ay,az,yaw,yawrate`)와 검증 플롯을 만든다:

```bash
ros2 run crazyflie_test traj_gen --all --speed 2.0 --yaw forward --plot --out traj_ref
# 또는: python3 -m crazyflie_test.traj.generator --shape clover --speed 1.5
```

### 추락 후 복구

기체가 추락하면 High-Level Commander Lock이 걸려 `takeoff` 등 명령을 보내도 **반응하지 않는다**
(서버 로그에는 명령이 정상 발행된 것처럼 찍히므로 헷갈리기 쉽다). 이때는 재부팅이 필요하다.

```bash
ros2 run crazyflie reboot --uri radio://0/80/2M/E7E7E7E7E7
```

> ⚠️ 라디오를 **직접** 여는 명령이라 crazyflie_server 가 떠 있으면 동글 충돌(busy)이 난다.
> launch 를 먼저 Ctrl+C 로 끈 뒤 실행할 것.

## bag 데이터 변환

```bash
python3 crazyflie_test/scripts/bag_to_csv.py ~/flight_logs/<bag_dir> ~/flight_logs/<out_dir>
```

토픽별 타임스탬프가 다르므로, 시계열 정렬이 필요하면 CSV 로드 후 pandas 등으로
`timestamp_ns` 기준 리샘플링/보간할 것.

## 기록되는 데이터

| 토픽               | 내용                               | 단위 / 비고                                      |
| ------------------ | ---------------------------------- | ------------------------------------------------ |
| `/poses`           | **mocap 원본 pose = ground truth** | `NamedPoseArray`. mocap 모드에서만 존재          |
| `/cf231/pose`      | 드론 온보드 Kalman **추정값**      | GT 아님. IMU 융합 + 라디오 왕복 지연 포함        |
| `/cf231/imu_raw`   | `acc.x/y/z`, `gyro.x/y/z`          | acc=g, gyro=deg/s. 완전 raw 아님(**필터 후** 값) |
| `/cf231/motor_pwm` | `motor.m1~m4`                      | **PWM(0~65535)**, 뉴턴 추력 아님                 |

- **GT 는 `/poses`, 추정값은 `/cf231/pose`** 로 서로 다른 값이다. 둘을 같이 기록해 두면
  온보드 추정 성능(오차·지연)을 GT 대비로 평가할 수 있다.
- `/poses` 는 `NamedPoseArray`(모든 강체를 이름과 함께 담음) 라, CSV 로는
  `poses.0.name`, `poses.0.pose.position.x` … 형태로 펼쳐진다.
- 모터 PWM → 추력(N) 변환은 이 패키지 범위 밖. PWM 을 기록해 두고 오프라인에서
  Bitcraze PWM→thrust 곡선 또는 자체 캘리브레이션으로 변환.
- 펌웨어 로그 패킷 크기 제한 때문에 imu_raw(6개)/motor_pwm(4개)로 토픽을 나눠 둠. 유지할 것.
