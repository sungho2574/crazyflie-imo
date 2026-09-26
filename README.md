# crazyflie-imo

Crazyswarm2 기반 Crazyflie 비행 레포. 목적별로 ROS 2 패키지 4개로 나뉜다.

| 패키지             | 용도                                                          | launch                                   |
| ------------------ | ------------------------------------------------------------- | ---------------------------------------- |
| `crazyflie_test`   | 기본 비행 예제, 기체·mocap 설정, sim 패치, bag→CSV 변환        | `crazyflie_test launch.py`               |
| `crazyflie_traj`   | Blackbird 스타일 주기 궤적 연속 비행 · 학습 데이터 수집        | (`crazyflie_test` launch 사용)           |
| `crazyflie_racing` | 게이트 코스: 게이트 맵 → TOGT 시간최적 궤적 → 온보드 궤적 비행 | `crazyflie_racing launch.py`             |
| `crazyflie_rl`     | 강화학습 레이싱 정책 실행: 무선 상태 → 정책 → 각속도·추력 명령  | `crazyflie_rl launch.py`                 |

- 단일 / 군집, Flow deck(opticalflow) / mocap(Qualisys), sim / 실기체 모두 지원
- IMU raw · 모터 PWM · pose 를 rosbag 으로 기록 (sim 도 패치로 같은 토픽 발행)

## 목차

- [레포 구조](#레포-구조)
- [설치 · 빌드](#설치--빌드)
- [서버 띄우기 (launch)](#서버-띄우기-launch)
- [crazyflie_test — 기본 예제](#crazyflie_test--기본-예제)
- [crazyflie_traj — 주기 궤적 · 학습 데이터](#crazyflie_traj--주기-궤적--학습-데이터)
- [crazyflie_racing — 게이트 코스](#crazyflie_racing--게이트-코스)
- [crazyflie_rl — 강화학습 레이싱 정책](#crazyflie_rl--강화학습-레이싱-정책)
- [sim IMU/PWM 확장 (서브모듈 패치)](#sim-imupwm-확장-서브모듈-패치)
- [기록 데이터 · bag 변환](#기록-데이터--bag-변환)
- [analysis — 오프라인 분석](#analysis--오프라인-분석)
- [문제 해결](#문제-해결)

## 레포 구조

```
ros2_ws/src/crazyflie-imo
├── crazyswarm2/                    # [서브모듈] Crazyflie ROS 2 스택 (서버·드라이버·sim)
├── motion_capture_tracking/        # [서브모듈] 모션 캡쳐(QTM) 패키지
│
├── crazyflie_test/                 # 기본 예제 + 공용 설정
│   ├── crazyflie_test/
│   │   ├── hello_world.py          #   이륙 → 호버 → 착륙
│   │   ├── goto_square.py          #   사각형 goTo
│   │   └── multi_square.py         #   편대 3기체 사각형
│   ├── config/
│   │   ├── crazyflies_opticalflow.yaml        # Flow deck 단일 (기본)
│   │   ├── crazyflies_opticalflow_multi.yaml  # Flow deck 편대
│   │   ├── crazyflies_mocap.yaml              # mocap 기체 설정
│   │   └── motion_capture.yaml                # QTM 연결 설정
│   ├── launch/launch.py            # mode·backend 로 설정 선택 → crazyflie 서버
│   ├── scripts/bag_to_csv.py       # rosbag → 토픽별 CSV
│   └── patches/                    # crazyswarm2 sim 확장 패치 (imu/pwm/pose 발행)
│
├── crazyflie_traj/                 # 주기 궤적 · 학습 데이터
│   ├── crazyflie_traj/
│   │   ├── shapes.py               #   도형 정의 (circle/oval/figure8/clover/star)
│   │   ├── flight.py               #   연속-랩 cmdFullState 비행 로직
│   │   ├── viz.py                  #   비행 중 rviz 계획/실궤적 발행 + rviz 자동 실행
│   │   ├── entry.py                #   도형별 실행 진입점
│   │   ├── collect_data.py         #   도형×속도 순회 rosbag 수집
│   │   └── generator.py            #   오프라인 레퍼런스 CSV·플롯
│   └── config/traj.rviz
│
├── crazyflie_racing/               # 게이트 코스
│   ├── crazyflie_racing/
│   │   ├── gate_course.py          #   gates.yaml·궤적 CSV 로딩, 검사 유틸
│   │   ├── plan_gate_trajectory.py #   gates.yaml → TOGT → gate_trajectory.csv
│   │   ├── gate_flight.py          #   궤적 업로드 + 온보드 추종 비행
│   │   └── gate_markers.py         #   rviz 게이트·계획·실궤적 마커
│   ├── config/
│   │   ├── gates.yaml              # 게이트 배치·방·이착륙 지점
│   │   ├── gate_trajectory.csv     # TOGT 결과 1바퀴 (커밋돼 있음)
│   │   ├── gate_loop_{entry,lap,exit}.csv  # 연속 비행용: 진입 · 순환 1랩 · 탈출
│   │   ├── crazyflies_gate.yaml    # initial_position = start 로 맞춘 기체 설정
│   │   └── gate_course.rviz
│   ├── launch/launch.py            # 서버 + gate_markers + rviz. mode:=gate(기본) | mocap
│   └── togt_tools/                 # TOGT 오프라인 도구 (colcon 빌드 대상 아님)
│       ├── TOGT-Planner/           #   [서브모듈] FSC-Lab TOGT-Planner
│       ├── togt_plan.cpp, CMakeLists.txt
│       └── params/                 #   크플 동역학·계획 파라미터
│
├── crazyflie_rl/                   # 강화학습 레이싱 정책 실행
│   ├── crazyflie_rl/
│   │   ├── policy.py               #   체크포인트(msgpack) → numpy MLP 추론, 행동 → 명령
│   │   ├── racing_obs.py           #   관측 49 생성 + 게이트 진행 추적 (학습 racing.py 이식)
│   │   ├── rl_flight.py            #   실기체 루프: 무선 로그 → 정책 → cmd_vel_legacy
│   │   └── sim_check.py            #   간이 시뮬레이션으로 정책·변환 점검 (기체 불필요)
│   ├── config/crazyflies_rl.yaml   # 모캡 + PID + RPYT rate 모드 + 정책용 100 Hz 로그
│   ├── launch/launch.py            # 서버 + gate_markers + rviz
│   └── models/                     # 정책별 가중치(best.msgpack)·학습 설정·평가 결과
│       ├── racing-body-rate-10s-dr-final/   #   기본: G7 10 s, 도메인 랜덤화
│       ├── racing-body-rate-5s-dr-final/    #   G7 5 s, 도메인 랜덤화 (공격적)
│       └── racing-body-rate-10s-final/      #   G7 10 s, 랜덤화 없음 (비교용)
│
└── analysis/                       # 수집 데이터 오프라인 분석 스크립트 (ROS 패키지 아님)
```

패키지 의존: `crazyflie_traj`·`crazyflie_racing` → `crazyflie_test`
(기체/mocap 설정과 `bag_to_csv.py` 를 공유). `crazyflie_rl` → `crazyflie_racing`(게이트 맵),
`crazyflie_test`(모캡 설정·기록기).

## 설치 · 빌드

실행 환경: Ubuntu 22.04 + ROS 2 Humble, 또는 Ubuntu 24.04 + ROS 2 Jazzy.

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/sungho2574/crazyflie-imo.git
# 이미 --recursive 없이 클론했다면
git submodule update --init --recursive
```

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # 최초 1회 crazyswarm2 의존성 설치
colcon build --symlink-install
source install/setup.bash
```

**sim 을 쓸 때** 추가로:

```bash
# crazyflie_sim 은 cffirmware 파이썬 바인딩이 필요
export PYTHONPATH=~/crazyflie/crazyflie-firmware/build:$PYTHONPATH
# sim 이 imu_raw/motor_pwm/pose 를 내도록 패치 (학습 데이터·게이트 sim 에 필요)
crazyflie_test/patches/apply_sim_patch.sh
```

> 패키지 구조가 바뀐 뒤 처음 빌드한다면 `build/crazyflie_test`, `install/crazyflie_test`
> 를 지우고 빌드한다(옛 실행 명령이 남지 않게).

## 서버 띄우기 (launch)

모든 비행 스크립트는 **crazyflie 서버가 떠 있는 상태**에서 별도 터미널로 실행한다.
서버는 한 번 띄워 두고 스크립트를 여러 번 돌려도 된다.

### `crazyflie_test` launch — 기본 예제 · 주기 궤적

```bash
ros2 launch crazyflie_test launch.py                          # opticalflow + cflib (기본)
ros2 launch crazyflie_test launch.py mode:=opticalflow_multi  # Flow deck 편대
ros2 launch crazyflie_test launch.py mode:=mocap              # mocap (QTM 세팅 필요)
ros2 launch crazyflie_test launch.py backend:=sim             # 하드웨어 없이 sim
```

| 인자      | 값                                             | 기본          |
| --------- | ---------------------------------------------- | ------------- |
| `mode`    | `opticalflow` \| `opticalflow_multi` \| `mocap` | `opticalflow` |
| `backend` | `cflib` \| `cpp` \| `sim`                       | `cflib`       |

`mode` 값으로 `config/crazyflies_<mode>.yaml` 을 고르고, `mocap` 일 때만
motion_capture_tracking 노드를 켠다.

### `crazyflie_racing` launch — 게이트 코스

```bash
ros2 launch crazyflie_racing launch.py backend:=sim   # sim (mode:=gate 기본)
ros2 launch crazyflie_racing launch.py                # Flow deck 실기체
ros2 launch crazyflie_racing launch.py mode:=mocap    # mocap 실기체
```

| 인자         | 값                         | 기본    | 설명                                         |
| ------------ | -------------------------- | ------- | -------------------------------------------- |
| `mode`       | `gate` \| `mocap`          | `gate`  | 기체 설정 선택 (아래)                        |
| `backend`    | `cflib` \| `cpp` \| `sim`  | `cflib` |                                              |
| `markers`    | `true` \| `false`          | `true`  | `gate_markers` 노드 같이 실행                |
| `rviz`       | `true` \| `false`          | `true`  | `gate_course.rviz` 로 rviz2 같이 실행        |
| `trajectory` | CSV 경로                   | (빈 값) | 마커로 그릴 궤적. 비우면 `config/gate_trajectory.csv` |
| `loop`       | `true` \| `false`          | `false` | 연속 비행 궤적(`gate_loop_*.csv`)을 그림 (`gate_flight --loop` 용) |

서버 + 게이트·궤적 마커 + rviz 가 한 번에 뜬다. 비행(`gate_flight`)만 별도 터미널에서 실행한다.

- `gate` — `crazyflie_racing/config/crazyflies_gate.yaml` (`initial_position` = gates.yaml `start`)
- `mocap` — `crazyflie_test` 의 `crazyflies_mocap.yaml` · `motion_capture.yaml` 을 그대로 사용

## crazyflie_test — 기본 예제

### 빠른 시작 (sim + rviz2)

```bash
# T1 — sim 서버
ros2 launch crazyflie_test launch.py backend:=sim
# T2 — 예제
ros2 run crazyflie_test hello_world
# T3 — 시각화: Global Options → Fixed Frame → world, Add → TF
rviz2
```

### 예제 목록

```bash
ros2 run crazyflie_test hello_world    # 이륙 → 호버 → 착륙
ros2 run crazyflie_test goto_square    # 사각형 goTo
ros2 run crazyflie_test multi_square   # 편대 3기체 (mode:=opticalflow_multi)
```

### 비행 기록

비행 직전 별도 터미널에서 시작한다. 멀티는 기체별 토픽으로 바꾼다(`/cf1/...`, `/cf2/...`).

```bash
# /poses       = mocap 원본 = ground truth (mocap 모드에서만 존재)
# /cf231/pose  = 드론 온보드 추정값 (GT 와 비교용)
ros2 bag record /poses /cf231/pose /cf231/imu_raw /cf231/motor_pwm \
  -o ~/flight_logs/$(date +%Y%m%d_%H%M%S)
```

토픽 설명과 CSV 변환은 [기록 데이터 · bag 변환](#기록-데이터--bag-변환) 참고.

## crazyflie_traj — 주기 궤적 · 학습 데이터

Blackbird 데이터셋의 설계 철학(**주기성 · 속도 격리 · yaw 모드**)을 따라 5개 도형을
코드로 정의하고, 각 도형을 **지정 랩 수만큼 연속으로** 비행한다. 목적은 imu·pose·pwm
을 기록해 **ML 학습 데이터**를 만드는 것.

도형: `circle` · `oval` · `figure8` · `clover`(네잎, 약한 3D) · `star`(5각). 앞 넷은
파라메트릭 수식(해석적 도함수), star 는 주기 스플라인.

| 실행 명령                               | 역할                                    |
| --------------------------------------- | --------------------------------------- |
| `circle` `oval` `figure8` `clover` `star` | 도형 연속-랩 비행                     |
| `collect_traj_data`                     | 도형×속도 순회하며 rosbag 수집          |
| `traj_gen`                              | 오프라인 레퍼런스 CSV·플롯 (비행 없음)  |

### 비행 (터미널 2개)

```bash
# T1 — 서버
ros2 launch crazyflie_test launch.py backend:=sim
```

```bash
# T2 — 비행. rviz 가 자동으로 뜨고 계획 궤적·실궤적을 보여 준다
ros2 run crazyflie_traj clover  --laps 3 --speed 2.0
ros2 run crazyflie_traj circle  --laps 5 --speed 1.5 --yaw forward
ros2 run crazyflie_traj star    --laps 3 --speed 1.0 --yaw constant
ros2 run crazyflie_traj figure8 --dry-run          # 계획만 확인
```

| 옵션            | 기본    | 설명                                                 |
| --------------- | ------- | ---------------------------------------------------- |
| `--laps`        | 3       | 바퀴 수 (연속 비행)                                  |
| `--speed`       | 1.0     | 목표 **최대** 속도 [m/s]. 도형 고정, period 만 조절  |
| `--yaw`         | forward | `forward`=진행 방향, `constant`=고정(0)              |
| `--height`      | 1.0     | 비행 고도 [m]                                        |
| `--scale`       | 1.0     | 도형 수평 크기 배율                                  |
| `--rate`        | 50      | cmdFullState 스트리밍 [Hz]                           |
| `--ramp`        | 2.0     | 시작/종료 가감속 [s] (호버에서 매끄럽게 진입·이탈)   |
| `--min-battery` | 3.7     | 이 전압 미만이면 이륙 안 함 [V]                      |
| `--no-arm`      | -       | arm 요청 안 함                                       |
| `--no-rviz`     | -       | rviz2 자동 실행 안 함 (토픽은 그대로 발행)           |
| `--record`      | -       | 이륙 직전~착륙 rosbag 자동 기록 (아래 참고)          |
| `--record-dir`  | `~/flight_logs/traj_data` | 기록 루트 폴더                     |
| `--dry-run`     | -       | 계획만 출력(실현 속도/가속도/bbox), 비행 안 함       |

- **기록(`--record`)**: `pose · imu_raw · motor_pwm · cmd_full_state · status · /poses`(모캡 GT)
  를 `<record-dir>/<shape>/<yawType>/<shape>_maxSpeed<V>/bag_<날짜_시각>/` 에 sqlite3(`.db3`)로
  저장한다. `collect_traj_data` 와 같은 구조라 `analysis/visualize_flights.py` 가 그대로 읽는다.
  여러 도형·속도를 순회하며 모으려면 `collect_traj_data` 를 쓴다.
- **속도 격리**: 같은 도형을 그대로 두고 `period = max|dp/ds| / speed` 만 바꿔 목표 속도를
  맞춘다(속도만 독립 변수로 실험 가능).
- 시작·종료만 램프시켜 t=0 에서 속도·가속도가 0이라 호버에서 이어붙여도 충격이 없다.
- ⚠️ `cmdFullState` 는 **low-level 제어**라 high-level commander 를 우회한다. 상태추정이
  튼튼해야 하므로 **mocap 권장**, opticalflow 라면 `--speed` 를 낮게 잡을 것.

### rviz (계획 vs 실궤적)

**비행 명령이 알아서 띄운다.** 이륙 전에 rviz2(`traj.rviz`)를 실행하고 두 선을 발행한다.

- 🔵 `/traj/path` (하늘색) — **계획 궤적**: 이번 비행의 도형 한 랩
- 🟡 `/traj/flown` (노랑) — **실궤적**: 드론 `/tf`(world→기체) 누적 실제 자취

- 계획선은 비행과 **같은 도형·`--scale`·`--height`·이륙 위치**로 그리므로 맞출 게 없다.
- rviz 가 이미 `traj.rviz` 로 떠 있으면 새로 띄우지 않고, 비행이 끝나도 닫지 않는다.
  다음 비행을 시작하면 계획선이 그 도형으로 바뀌고 자취는 새로 쌓인다.
- 원격 접속 등으로 rviz 가 필요 없으면 `--no-rviz`.

비행 없이 도형만 확인하려면 `--dry-run`(수치) 또는 `traj_gen --plot`(그림)을 쓴다.

### 데이터 수집

서버가 떠 있는 상태에서 도형×속도를 순회하며 rosbag 기록 → 비행 → (옵션)CSV 변환한다.

```bash
# 서버 (sim)
ros2 launch crazyflie_test launch.py backend:=sim

# 수집 (다른 터미널). sim 은 mocap GT 가 없으므로 --gt-topic ''
ros2 run crazyflie_traj collect_traj_data --shapes clover circle star \
     --speeds 1.0 2.0 --laps 3 --yaw forward --to-csv --out traj_data --gt-topic ''
```

| 옵션         | 기본                  | 설명                                                      |
| ------------ | --------------------- | --------------------------------------------------------- |
| `--shapes`   | 5개 전부              | 수집할 도형들                                             |
| `--speeds`   | 1.0                   | 목표 최대 속도 목록 [m/s]                                 |
| `--laps`     | 3                     | 런당 바퀴 수 (`--seconds` 없을 때)                        |
| `--seconds`  | -                     | 런당 목표 비행 시간 [s]. 주면 도형·속도별 laps 자동 계산  |
| `--yaw`      | forward               | `forward` \| `constant`                                   |
| `--cf`       | cf231                 | 기체 이름(토픽 네임스페이스)                              |
| `--out`      | traj_data             | 출력 루트 폴더                                            |
| `--to-csv`   | -                     | `crazyflie_test/scripts/bag_to_csv.py` 로 토픽별 CSV 변환 |
| `--gt-topic` | `/poses`              | mocap GT 토픽. `''` 이면 미기록                           |
| `--height` / `--scale` | -           | 비행에 그대로 전달                                        |

- 기록 토픽: `pose · imu_raw · motor_pwm · cmd_full_state`(= 레퍼런스 입력) `· status`
  (배터리·supervisor) `· /poses`(mocap GT).
- 도형마다 비행 명령을 실행하므로 rviz 계획선도 **도형이 바뀔 때마다 자동으로 바뀐다.**
- 비행 중 **각 랩마다** `[<도형> <속도>m/s] 랩 k/N  배터리 x.xx V` 가 찍힌다
  (실기체 `status` 필요, sim 은 배터리 `N/A`).
- 출력 폴더(Blackbird 미러): `traj_data/<shape>/<yawType>/<shape>_maxSpeed<V>/{bag, csv}`
  (예 `clover/yawForward/clover_maxSpeed2p0/`). bag 은 sqlite3(`.db3`)로 저장한다(Jazzy 기본
  mcap 대신 — `analysis/` 스크립트가 `.db3` 를 읽는다). CSV 는 약 100 Hz, 토픽별 timestamp 가
  달라 오프라인 리샘플·정렬이 필요하다.
- sim 에서 imu_raw/motor_pwm/pose 를 받으려면 [sim 패치](#sim-imupwm-확장-서브모듈-패치)가 필요하다.

### 오프라인 CSV/플롯 (비행 없이)

한 랩 주기 레퍼런스 CSV(스키마 `t,x,y,z,vx,vy,vz,ax,ay,az,yaw,yawrate`)와 검증 플롯:

```bash
ros2 run crazyflie_traj traj_gen --all --speed 2.0 --yaw forward --plot --out traj_ref
# 또는: python3 -m crazyflie_traj.generator --shape clover --speed 1.5
```

옵션: `--shape`/`--all`, `--speed`(1.0), `--yaw`(forward), `--rate`(100 Hz), `--out`(traj_ref), `--plot`.

## crazyflie_racing — 게이트 코스

`crazyflie_racing/config/gates.yaml` 에 적힌 게이트를 **번호 순으로 전부 통과**한다.
경로는 **TOGT-Planner(FSC-Lab) 로 오프라인에서 시간최적 궤적**을 만들어 다항식
CSV(`config/gate_trajectory.csv`)로 저장해 두고, 비행은 **크플 펌웨어의 온보드
궤적 추종기**(High-Level Commander: `uploadTrajectory` + `startTrajectory`)가
실행한다. 펌웨어가 온보드에서 궤적을 추종하므로 통신이 끊겨도 계속 따라간다.

    gates.yaml ─(오프라인: TOGT)─► gate_trajectory.csv ─► 펌웨어 uploadTrajectory ─► 비행

동작: `start` 좌표에서 이륙 → 게이트 1..N 통과 → `start` 상공 복귀 → 착륙.

| 실행 명령              | 역할                                              |
| ---------------------- | ------------------------------------------------- |
| `plan_gate_trajectory` | gates.yaml → TOGT → `gate_trajectory.csv`         |
| `gate_flight`          | 사전 점검 → 궤적 업로드 → 비행 (`--dry-run` 가능) |
| `gate_markers`         | rviz 게이트·계획 궤적·실궤적                      |

### 1. 궤적 계획 (게이트를 바꿨을 때만)

기본 `gates.yaml` 로 만든 `gate_trajectory.csv` 가 이미 커밋돼 있어 **평소 비행에는
필요 없다.** TOGT-Planner 는 `crazyflie_racing/togt_tools/TOGT-Planner/` 서브모듈이다.

```bash
git submodule update --init --recursive     # TOGT-Planner 받기 (최초 1회)
ros2 run crazyflie_racing plan_gate_trajectory
# 또는: python3 -m crazyflie_racing.plan_gate_trajectory
```

| 옵션         | 기본                         | 설명                                   |
| ------------ | ---------------------------- | -------------------------------------- |
| `--gates`    | 패키지 `config/gates.yaml`   | 입력 게이트 맵                         |
| `--out`      | 패키지 `config/gate_trajectory.csv` | 출력 CSV                        |
| `--togt-dir` | 서브모듈 (`$TOGT_DIR`)       | 다른 TOGT-Planner 소스 경로            |

`togt_plan` 은 처음 실행 때 서브모듈에서 자동 빌드된다. 출력에서 **조각 수 ≤ 32**
(펌웨어 궤적 메모리 한계), 방 이탈 0, 게이트 프레임 간섭 없음을 확인한다.

### 2. 계획 확인 (기체 불필요)

```bash
ros2 run crazyflie_racing gate_flight --dry-run
```

통과 순서·궤적 시간·조각 수·방 경계·게이트 프레임 여유를 출력한다.

### 3. 시뮬레이션 (터미널 2개)

launch 가 sim 서버 · `gate_markers` · rviz 를 함께 띄운다.

```bash
# T1 — sim 서버 + 게이트·궤적 마커 + rviz (crazyflies_gate.yaml 자동 선택)
export PYTHONPATH=~/crazyflie/crazyflie-firmware/build:$PYTHONPATH
ros2 launch crazyflie_racing launch.py backend:=sim
```
```bash
# T2 — 비행
ros2 run crazyflie_racing gate_flight
```

⚠️ launch 가 이미 `gate_markers` 를 띄우므로 따로 `ros2 run ... gate_markers` 를 또 켜지
않는다. 두 개가 뜨면 rviz 경로가 깜빡인다([문제 해결](#문제-해결)). 다른 궤적을 보려면
`trajectory:=/path/to.csv`, 마커나 rviz 가 필요 없으면 `markers:=false` / `rviz:=false`.

### 4. 실기체

- **모캡(Qualisys, 권장)** — 트래커가 절대 위치를 준다. `initial_position=start` 검사는
  `--mocap` 으로 건너뛴다. ⚠️ **QTM 캘리브레이션 원점을 방 중심(gates.yaml 원점)에
  놓고 x/y 축을 gates.yaml 과 정렬**해야 한다. 기체는 볼륨 안 아무 데나 놓아도 된다.

  ```bash
  ros2 launch crazyflie_racing launch.py mode:=mocap
  ros2 run crazyflie_racing gate_flight --mocap --timescale 2   # 첫 비행은 느리게
  ```

- **Flow deck(외부 트래커 없음)** — 온보드 추정이라 원점 = `initial_position`.
  기체를 `start` 좌표에 기수 +x 로 놓고 `crazyflies_gate.yaml` 의 `initial_position`
  (= start)·`uri` 를 맞춘다.

  ```bash
  ros2 launch crazyflie_racing launch.py
  ros2 run crazyflie_racing gate_flight --timescale 2
  ```

`gate_flight` 는 이륙 전에 `initial_position` 일치·배터리·tumble·lock 을 점검하고
문제가 있으면 **이륙하지 않는다**. 익숙해지면 `--timescale 1` 로 정상 속도.

| 옵션            | 기본  | 설명                                            |
| --------------- | ----- | ----------------------------------------------- |
| `--timescale`   | 1.0   | 궤적 시간 배율. >1 이면 느리게 (첫 비행 권장)    |
| `--trajectory`  | -     | 궤적 CSV (기본 `config/gate_trajectory.csv`)    |
| `--laps`        | 1     | 같은 궤적을 반복할 바퀴 수 (아래 참고)          |
| `--lap-pause`   | 1.0   | 바퀴 사이 start 상공 재정렬 시간 [s]            |
| `--loop`        | 0     | 멈추지 않고 연속으로 도는 바퀴 수 (아래 참고)   |
| `--record`      | -     | 이륙 직전~착륙 rosbag 자동 기록 (아래 참고)     |
| `--record-dir`  | `~/flight_logs` | 기록 루트 폴더                        |
| `--gates`       | -     | gates.yaml (기본 패키지 config)                 |
| `--height`      | yaml  | 이륙 고도 [m]. 기본은 `start.takeoff_z`         |
| `--min-battery` | 3.85  | 이 전압 미만이면 이륙 안 함 [V]                  |
| `--mocap`       | -     | 모캡으로 절대 위치를 받는 경우                   |
| `--force-start` | -     | initial_position 이 start 와 달라도 강행        |
| `--dry-run`     | -     | 계획만 출력하고 비행하지 않음                    |

#### 여러 바퀴 비행 — 명령 정리

| 방식                     | 옵션        | 궤적                                   | 바퀴 사이        | rviz 서버 인자  |
| ------------------------ | ----------- | -------------------------------------- | ---------------- | --------------- |
| 1바퀴 궤적 반복          | `--laps N`  | `gate_trajectory.csv`                  | start 에서 약 1.5 s 정지 | (기본)   |
| 연속 궤적                | `--loop N`  | `gate_loop_{entry,lap,exit}.csv`       | 멈추지 않음      | `loop:=true`    |

**1바퀴 궤적 반복 (`--laps`)**

```bash
# T1 — 서버 (rviz 에 1바퀴 궤적)
ros2 launch crazyflie_racing launch.py mode:=mocap
```
```bash
# T2 — 계획 확인 → 비행
ros2 run crazyflie_racing gate_flight --mocap --laps 3 --dry-run
ros2 run crazyflie_racing gate_flight --mocap --laps 3 --timescale 2
```

**연속 궤적 (`--loop`)**

```bash
# T1 — 서버 (rviz 에 진입+순환+탈출 궤적)
ros2 launch crazyflie_racing launch.py mode:=mocap loop:=true
```
```bash
# T2 — 이음매·총 시간 확인 → 비행
ros2 run crazyflie_racing gate_flight --mocap --loop 3 --dry-run
ros2 run crazyflie_racing gate_flight --mocap --loop 3 --timescale 2
```

- `--laps` 와 `--loop` 는 같이 쓸 수 없다.
- 둘 다 `--record` 를 붙이면 `~/flight_logs/gate_<시각>/` 에 rosbag 으로 기록된다.
- 첫 비행은 `--timescale 2`(절반 속도), 익숙해지면 1.
- Flow deck 이면 `mode:=mocap` 과 `--mocap` 을 빼고, 기체를 start 좌표에 정확히 놓는다.

**여러 바퀴(`--laps N`)** — 업로드한 1바퀴 궤적을 N번 다시 실행한다. 궤적이 start 상공에서
속도 0 으로 시작·끝나므로, 바퀴 사이에 start 상공에서 `--lap-pause` 동안 **잠깐 멈춰 위치를
다시 맞춘 뒤** 다음 바퀴를 시작한다(오차가 바퀴마다 누적되지 않게). 멈추지 않고 돌려면
아래 `--loop` 를 쓴다.

```bash
ros2 run crazyflie_racing gate_flight --laps 3 --timescale 2
```

**연속 여러 바퀴(`--loop N`)** — 바퀴 사이에 멈추지 않는다. 궤적 3개를 이어 붙인다:

| 파일                       | 구간                                          |
| -------------------------- | --------------------------------------------- |
| `gate_loop_entry.csv`      | start 호버 → 이음매 (정지에서 출발)           |
| `gate_loop_lap.csv`        | 이음매 → G1..G7 → 이음매 (순환 1랩)           |
| `gate_loop_exit.csv`       | 이음매 → start 호버 (정지로 끝남)             |

이음매((1.5, −1.5, 1.25), +y 로 약 0.78 m/s)에서 세 궤적의 위치·속도·가속도가 같도록
계획돼 있어 `진입 → 랩 × N → 탈출` 로 끊김 없이 이어진다. 세 궤적을 펌웨어 메모리에
함께 올리고(조각 1+16+4=21 ≤ 31), 각 궤적이 끝나는 시각에 호스트가 다음 궤적을
시작한다(절대좌표, `relative=False`). 펌웨어가 궤적을 줄 세우지 못해 전환 명령은
무선으로 가므로, 지연(수~수십 ms)만큼 이음매에서 수 cm 어긋날 수 있다.

```bash
ros2 launch crazyflie_racing launch.py backend:=sim loop:=true   # rviz 에 연속 비행 궤적
ros2 run crazyflie_racing gate_flight --loop 10 --dry-run         # 이음매·총 시간 확인
ros2 run crazyflie_racing gate_flight --loop 10
```

- `--dry-run` 이 이음매 3곳(진입→랩, 랩→랩, 랩→탈출)의 pos/vel/acc 차이를 검사한다.
  궤적 파일을 새로 만들면 여기서 `OK` 인지 먼저 확인할 것.
- `--timescale` 은 세 궤적에 똑같이 적용되므로 이음매 연속성이 유지된다.
- 진입·탈출이 랩과 같은 통로를 쓰므로 `경로가 자기 자신과 … 붙는다` 경고는 정상이다.
- `--laps`(바퀴마다 정지) · `--trajectory` 와는 같이 쓸 수 없다.

**기록(`--record`)** — 이륙 2초 전에 `ros2 bag record` 를 시작해 착륙 후 닫는다(Ctrl+C 로
중단해도 닫힌다). `~/flight_logs/gate_<날짜_시각>/` 에 sqlite3(`.db3`)로 저장하므로
`analysis/gate_3d.py` 가 바로 읽는다. 토픽: `/poses`(모캡 GT, 모캡 모드만) ·
`/<cf>/pose` · `imu_raw` · `motor_pwm` · `status`.

```bash
ros2 run crazyflie_racing gate_flight --mocap --record --laps 3
```

### 위치 출처: 모캡 vs 온보드 추정 (중요)

게이트 좌표(방 중심 = 원점)와 기체가 아는 자기 위치가 **같은 좌표계**여야 한다.

| 방식               | 위치 출처       | world 원점           | 필요한 것                   | launch        |
| ------------------ | --------------- | -------------------- | --------------------------- | ------------- |
| **모캡(Qualisys)** | 트래커 절대좌표 | **QTM 캘리브레이션** | QTM 원점 = 방 중심, 축 정렬 | `mode:=mocap` |
| Flow deck / sim    | 온보드 추정     | `initial_position`   | `initial_position = start`  | `mode:=gate`  |

`gates.yaml` 의 `start` 를 바꾸면 `crazyflies_gate.yaml` 의 `initial_position` 도 같이
바꾼다(다르면 `gate_flight` 가 이륙 거부).

### 좌표·게이트 규약

- `gates.yaml` 의 x, y 는 **world 절대 좌표**, 방 중심이 원점.
- **`yaw_deg` 는 위에서 봤을 때 시계방향(CW)** 기준이다. 진행 방향 =
  `(cos(-yaw_deg), sin(-yaw_deg))` — 게이트 맵 생성기 `gate_map.py` 의 `yaw_rad()`
  와 같다. **`gate_map.py` 헤더의 `yaw_deg ... CCW` 주석은 실제(`-yaw_deg`, CW)와
  반대이니 고칠 것.** CCW 로 읽으면 게이트 방향이 x축 대칭으로 뒤집힌다.
- 이 코스는 게이트가 안전여유선(y=4.0)에 붙어 있어 진입/탈출 구간이 여유 영역을
  조금 벗어난다(방 경계 y=5.5 까지는 넘지 않음). 그래서 `벽까지 여유 NG` 경고가
  뜨는데, 게이트 배치에서 오는 것이다.

### 게이트 통과 마진

TOGT 는 게이트 물리 개구부를 그대로 쓰지 않고 **가운데로 좁힌 유효 창**으로
궤적(=기체 중심)을 통과시킨다. `gates.yaml` 의 `gate:` 에서 조절:

| 값             | 기본 | 뜻                                           |
| -------------- | ---- | -------------------------------------------- |
| `inner_size`   | 0.5  | 게이트 개구부 한 변 [m] (물리 크기)          |
| `drone_radius` | 0.06 | 크플 반경(프로펠러 끝까지) [m]. CF2.1 ≈ 0.06 |
| `frame_margin` | 0.10 | 프레임 안쪽 ~ 프로펠러 끝 최소 여유 [m]      |

```
유효 창 반폭 = inner_size/2 − (drone_radius + frame_margin)
             = 0.25 − (0.06 + 0.10) = 0.09 m   → 유효 창 0.18 m
```

더 타이트하게(가운데로) 가려면 `frame_margin` 을 키우고, 더 빠르게 가려면 줄인다.
rviz 의 **주황 사각형**이 이 유효 창이다. 값을 바꾸면 `plan_gate_trajectory` 를 다시 돌린다.

### TOGT 파라미터 (`togt_tools/params/`)

바꾼 뒤 `plan_gate_trajectory` 재실행.

- **`cf_quad.yaml`** — 기체 모델: `mass`(0.033), `inertia`, `thrust_max`(추력 상한
  [m/s²]), `omega_max`(각속도 [rad/s]).
- **`{init,refine}/cf_planning.yaml`** — 계획 제약:
  - `maxVelNorm` — **최대 속도** [m/s] (기본 2.0)
  - `maxOmgXY` / `maxOmgZ` — 최대 각속도 [rad/s]. 클수록 민첩
  - `maxTiltedAngle` — 최대 기울기 [rad]
  - `maxThr` / `minThr` — 로터 추력 한계 [N]
  - `boundX/Y/Z` — 방 경계 [m]
  - `refine/` 의 **`piecesPerSegment`** — 세그먼트당 조각 수. 세그먼트 8개 × 이 값 =
    총 조각. **펌웨어 한계(~32) 이하** 유지(기본 3 → 24). 넘으면 업로드 실패.
- **`cf_setups.yaml`** — 위 파일들을 묶는 진입점(보통 그대로).

계획 결과의 `TrajExtremum`(maxVel/maxAcc/maxOmg/최대 틸트)·조각 수·비행 시간을 보고
실기체 한계를 넘지 않는지 확인한다.

### rviz (`gate_markers`)

`crazyflie_racing` launch 가 기본으로 함께 띄운다. 단독 실행은
`ros2 run crazyflie_racing gate_markers` (launch 에 `markers:=false` 를 준 경우만).
`gate_flight` 가 실행하는 **같은 궤적 CSV** 를 샘플해 그린다(보이는 경로 = 나는 경로).

- 방 경계 · 게이트 프레임(초록, `G1`..`G7`) · 유효 통과 창(주황) · 통과 방향 화살표 · `TAKEOFF`
- **계획 궤적** `/gate_course/path` (하늘색)
- **실제 자취** `/gate_course/flown` (노랑) — 드론 TF 누적. 위치가 크게 튀면 자동 초기화

| 파라미터      | 기본           | 설명                                         |
| ------------- | -------------- | -------------------------------------------- |
| `gates_yaml`  | 패키지 config  | 게이트 맵                                    |
| `trajectory`  | 패키지 config  | 궤적 CSV                                     |
| `start`       | gates.yaml     | 이착륙 xy                                    |
| `height`      | gates.yaml     | 이륙 고도 (0 이면 `start.takeoff_z`)         |
| `show_path`   | true           | 계획 궤적 표시                               |
| `robot_frame` | `cf231`        | 실궤적 TF 프레임                             |
| `show_trail`  | true           | 실궤적 표시                                  |
| `trail_step`  | 0.02           | 점 추가 간격 [m]                             |
| `trail_max`   | 0              | 자취 최대 점 수 (0=무제한)                   |

## crazyflie_rl — 강화학습 레이싱 정책

게이트 7개를 도는 PPO 정책(body-rate 제어, 학습 목표 G7 10 s)을 실기체에서 돌린다.
PC 가 무선으로 상태를 받아 정책을 계산하고, **각속도 목표 + 추력**을 다시 보낸다.
궤적을 기체에 올리는 `crazyflie_racing` 과 달리 **PC 실시간 제어**라 통신이 끊기면 제어를 잃는다.

    무선 로그 100 Hz (rl_pv · rl_att · rl_gyro)
      → 관측 49 → 정책(MLP 256×2, tanh) → [ωx, ωy, ωz (rad/s), 총추력 (N)]
      → cmd_vel_legacy (RPYT rate 모드) → 펌웨어 PID 각속도 루프

| 실행 명령      | 역할                                                      |
| -------------- | --------------------------------------------------------- |
| `rl_sim_check` | 간이 시뮬레이션으로 정책이 코스를 도는지 점검 (ROS·기체 불필요) |
| `rl_flight`    | 실기체 비행 (`--shadow` 면 명령 없이 관측·정책 출력만 기록) |

### 모델

학습 산출물의 가중치·설정·평가 결과(소스 코드는 제외). JAX·flax 없이 `msgpack` + numpy 로
추론한다. 모든 모델은 관측·행동 구성이 같아 실행할 때 `--model` 로 골라 쓴다.

| 모델 (`--model …`)                         | 학습 평가 (랜덤화 환경)                         | 비고                      |
| ------------------------------------------ | ----------------------------------------------- | ------------------------- |
| `racing-body-rate-10s-dr-final` (**기본**) | 256/256 완주, G7 9.98 s, 최대 4.5 m/s·61°       | 도메인 랜덤화             |
| `racing-body-rate-5s-dr-final`             | 256/256 완주, G7 5.12 s, 최대 6.1 m/s·72°       | 도메인 랜덤화, ⚠️ 아래 참고 |
| `racing-body-rate-10s-final`               | 랜덤화 환경 203/1024 완주(19.8%)                | 랜덤화 없이 학습 (비교용) |

```bash
ros2 run crazyflie_rl rl_flight --list-models        # 설치된 모델과 학습 평가
ros2 run crazyflie_rl rl_flight --model 5s --dry-run # 이름 전체 또는 고유한 일부 ('5s', '10s-dr')
ros2 run crazyflie_rl rl_sim_check --model 5s
```

`--model` 이 없으면 기본 모델. 폴더를 직접 주려면 `--model-dir <경로>`.
기울기 중단 기준(`--max-tilt`)은 모델마다 자동으로 잡는다: 70° 와 (학습 평가 최대 기울기 + 10°)
중 큰 값, 최대 85° (10s-dr 71°, 5s-dr 82°, 10s 70°). 실행 시 `안전 기준:` 줄에 출력된다.

**새 모델 추가** — `models/<이름>/` 폴더를 만들어 아래 파일을 넣고 다시 빌드하면 `--model <이름>`
으로 바로 쓸 수 있다(setup.py 가 `models/` 아래 폴더를 모두 설치한다).

| 파일                   | 필요 | 쓰임                                                            |
| ---------------------- | ---- | --------------------------------------------------------------- |
| `best.msgpack`         | 필수 | 가중치                                                          |
| `config.yaml`          | 필수 | `control_mode`·`control_hz`·각속도 한계·관측 옵션 확인          |
| `best_evaluation.json` | 권장 | 기울기 안전 기준 자동 설정, `--list-models` 요약                |
| `metadata.json` 등     | 선택 | 기록용                                                          |

가중치만으로는 안 되고 `config.yaml` 이 같이 있어야 한다(각속도 한계·제어 주기가 모델마다 다를
수 있다). 로드할 때 `control_mode: body_rate`, `delay_gate_target_until_clear: false` 가
아니면 거부한다. **학습 코드의 관측·행동·환경(`racing.py` 의 observe/command 등)이 바뀐 모델은
`source/` 도 같이 주고 변환부터 다시 확인해야 한다** — 그대로 넣으면 로드는 되지만 틀린 관측으로 난다.

도메인 랜덤화: 질량 ±5%, 관성 ±10%, 추력·토크 ±5%, 모터 응답 ±15%, 항력 ±20%, 각속도 제어
이득 ±10%, 바람·돌풍, 위치·속도·자세·각속도·RPM 편향과 노이즈, 게이트 위치·방향 오차,
관측·명령 지연 1스텝(10 ms). 관측 구성은 그대로이고 노이즈·지연만 더해졌다.

```bash
# 랜덤화 없는 이전 모델로 비행·점검
ros2 run crazyflie_rl rl_flight --model racing-body-rate-10s-final
```

| 관측 (49)                                   | 행동 → 명령 (4)                               |
| ------------------------------------------- | --------------------------------------------- |
| 다음 목표 3개 상대위치 ×0.5 (9)             | ωx, ωy: ±12 rad/s                             |
| 그 목표들의 통과 법선 (9, 결승점은 0)       | ωz: ±8 rad/s                                  |
| 현재 목표 one-hot (8: 게이트 1..7, 결승)    | 총추력: 0.051 ~ 0.48 N                        |
| 회전행렬 (9) · 속도/4 (3) · 각속도/10 (3)   |                                               |
| 로터 RPM/25000 (4) · 직전 행동 (4)          |                                               |

결승점 = `gates.yaml` 의 start 상공(1.5, −2.0, 1.25). 게이트 통과는 학습과 같이 다음 게이트
평면을 진행 방향으로 넘을 때 개구부(반폭 0.19 m) 안이면 센다.

### 1. 오프라인 점검 (먼저)

```bash
ros2 run crazyflie_rl rl_sim_check
ros2 run crazyflie_rl rl_sim_check --latency 0.03 --rate-tau 0.05
```

crazyflow `cf2x_L250` 파라미터의 강체 모델(각속도·모터는 1차 지연으로 단순화)로 정책을 돌린다.
속도 부호·회전행렬 전치·게이트 법선 반전을 넣으면 실패하므로, 이 점검은 **변환이 틀렸는지
가려낸다.** 여기서 통과하는 것은 필요조건일 뿐 실기체 성공 보장은 아니다.

| 조건                                   | 기본 모델(DR)                    | 이전 모델                        |
| -------------------------------------- | -------------------------------- | -------------------------------- |
| 기본                                   | 완주, G7 9.93 s, 4.3 m/s, 51°    | 완주, G7 10.08 s, 2.6 m/s, 24°   |
| 관측 지연 30 / 50 ms                   | 완주 / 완주                      | 완주 / 완주                      |
| 각속도·모터 응답 τ = 0.06 s            | 완주                             | 완주                             |
| 추력 ×0.90 / ×1.10                     | 완주 / 완주                      | 정착 실패 / 완주                 |
| 지연 50 ms + 응답 τ = 0.06 s           | **추락 (기울기 69°)**            | 완주                             |

기본 모델은 추력 오차에 강해졌지만 훨씬 공격적이다(최대 4~5 m/s, 기울기 50~60°). 지연과 느린
응답이 겹치면 무너질 수 있으니, 첫 비행은 무선 지연을 줄이고(로그 대역 여유, 동글 가까이)
섀도 모드로 실제 지연부터 확인할 것.

**⚠️ 5s-dr 모델** — 기본 조건에서는 완주(G7 5.27 s, 최고 6.1 m/s, 출발 직후 기울기 약 70°)하지만,
간이 점검에서 **관측 지연 10 ms 만 넣어도 완주하지 못하고**, 추력 +10% 나 응답 τ = 0.05 s 에서도
추락한다(기울기 한계를 풀어도 마찬가지 — `rl_sim_check --model 5s --latency 0.01 --max-tilt 89`).
학습 평가는 1스텝 지연 랜덤화에서도 완주했으므로 간이 모델(각속도·모터 1차 지연)의 한계일 수
있지만, 한계 근처에서 나는 정책이라 모델 오차에 민감하다는 뜻이다. 실기체 무선 왕복 지연은
보통 1스텝(10 ms)보다 길다. 섀도 모드로 실제 지연을 확인하기 전에는 날리지 말 것.

### 2. 서버

```bash
ros2 launch crazyflie_rl launch.py
```

`config/crazyflies_rl.yaml` 로 서버를 띄운다(모캡 설정은 `crazyflie_test` 것 사용).
- `stabilizer.controller: 1` (**PID**) — Mellinger 는 레거시 rate 명령을 받지 않고 수평을 잡으려 한다.
- `flightmode.stabModeRoll/Pitch/Yaw: 0` — `cmd_vel_legacy` 의 roll/pitch/yaw 를 각속도로 해석.
- 정책용 로그 100 Hz: `rl_pv`(위치·속도) · `rl_att`(쿼터니언·모터 PWM) · `rl_gyro`(각속도).
- ⚠️ URI·기체 이름은 `crazyflie_test/config/crazyflies_mocap.yaml` 과 같게 유지할 것.
- sim 백엔드는 `cmd_vel_legacy` 를 구현하지 않아 쓸 수 없다.

### 3. 섀도 모드 (명령 없이 관측 확인)

```bash
ros2 run crazyflie_rl rl_flight --shadow --hover-pwm <호버 PWM>
```

아무 명령도 보내지 않고, 들어오는 상태로 관측을 만들어 정책 출력을 `~/flight_logs/rl_shadow_*.csv`
에 기록한다. 다른 터미널에서 `gate_flight --mocap` 으로 코스를 도는 동안 켜 두면, 게이트
진행 추적·관측 값·정책이 낼 명령(각속도·추력)이 실제 비행과 어울리는지 날리기 전에 볼 수 있다.
`--hover-pwm` 은 호버 중 `motor.m1..m4` 평균(모르면 생략 — crazyflow 기본 모델 사용).

### 4. 비행

```bash
ros2 run crazyflie_rl rl_flight --record
```

1. 지상에서 추력 0 레거시 패킷으로 thrust lock 을 풀고 HLC 로 start 상공까지 이륙·정렬
2. **호버 PWM 보정** — 호버 중 모터 PWM 평균을 학습 기체 호버 추력(0.313 N)에 맞춘다.
   추력 명령(N→PWM)과 로터 관측(PWM→RPM) 모두 이 비율을 쓴다. (간이 점검상 추력 명령
   스케일은 ±5% 안이어야 정착까지 되고, 로터 관측은 ±20% 까지 버틴다.)
3. 정책 비행 100 Hz. 게이트 통과마다 시각·속도 출력
4. G7 이후 결승점 10 cm·0.2 m/s 안에 0.5 s 머무르면 완주 → HLC 로 넘겨 착륙

중단 조건(→ 즉시 HLC 로 넘겨 현재 위치 유지 후 착륙): 기울기 > `--max-tilt`(모델별 70~85°), 벽까지
< `--wall-margin`(0.3 m), 고도 < `--min-z`(0.2 m), 상태 로그 끊김 > `--stale`(0.1 s),
`--max-time`(25 s) 초과. 사전 점검: 파라미터(PID·rate 모드) 확인, start 상공 15 cm·0.2 m/s 이내.

| 옵션                | 기본            | 설명                                              |
| ------------------- | --------------- | ------------------------------------------------- |
| `--model`           | 10s-dr          | 모델 이름 또는 고유한 일부 (`--list-models`)      |
| `--max-tilt`        | 모델별 자동     | 기울기 중단 기준 [deg] (위 모델 절 참고)          |
| `--shadow`          | -               | 명령 없이 관측·정책 출력만 기록                   |
| `--hover-pwm`       | 측정            | 호버 PWM 직접 지정 (보정 생략)                    |
| `--calib-time`      | 2.0             | 호버 PWM 측정 시간 [s]                            |
| `--aperture-margin` | 0.0             | 게이트 통과 판정 개구부 여유 [m]                  |
| `--record`          | -               | rosbag 도 기록 (`~/flight_logs/rl_<시각>/`)       |
| `--log-dir`         | `~/flight_logs` | 매 스텝 CSV(`rl_flight_<시각>.csv`) 폴더          |
| `--dry-run`         | -               | ROS 없이 정책 로드·start 호버에서의 첫 명령만 출력 |

### 실기체 차이 (확인 필요)

- **각속도 제어기** — 학습은 crazyflow Mellinger 계열 rate 루프(자세 항 제거), 실기체는 펌웨어
  PID rate 루프다. 응답이 다르면 궤적이 달라진다.
- **추력 모델** — 호버 한 점으로 선형 보정한다. 실제 PWM–추력은 비선형이고 배터리에 따라 변한다.
- **지연** — 무선 로그 + 명령 왕복(수십 ms). 학습은 지연 없음.
- **기체** — 학습 기체는 0.0319 kg(cf2x_L250). 마커 덱 등으로 더 무거우면 호버 보정이 흡수하는 건
  추력 스케일뿐이다.
- **코스** — 학습 코스 파일(`seven_gates.yaml`)과 이 레포 `gates.yaml` 은 해시가 달라 동일함을
  파일로 확인하지 못했다. 간이 점검에서 학습 평가와 같은 시각에 7개를 통과하는 것이 같은 코스라는
  근거다. 게이트를 옮기면 정책은 재학습이 필요하다.

## sim IMU/PWM 확장 (서브모듈 패치)

sim(`crazyflie_sim`)은 원래 `firmware_logging` 을 무시해 **imu_raw/motor_pwm/pose 를
안 낸다**(pose 는 `/tf` 뿐). 패치를 적용하면 **실기체와 동일 토픽·포맷**으로 낸다:

- `/<cf>/pose`      — `geometry_msgs/PoseStamped` (sim 은 ground truth)
- `/<cf>/imu_raw`   — `LogDataGeneric.values = [acc.x, acc.y, acc.z(g), gyro.x,y,z(deg/s)]`
- `/<cf>/motor_pwm` — `LogDataGeneric.values = [m1, m2, m3, m4]` (0..65535)

acc 는 동역학에서 **비추력**(drag-free, 바디 `[0,0,총추력/mass]`, 호버≈+1g z)으로
합성하고, gyro 는 `sensors.gyro`, PWM 은 `motors_thrust_pwm` 을 그대로 쓴다. ~100 Hz.
sim 파라미터 `sim.log_topics:=false` 로 끈다. 같은 패치에 게이트 궤적용
`plan_start_trajectory` 시그니처 수정도 들어 있다.

`crazyswarm2` 서브모듈 내부 수정이라 **패치 파일**로 보관한다. `git submodule update`
로 원복되면 다시 적용한다.

```bash
crazyflie_test/patches/apply_sim_patch.sh          # 적용 (이미 적용돼 있으면 건너뜀)
crazyflie_test/patches/apply_sim_patch.sh --check  # 적용 여부만 확인
crazyflie_test/patches/apply_sim_patch.sh --revert # 원본 sim 으로 복원
```

자세한 내용: [crazyflie_test/patches/README.md](crazyflie_test/patches/README.md)

> **sim vs 실기체 데이터 차이**: sim pose 는 GT(온보드 추정 아님), imu 는 drag-free·무노이즈,
> pwm 은 컨트롤러 출력 그대로. 실기체 imu 는 필터 후 값·노이즈 포함, pose 는 온보드 Kalman
> 추정. 학습 시 도메인 차이를 고려할 것.

## 기록 데이터 · bag 변환

| 토픽                  | 내용                               | 단위 / 비고                                      |
| --------------------- | ---------------------------------- | ------------------------------------------------ |
| `/poses`              | **mocap 원본 pose = ground truth** | `NamedPoseArray`. mocap 모드에서만 존재          |
| `/cf231/pose`         | 드론 온보드 Kalman **추정값**      | GT 아님. IMU 융합 + 라디오 왕복 지연 포함        |
| `/cf231/imu_raw`      | `acc.x/y/z`, `gyro.x/y/z`          | acc=g, gyro=deg/s. 완전 raw 아님(**필터 후** 값) |
| `/cf231/motor_pwm`    | `motor.m1~m4`                      | **PWM(0~65535)**, 뉴턴 추력 아님                 |
| `/cf231/cmd_full_state` | 레퍼런스 입력                    | `crazyflie_traj` 비행에서만                      |
| `/cf231/status`       | 배터리 전압·supervisor             |                                                  |

- **GT 는 `/poses`, 추정값은 `/cf231/pose`** 로 서로 다르다. 같이 기록하면 온보드 추정
  성능(오차·지연)을 GT 대비로 평가할 수 있다.
- `/poses` 는 모든 강체를 이름과 함께 담아, CSV 로는 `poses.0.name`,
  `poses.0.pose.position.x` … 형태로 펼쳐진다.
- 모터 PWM → 추력(N) 변환은 범위 밖. 오프라인에서 Bitcraze PWM→thrust 곡선 또는 자체
  캘리브레이션으로 변환.
- 펌웨어 로그 패킷 크기 제한 때문에 imu_raw(6개)/motor_pwm(4개)로 토픽을 나눠 둠. 유지할 것.

bag → 토픽별 CSV:

```bash
python3 crazyflie_test/scripts/bag_to_csv.py ~/flight_logs/<bag_dir> ~/flight_logs/<out_dir>
```

토픽별 타임스탬프가 다르므로 시계열 정렬이 필요하면 `timestamp_ns` 기준으로 리샘플/보간한다.

## analysis — 오프라인 분석

ROS 패키지가 아닌 독립 스크립트다. 경로 상수(`~/flight_logs` 등)는 각 파일 상단에서 바꾼다.

| 파일                     | 역할                                                                   |
| ------------------------ | ---------------------------------------------------------------------- |
| `visualize_flights.py`   | 수집한 `.db3` 를 훑어 궤적 XY/3D·고도 PNG 생성, 품질 선별용 summary    |
| `organize_trainset.py`   | summary 플래그로 비행 선별 → `trainset/` 정리, 불량은 `_rejected/` 격리 |
| `preprocess_trainset.py` | 이착륙 제거·타임스탬프 통일 등 학습 데이터 전처리                       |
| `build_notebook.py`      | `trainset_analysis.ipynb` 생성 (지연·드롭·배터리 vs 출력 분석)          |
| `gate_3d.py`             | 게이트 비행 bag 중 마지막 게이트까지 간 것을 게이트와 함께 3D 시각화    |

## 문제 해결

**추락 후 명령에 반응 없음** — 추락하면 High-Level Commander Lock 이 걸려 `takeoff` 등을
보내도 반응하지 않는다(서버 로그엔 정상 발행된 것처럼 찍힌다). 재부팅한다:

```bash
ros2 run crazyflie reboot --uri radio://0/80/2M/E7E7E7E7E7
```

> ⚠️ 라디오를 직접 여는 명령이라 서버가 떠 있으면 동글 충돌(busy)이 난다. launch 를 먼저 끈다.

**rviz 게이트 경로가 두 개 겹쳐 깜빡임** — `gate_markers` 가 두 개 떠 있는 것이다(launch 가
이미 띄웠는데 따로 또 실행했거나, 옛 프로세스가 남음). `pkill -f gate_markers` 후 launch 를 다시 띄운다. `ros2 topic info /gate_course/path` 의
`Publisher count` 가 **1** 이어야 한다.

**sim 이 `startTrajectory` 에서 `plan_start_trajectory() missing ... start_yaw` 로 죽음** —
설치된 `crazyflie_sim` 이 최신 `cffirmware`(7인자 시그니처)보다 오래된 것이다.
[sim 패치](#sim-imupwm-확장-서브모듈-패치)를 적용하면 해결된다(`relative_yaw`·`start_yaw`
전달). 실기체 펌웨어는 해당 없음.

**sim 에서 imu_raw/motor_pwm/pose 토픽이 없음** — sim 패치 미적용. `apply_sim_patch.sh --check`.

**`ros2 run crazyflie_test gate_flight` / `circle` 이 없다고 나옴** — 패키지가 분리됐다.
`crazyflie_racing` / `crazyflie_traj` 로 실행하고, 옛 `build/`·`install/` 을 지운 뒤 다시 빌드한다.
