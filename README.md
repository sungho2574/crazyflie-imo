# crazyflie-imo

Crazyswarm2 기반 Crazyflie 비행 테스트 리포.

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

## 설치

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

```bash
rviz2
```

- 이때 rviz2 화면에서
  - Global Options → Fixed Frame → `world` 선택
  - Add 버튼 클릭 → TF 선택

### 확장 버전 (launch/logging/test)

```bash
# 터미널 1 — 서버 (config 적용 지점)
export ROS_DOMAIN_ID=42          # 랩 공용망이면 충돌 방지용
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
ros2 run crazyflie_test figure8            # 8자 — goTo 샘플링 (간단)
ros2 run crazyflie_test figure8_traj       # 8자 — 다항식 궤적 업로드 (부드러움, 정석)
ros2 run crazyflie_test figure8_continuous # 8자 — 여러 바퀴 끊김 없이 연속
ros2 run crazyflie_test multi_opticalflow
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

## 후처리

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
