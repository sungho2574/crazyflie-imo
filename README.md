# crazyflie-imo

Crazyswarm2 기반 Crazyflie 비행 테스트 리포.

- 단일 / 군집 예제 모두 제공
- Flow deck(opticalflow) / mocap(Qualisys) 예제 모두 제공
- IMU raw · 모터 PWM · pose 를 rosbag 으로 기록

## 구성

```
.
├── crazyswarm2/            # Crazyflie ROS 2 스택 (서버·드라이버)
└── crazyflie_test/         # 비행 테스트 패키지
```

## 실행 환경

- Ubuntu 22.04 + ROS 2 Humble
- Ubuntu 24.04 + ROS 2 Jazzy

## 설치

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/sungho2574/crazyflie_test.git
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
colcon build
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
ros2 bag record /cf231/pose /cf231/imu_raw /cf231/motor_pwm \
  -o ~/flight_logs/$(date +%Y%m%d_%H%M%S)

# 터미널 3 — 테스트 스크립트 (서버 유지한 채 여러 번 실행 가능)
ros2 run crazyflie_test hello_world
ros2 run crazyflie_test goto_square
ros2 run crazyflie_test figure8
ros2 run crazyflie_test multi_opticalflow
```

- `launch.py` 인자:
  - `mode`: `opticalflow`|`opticalflow_multi`|`mocap` (default `opticalflow`)
  - `backend`: `cflib`|`cpp`|`sim` (default `cflib`)

## 후처리

```bash
python3 crazyflie_test/scripts/bag_to_csv.py ~/flight_logs/<bag_dir> ~/flight_logs/<out_dir>
```

토픽별 타임스탬프가 다르므로, 시계열 정렬이 필요하면 CSV 로드 후 pandas 등으로
`timestamp_ns` 기준 리샘플링/보간할 것.

## 기록되는 데이터

| 토픽               | 내용                            | 단위 / 비고                                      |
| ------------------ | ------------------------------- | ------------------------------------------------ |
| `/cf231/pose`      | 상태추정 position + orientation | mocap 모드에서는 사실상 ground truth             |
| `/cf231/imu_raw`   | `acc.x/y/z`, `gyro.x/y/z`       | acc=g, gyro=deg/s. 완전 raw 아님(**필터 후** 값) |
| `/cf231/motor_pwm` | `motor.m1~m4`                   | **PWM(0~65535)**, 뉴턴 추력 아님                 |

- 모터 PWM → 추력(N) 변환은 이 패키지 범위 밖. PWM 을 기록해 두고 오프라인에서
  Bitcraze PWM→thrust 곡선 또는 자체 캘리브레이션으로 변환.
- 펌웨어 로그 패킷 크기 제한 때문에 imu_raw(6개)/motor_pwm(4개)로 토픽을 나눠 둠. 유지할 것.

## QTM(Qualisys) 수동 설정 (mocap 모드)

- 마커 스트리밍 방식(`tracking: librigidbodytracker`, 현재 기본): QTM 의 6DOF 계산을 **끄고**
  labeled/unlabeled 마커를 스트리밍.
- QTM 이 6DOF 를 계산하게 하려면 `crazyflies_mocap.yaml` 의 `tracking` 을 `"vendor"` 로 바꾸고
  QTM `Project options → Processing → Real-time actions → Calculate 6DOF` 체크.
- 좌표계 up axis 는 Z 로 설정.
- `config/motion_capture.yaml` 의 `hostname` 을 QTM PC IP 로 교체.
