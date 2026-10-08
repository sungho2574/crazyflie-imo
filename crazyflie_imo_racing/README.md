# crazyflie_imo_racing

CFSim으로 학습한 변위 모델과 기존 IMO EKF를 Crazyflie 비행에 연결하는 ROS 2 패키지.

```
기존 onboard 제어: 이륙 → 궤적 시작점·yaw 정렬 → 호버 안정화
                                      ↓ 1회 상태 인계
IMU raw + 실제 motor PWM → 호스트 IMO/clone EKF → 호스트 위치 제어
                                                   ↓
                              roll/pitch/yaw-rate/thrust 명령
                                                   ↓
                              Crazyflie 펌웨어 자세 제어
```

**이륙 중에는 실제 센서로 IMO를 실행하지 않는다.** 지상에서 바이어스를 교정하고,
이륙·정렬이 끝나면 onboard pose/그 차분 속도로 한 번 초기화한다. 궤적 비행이 시작된
뒤에는 onboard pose를 무시한다. 이후 위치 피드백은 호스트 EKF뿐이며, 착륙도 호스트
제어로 수행한다. 모델/JIT 워밍업은 비행 전 별도 가상 입력으로 수행하고 상태를 버린다.

## 실행

ROS Humble의 **시스템 Python 3.10**을 사용한다. Python 3.11 학습용 conda 환경을
ROS 프로세스에 섞지 않는다. 시스템 Python에 torch(CPU), numba, numpy, scipy,
PyYAML, rowan, cffirmware가 필요하다. HDF5 재생은 h5py, 그래프는 matplotlib도 필요하다.
현재 머신에 있는 의존성으로 빌드·실행을 확인했다.

```bash
source /opt/ros/humble/setup.bash
cd /home/artemis-2/crazyflie/cf_ws
colcon build --symlink-install --packages-select crazyflie_imo_racing
source install/setup.bash

# 터미널 1: 서버 + RViz만 실행. 이 명령으로는 이륙하지 않는다.
ros2 launch crazyflie_imo_racing server.launch.py backend:=sim

# 터미널 2: 위와 같이 ROS/워크스페이스 source 후 실행
# 기본: CFSim epoch 85, 반경 1 m 원, 순항 1 m/s, 1랩, 진행 방향 yaw
ros2 launch crazyflie_imo_racing flight.launch.py

# CFSim과 같은 8자 형상, 순항 최대 약 1.05 m/s
ros2 launch crazyflie_imo_racing flight.launch.py trajectory:=figure8 period:=12.0

# 수동 시작
ros2 launch crazyflie_imo_racing flight.launch.py autostart:=false
ros2 service call /imo/start std_srvs/srv/Trigger '{}'
```

`server.launch.py`가 RViz를 자동 실행한다. `flight.launch.py`는 이미 실행 중인 서버에
연결하여 한 번 비행하고 종료한다. 다음 비행은 같은 서버에서 flight 명령을 다시 실행한다.
동시에 여러 flight 프로세스를 실행하지 않는다. 두 터미널의 `config`에서 robot/backend/
initial_position/firmware_controller 설정은 같아야 한다. 모델과 궤적은 flight 쪽에서 선택한다.
새 비행이 시작되면 RViz의 이전 경로를 지우고 새 참조 궤적으로 교체한다. Fixed Frame은 `world`이며 청록색은 참조 궤적,
초록색은 IMO 추정 경로, 주황색은 시뮬레이션 실제 경로다. 기체와 비행 상태도 표시한다.
시뮬레이션의 주황색 기체 모형은 실제 위치·자세를 따라간다. 초록색 IMO 경로는
궤적 시작 이후부터 별도로 그린다. 실기체의 기체 모형은 이륙 중 onboard pose,
IMO 시작 후에는 추정 위치·자세를 따른다.
실기체에서는 시뮬레이션 실제 경로가 표시되지 않는다.
비행 완료 후에도 서버와 화면·경로를 유지하며, 서버 터미널에서 `Ctrl+C`로 종료한다.
RViz 기본 패널과 도구를 사용한다. 상단 `Move Camera`를 선택하면 왼쪽 드래그로
회전, 가운데 드래그로 이동, 휠로 확대/축소할 수 있다. Views에서 Orbit 시점을 바꿀 수도 있다.

```bash
# 터미널 1: RViz 없이 서버 실행
ros2 launch crazyflie_imo_racing server.launch.py rviz:=false
# 터미널 2: 비행 완료 후 flight만 자동 종료
ros2 launch crazyflie_imo_racing flight.launch.py

# 별도로 연 RViz에는 아래 설정을 로드한다.
rviz2 -d $(ros2 pkg prefix crazyflie_imo_racing)/share/crazyflie_imo_racing/config/imo.rviz
```

`config/flight.yaml`이 시뮬 기본값이다. 이륙 4초 → 시작 위치·yaw 정렬 2초 + 안정화
1초 후 IMO로 전환한다. 기본 원은 가속 2초, 정속 구간, 감속 2초로 총 8.283초,
그 후 착륙 4초이다. `period`는 **정속일 때 한 랩 시간**이고 실제 총 시간은
`laps × period + ramp_duration`이다. 모델 학습 시 사용한 수집 스케줄과 같은 형태다.

## 모델과 궤적 선택

기본 체크포인트는 **`results/CFSim/checkpoints/model_net/checkpoint_best.pt`, epoch 85**.
전체 optimizer를 제외한 가중치 복사본을 `models/cfsim_best.pt`에 포함했다.
원본/복사본 SHA256, 비교 기준과 평가 수치는 `models/selection.json`에 있다.

저장된 공통 CFSim test 14개에서 unaligned EKF ATE 평균이 가장 낮은 기존 실행을
선택했다(평균 0.281 m, 중앙값 0.146 m). `star 2 m/s`도 포함했다. 최종 실행의
`circle 1 m/s` 결과 파일 하나가 없어 공통 14개만 비교했다. 스윕의 D80은 중앙값이
더 낮지만 평균/최악 오차는 더 크다. 기존 평가의 필터 설정은 실행마다 다르므로
이 기록은 체크포인트만 바꾼 동일 조건 재평가를 뜻하지 않는다.

```bash
# 다른 CFSim/실기체 학습 모델. 두 경로를 함께 지정한다.
ros2 launch crazyflie_imo_racing flight.launch.py \
  checkpoint:=/absolute/path/checkpoint_best.pt \
  model_parameters:=/absolute/path/model_net_parameters.json

# 호버 / 궤적 크기 / 랩 수
ros2 launch crazyflie_imo_racing flight.launch.py trajectory:=hover period:=3.0
ros2 launch crazyflie_imo_racing flight.launch.py radius:=1.0 laps:=2

# 기존 crazyflie_racing의 다항식 CSV
ros2 launch crazyflie_imo_racing flight.launch.py \
  config:=/absolute/path/my_flight.yaml trajectory:=csv \
  trajectory_csv:=/absolute/path/gate_trajectory.csv timescale:=2.0
```

CSV 형식: 헤더 + duration, x/y/z/yaw 각 8개 계수(총 33열). world 절대 좌표로 실행한다.
`my_flight.yaml`의 `initial_position` XY를 CSV 시작 XY와 맞춘다. Z는 지상 위치다.
CSV 시작점으로 이륙하고 yaw를 정렬한다. `timescale`은 CSV 시간과 속도/가속도에 적용한다.
반복 CSV는 끝점과 시작점의 위치·속도·가속도가 이어져야 한다. 내장 궤적의 yaw는
`yaw_mode: forward`가 기본이다. `fixed`도 지원하지만 CFSim 학습 조건과 다르다.

커스텀 모델은 기존 6입력/3출력 TCN 구조와 `model_state_dict` 키를 가져야 한다.
JSON의 `sampling_freq`와 `window_time`을 읽는다. 원본 CFSim 체크포인트도 직접 사용
가능하다. 안전한 weights-only 로딩과 호환되지 않는 오래된 pickle이면 신뢰하는 파일에
한해서 별도로 tensor 가중치만 export한다. 런타임에서 임의 pickle 허용으로 우회하지 않는다.
`imo_repo:=...`로 원본 학습 저장소 경로를 바꿀 수 있다.

## 추정·제어 세부사항

- accel g → m/s², gyro deg/s → rad/s. 네 모터 PWM → RPM → 힘 → 질량으로 나눈
  body +Z 추력은 CFSim 변환기의 식과 동일하다. 질량과 `thrust_scale`은 설정 가능하다.
- 네트워크 입력은 EKF 자세로 world에 회전한 gyro와 추력이다. 가속도계는 EKF 전파에
  사용한다. 기본 창은 100 Hz × 0.5초, 학습 변위 업데이트는 20 Hz이다.
- 기체 millisecond timestamp로 두 log block을 짝지으며 도착 시간으로 적분하지 않는다.
  원본 FilterRunner의 일정 간격 요구를 맞추기 위해 정수 microsecond 시간축으로 보간한다.
  역행/리셋, 긴 데이터 공백, 과도한 지연은 비행을 중단한다.
- 원본 `FilterRunner`/`ImuMSCKF`를 import한다. 원본 저장소는 수정하지 않는다.
  초기 thrust 버퍼를 직접 채우고, 첫 업데이트부터 NIS 검사 및 위치/속도/자세 보정량
  상한을 적용한다. 과도한 보정은 상태에 적용하기 전에 거부한다.
- 호스트 위치 PD + 가속도 feedforward가 자세/추력 지령을 계산한다. cflib의
  `cmd_vel_legacy` 규약(각도 degree, yaw-rate degree/s, collective PWM)에 맞춘다.
  기본 펌웨어 내부 제어기는 `mellinger`; `pid`도 설정 가능하다. 펌웨어의 XYZ 제어는
  IMO 전환 후 사용하지 않는다.

**온라인 필터 설정은 오프라인 결과와 다르다.** CFSim 최종 모델은 순항 중심으로
학습됐고, 새 제어 루프에서는 센서/제어 분포가 달라 측정 오차가 커졌다. 기본 시뮬 설정은
변위 σ=0.3 m로 보수적으로 반영하고, 잡음 없는 이상 센서에 맞춰 작은 IMU/초기 상태
불확실성을 쓴다. 이 수치를 실기체에 그대로 적용하면 안 된다. 두 오차 설정과 보정
상한은 모두 YAML에 있으며, 적용/거부 횟수가 결과에 기록된다. 위치/yaw의 절대 기준과
장기 드리프트는 순수 관성·추력 오도메트리만으로 해결되지 않는다.

## 시뮬 백엔드

`crazyflie_sim.backend.np.Quadrotor`와 `CrazyflieSIL`/cffirmware를 재사용하는 어댑터다.
공용 Crazyswarm2 소스나 데이터 수집 코드는 바꾸지 않는다.

- 기존 sim 서버에 없던 legacy 자세·추력 명령을 구현했다.
- 물리 적분 2 kHz, 센서 전송 100 Hz. 센서는 10 ms 구간 평균으로 anti-aliasing한다.
  PWM은 실제 firmware mixer 출력이다. 기존 수집기의 순간 샘플과 동일하지 않으며,
  이 차이도 온라인 모델 분포 차이에 포함된다.
- 펌웨어가 요구하는 ZYX roll/pitch/yaw로 자세각을 전달한다. upstream SIL의 intrinsic
  XYZ 변환을 그대로 쓰면 yaw가 있는 기동에서 PID 방향이 틀어지는 문제를 방지한다.
- 이륙·시작점 정렬 동안만 simulator 위치/속도로 기존 제어를 한다. IMO 전환 뒤에는
  펌웨어 위치/속도 필드도 0으로 지워 두며, 내장 위치 제어 모드는 비활성이다.
  실제 자세/자이로는 이상적인 onboard attitude sensing으로 사용한다.
- `/imo/ground_truth`는 평가용으로만 발행한다. 호스트 비행 노드는 이 토픽을 구독하지 않는다.
  `/cf231/pose`도 초기 인계 후에는 읽지 않는다.
- wall timer 방식이므로 부하에 따라 실시간보다 느릴 수 있다. 적분은 센서 시각 기준이다.

## 실기체로 옮길 때

`config/flight_hardware.yaml`은 **아직 비행 검증하지 않은 시작 설정**이다.
시뮬보다 큰 센서/초기 상태 불확실성, 짧은 호버, 수동 시작으로 분리했다.

1. `flight_hardware.yaml`, `crazyflies.yaml`을 복사해 기체 이름/URI, 초기 위치·기수,
   측정 질량, PWM 추력 보정과 센서 배율/잡음을 설정한다. `thrust_scale=1`은 실기체
   보정값이라는 뜻이 아니다. 나중에 실기체 학습 모델을 경로로 교체할 수 있다.
2. 이륙·정렬용 onboard 위치 추정(Flow deck 또는 별도로 구성한 mocap)이 필요하다.
   100 Hz `imu_raw`/`motor_pwm`, 50 Hz `pose`, 배터리 status 로그를 사용한다.
3. 아래처럼 연결한다. 하드웨어의 `autostart:=true`는 거부된다. 시뮬 튜닝을 무심코
   재사용하지 않도록 **하드웨어용 config를 명시**한다.

```bash
# 터미널 1: 기체 연결 + RViz
ros2 launch crazyflie_imo_racing server.launch.py backend:=cflib \
  config:=/absolute/path/flight_hardware.yaml crazyflies:=/absolute/path/crazyflies.yaml
# 터미널 2: 추정·비행 노드 (교정 후 수동 시작 대기)
ros2 launch crazyflie_imo_racing flight.launch.py backend:=cflib \
  config:=/absolute/path/flight_hardware.yaml
# 터미널 3: 시작
ros2 service call /imo/start std_srvs/srv/Trigger '{}'
```

지상·모터 정지 상태로 100쌍의 센서 샘플을 모아 교정한다. 시작에는 최신 배터리,
센서·pose, firmware supervisor 상태와 arm/takeoff/go_to/land 서비스가 필요하다.
현 펌웨어의 Mellinger manual attitude 모드 지원을 전제로 한다. 실행 과정에서
이 모델을 실기체에 연결하거나 실제 비행시키지는 않았다.

```bash
# 정상 상태에서 착륙: 인계 전에는 onboard land, 인계 후에는 IMO 기반 착륙
ros2 service call /imo/land std_srvs/srv/Trigger '{}'
# 즉시 모터 정지 — 착륙 명령이 아님. 다시 날리려면 프로세스를 재시작한다.
ros2 service call /imo/stop std_srvs/srv/Trigger '{}'
```

추정 상태 이상, 센서 watchdog, 추종 오차/속도/기울기 제한 초과 시에도 모터 정지를
래치한다. 실기체에서는 별도 emergency 수단과 지상 추정 검증이 필요하다.

## 결과·검증

`~/flight_logs/imo/<timestamp>/`:

- `run.json`: 모델 경로/hash 및 전체 실행 설정
- `calibration.json`, `handoff.json`: 교정과 **IMO가 켜진 순간**의 상태
- `sensors.csv`: 원시 IMU/PWM, `flight.csv`: 인계 이후 추정·참조·명령
- `result.json`: 종료 원인, 학습 업데이트 적용/거부 횟수, 유실 샘플 수

`/imo/odometry`는 pose/velocity 공분산을 포함하고, `/imo/status`는 상태 전환을 보낸다.
별도 `ground_truth_<timestamp>.csv`와 비교해 절차 완료와 실제 추종 성공을 구별한다.

```bash
ros2 run crazyflie_imo_racing imo_evaluate \
  --run /absolute/path/run_folder --truth /absolute/path/ground_truth.csv --plot
# evaluation.json, evaluation.png 생성. 정렬해서 드리프트를 지우지 않는다.

ros2 run crazyflie_imo_racing imo_replay --help
# HDF5 기반 읽기 전용 재생. 오프라인 비교용 초기 GT pose/velocity는 이 도구에서만 사용.

cd /home/artemis-2/crazyflie/cf_ws/src/crazyflie-imo/crazyflie_imo_racing
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -q test
```

구현 검증 수치는 `VALIDATION.md`에 기록한다. 패키지는 원본 GPL-3.0 IMO 구현에 의존하며,
해당 라이선스를 함께 포함한다.
