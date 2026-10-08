# Validation — 2026-10-08

System Python 3.10 / ROS Humble, CPU inference, bundled CFSim epoch 85 checkpoint.
No radio connection or hardware flight was performed. Existing training results
and Crazyswarm2 sources were not modified.

## End-to-end ROS simulation

Onboard takeoff → start pose/yaw alignment → stabilization → one-time onboard state
handoff → host IMO/EKF and position controller → landing. Ground-truth position
is never subscribed to by the host controller; onboard pose is ignored after the
handoff. Firmware XYZ control modes are disabled during IMO control.

| Run | Estimation RMS | Actual tracking RMS | Max actual tracking error | Accepted / rejected learned updates |
|---|---:|---:|---:|---:|
| Circle, R=1 m, cruise=1 m/s, 1 lap | 0.1103 m | 0.1106 m | 0.2405 m | 235 / 0 |
| Figure8, period=12 s, scale=1, 1 lap | 0.2253 m | 0.2268 m | 0.4750 m | 350 / 0 |

Both completed, with zero dropped sensor pairs. Errors are computed in the world
frame without post-hoc alignment, from handoff through landing. These are single
runs, not a statistical reliability estimate. The simulator has ideal attitude
sensing, no drag/noise, and interval-averaged IMU. The conservative online model
sigma is 0.3 m, with simulator-specific small initial/process uncertainty. These
conditions and settings must not be presented as hardware performance.

Circle artifacts:

- `/home/artemis-2/flight_logs/imo/20261008_162824_230576963/`
- `/home/artemis-2/flight_logs/imo/ground_truth_20261008_162822.csv`

Figure8 artifacts:

- `/home/artemis-2/flight_logs/imo/20261008_162958_496341938/`
- `/home/artemis-2/flight_logs/imo/ground_truth_20261008_162956.csv`

Each run directory contains `evaluation.json`, `evaluation.png`, model hash,
configuration, initialization state, actual raw inputs, command/estimate logs,
and learned update counters.

Commands:

```bash
source /opt/ros/humble/setup.bash
source /home/artemis-2/crazyflie/cf_ws/install/setup.bash
ros2 launch crazyflie_imo_racing flight.launch.py backend:=sim
ros2 launch crazyflie_imo_racing flight.launch.py trajectory:=figure8 period:=12.0
```

## Offline replay

`datasets/CFSim/figure8/yawForward/maxSpeed1p0/test/data.hdf5`, first 10 seconds:
1000 sensor samples, 189 learned updates, unaligned position ATE **0.0490 m**.
CPU elapsed time was 1.50 seconds including the replay's filter work. This replay
uses the sequence's initial GT pose and velocity once to match the offline
benchmark; it does not continuously fuse GT. It is separate from the flight
node and uses the estimator adapter's offline defaults, not the live simulator
profile. Result: `/tmp/imo_replay_final.json`.

## Tests and build

- `colcon build --symlink-install --packages-select crazyflie_imo_racing`
- 15 tests: motor mapping, controller signs, nonzero-yaw firmware convention,
  polynomial time scaling, analytic derivatives, path continuity, timestamp
  pairing/jitter/gap handling with the actual model, delayed estimator handoff,
  unsettled-state rejection, ignoring pose after handoff, and firmware manual
  control independence from position/velocity.
- The last firmware independence test explicitly changes simulator position and
  velocity while holding attitude/sensors/commands fixed and verifies identical
  motor output, covering the adapter's post-handoff position/velocity zeroing.

Existing upstream torch weight_norm deprecation and Numba object-mode warnings
remain; neither prevented the checks from completing.

## What is not established

Hardware performance, long multi-lap drift, all arbitrary gate CSVs, packet-loss
flight recovery, and real-trained checkpoint performance have not been flight
validated. Hardware launch and logs use the cflib interfaces in this workspace,
but the hardware profile is deliberately separate and requires manual start.
Emergency stop is a motor stop, not guaranteed safe landing after estimator loss.

## RViz 연결 검증 (2026-10-08)

`trajectory:=figure8 period:=12.0`을 별도 ROS domain 72에서 실제 ROS launch로 실행했다.
RViz OpenGL 초기화와 `/imo/reference_path` 구독의 Reliable/Transient Local QoS 일치를 확인했다.
비행 `complete` 시 world 좌표 참조 1001점, 추정 1316점, 실제 1484점 및 marker 3개를 확인했다.
완료 후 신규 subscriber가 마지막 추정 경로를 수신하는 것도 확인했다. 기존 pytest 15개 통과.

## 서버·비행 분리 검증 (2026-10-08)

`server.launch.py`만 실행했을 때 `imo_flight` 노드와 자동 이륙이 없음을 확인했다.
ROS domain 73에서 같은 서버에 `flight.launch.py trajectory:=hover period:=1.0`을
두 번 순차 실행하여 모두 complete/정상 종료, 서버 생존 및 경로 유지(참조 1001점,
추정 142점)를 확인했다. RViz 기본 패널·Tools를 설치된 Humble 기본 설정에서 복구했고,
MoveCamera/Interact 등록과 RViz 기동을 확인했다. 마우스 조작 자체는 자동 검증하지 않았다.
pytest 15개 통과.
