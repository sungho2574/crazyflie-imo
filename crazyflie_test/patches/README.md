# crazyswarm2 sim 패치

`crazyswarm2`(서브모듈)의 시뮬레이터를 두 가지로 고치는 패치를 **패치 파일로** 보관한다.
서브모듈 내용을 직접 커밋하지 않으므로, `git submodule update` 로 원복되면 다시 적용한다.

## 무엇을 고치나 — `crazyswarm2_sim_logging.patch`

- **`crazyflie_sim/.../crazyflie_sil.py`** — `plan_start_trajectory` 를 최신 `cffirmware`
  시그니처(7인자: `relative_yaw`, `start_yaw` 추가)에 맞춘다. 이게 없으면 게이트 궤적
  `startTrajectory` 에서 `TypeError` 로 sim 서버가 죽는다.
- **`crazyflie_sim/.../crazyflie_server.py`** — sim 이 `pose · imu_raw · motor_pwm` 를
  **실기체와 동일 토픽·포맷**(`geometry_msgs/PoseStamped`, `crazyflie_interfaces/LogDataGeneric`)
  으로 발행한다. Blackbird 궤적 학습 데이터 수집(`collect_traj_data`)에 필요.
  `sim.log_topics:=false` 로 끌 수 있다.

## 사용

```bash
# 최초 셋업: 서브모듈 받은 뒤 패치 적용
git submodule update --init --recursive
crazyflie_test/patches/apply_sim_patch.sh

# 서브모듈을 sync/update 해서 패치가 날아갔을 때 다시 적용
crazyflie_test/patches/apply_sim_patch.sh          # 적용 (이미 적용돼 있으면 건너뜀)
crazyflie_test/patches/apply_sim_patch.sh --check  # 적용 여부만 확인
crazyflie_test/patches/apply_sim_patch.sh --revert # 원본 sim 으로 복원
```

symlink-install 이면 적용 후 재빌드 불필요(파이썬 수정이라 바로 반영). sim 서버를 다시
띄우면 확장된 토픽이 나온다.

## 패치 갱신

sim 코드를 더 고쳤다면 패치를 다시 뜬다:

```bash
git -C crazyswarm2 diff \
  crazyflie_sim/crazyflie_sim/crazyflie_sil.py \
  crazyflie_sim/crazyflie_sim/crazyflie_server.py \
  > crazyflie_test/patches/crazyswarm2_sim_logging.patch
```
