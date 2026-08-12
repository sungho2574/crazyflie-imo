#!/bin/bash
# crazyswarm2 sim 확장 패치를 적용/복원한다.
#
# 이 패치는 crazyflie_sim(서브모듈)을 두 가지로 고친다:
#   1. crazyflie_sil.py     — plan_start_trajectory 를 최신 cffirmware(7인자)에 맞춤
#                             (게이트 궤적 startTrajectory 시 sim 크래시 수정)
#   2. crazyflie_server.py  — pose / imu_raw / motor_pwm 를 실기체와 동일 토픽·포맷으로
#                             발행 (Blackbird 학습 데이터 수집용)
#
# 서브모듈이라 `git submodule update` 하면 원복되므로, 그 뒤 이 스크립트를 다시 돌린다.
#
#   crazyflie_test/patches/apply_sim_patch.sh          # 적용
#   crazyflie_test/patches/apply_sim_patch.sh --revert # 복원
#   crazyflie_test/patches/apply_sim_patch.sh --check  # 적용 가능/여부만 확인
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH="$HERE/crazyswarm2_sim_logging.patch"
# patches/ -> crazyflie_test -> repo root -> crazyswarm2
SUB="$(cd "$HERE/../.." && pwd)/crazyswarm2"

if [ ! -d "$SUB/crazyflie_sim" ]; then
  echo "✗ crazyswarm2 서브모듈을 못 찾음: $SUB"
  echo "  git submodule update --init --recursive 먼저 실행하세요."
  exit 1
fi

cd "$SUB"
case "${1:-apply}" in
  --revert)
    git apply -R "$PATCH" && echo "✓ 패치 복원됨 (원본 sim)" ;;
  --check)
    if git apply --reverse --check "$PATCH" 2>/dev/null; then
      echo "= 이미 적용돼 있음"
    elif git apply --check "$PATCH" 2>/dev/null; then
      echo "○ 적용 가능 (아직 미적용)"
    else
      echo "⚠ 깔끔히 적용/복원 불가 — sim 버전이 바뀌었을 수 있음. 패치 수동 확인 필요"
      exit 2
    fi ;;
  apply|*)
    if git apply --reverse --check "$PATCH" 2>/dev/null; then
      echo "= 이미 적용돼 있음 (건너뜀)"
    else
      git apply "$PATCH" && echo "✓ 패치 적용됨 (sim 이 imu/pwm/pose 발행)"
    fi ;;
esac
