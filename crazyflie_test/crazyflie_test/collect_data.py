"""궤적별 비행 데이터 수집 — 학습 데이터 생성용.

각 (도형 × 속도) 조합마다:
  1. rosbag 으로 pose·imu_raw·motor_pwm·cmd_full_state(레퍼런스 입력)
     + status(배터리) + ground-truth 위치(mocap /poses) 기록 시작
  2. 해당 도형을 지정 랩 수만큼 연속 비행 (`ros2 run crazyflie_test <shape>`)
  3. 기록 종료 → (옵션) bag_to_csv.py 로 토픽별 CSV 변환

출력 폴더 (Blackbird 네이밍 미러):
    <out>/<shape>/<yawType>/<shape>_maxSpeed<V>/
    예) traj_data/clover/yawForward/clover_maxSpeed2p0/{bag, csv}

전제: **crazyflie 서버가 이미 떠 있어야 한다** (sim 이면 IMU/PWM 확장 반영본 필요).
    ros2 launch crazyflie launch.py backend:=sim ... crazyflies_yaml_file:=<...>
    ros2 run crazyflie_test collect_traj_data --shapes clover circle --speeds 1.0 2.0

⚠️ sim 은 확장으로 imu_raw/motor_pwm/pose 를 실기체와 같은 포맷으로 낸다. 단 sim pose 는
   ground truth, imu 는 drag-free·무노이즈(실기체와 특성 차이 있음 — README 참고).
"""
import argparse
import os
import signal
import subprocess
import time

from .traj import shapes as sh


def _vtag(v):
    return f'{v:.1f}'.replace('.', 'p')       # 2.0 -> 2p0


def _laps_for(shape, speed, seconds):
    """런당 스트리밍 시간이 대략 `seconds` 가 되도록 랩 수를 계산.

    period = max|dp/ds| / speed (한 랩 시간). laps ≈ seconds / period.
    """
    unit = sh.max_speed_unit(sh.get_shape(shape))
    period = unit / max(speed, 1e-6)
    return max(1, round(seconds / period))


def _bag_to_csv_path():
    """설치된 share → 소스 트리 순으로 bag_to_csv.py 를 찾는다."""
    bases = []
    try:
        from ament_index_python.packages import get_package_share_directory
        bases.append(get_package_share_directory('crazyflie_test'))
    except Exception:
        pass
    bases.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    for base in bases:
        path = os.path.join(base, 'scripts', 'bag_to_csv.py')
        if os.path.exists(path):
            return path
    return None


def record_one(shape, speed, laps, yaw, cf, out_root, extra, to_csv, gt_topic):
    """한 조합을 수집한다. 수집 폴더 경로를 반환."""
    yaw_type = 'yawForward' if yaw == 'forward' else 'yawConstant'
    name = f'{shape}_maxSpeed{_vtag(speed)}'
    out_dir = os.path.join(out_root, shape, yaw_type, name)
    os.makedirs(out_dir, exist_ok=True)
    bag_dir = os.path.join(out_dir, 'bag')
    if os.path.isdir(bag_dir):
        # rosbag 은 기존 폴더를 덮지 못한다 → 타임스탬프 붙여 새로
        bag_dir = os.path.join(out_dir, 'bag_' + time.strftime('%H%M%S'))

    topics = [f'/{cf}/pose', f'/{cf}/imu_raw', f'/{cf}/motor_pwm',
              f'/{cf}/cmd_full_state', f'/{cf}/status']   # status = 배터리·supervisor
    if gt_topic:                       # mocap ground-truth 위치 (예: /poses)
        topics.append(gt_topic)
    print(f'\n=== {shape} speed={speed} yaw={yaw} laps={laps} → {out_dir} ===')

    rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_dir] + topics,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           preexec_fn=os.setsid)
    time.sleep(2.0)      # 레코더가 구독을 붙일 시간

    fly = ['ros2', 'run', 'crazyflie_test', shape,
           '--laps', str(laps), '--speed', str(speed), '--yaw', yaw] + list(extra)
    print('  비행:', ' '.join(fly))
    subprocess.run(fly)  # 블로킹 — 착륙까지 대기

    # 레코더 종료 (프로세스 그룹에 SIGINT → 깔끔히 flush)
    os.killpg(os.getpgid(rec.pid), signal.SIGINT)
    try:
        rec.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(rec.pid), signal.SIGKILL)
    print(f'  기록 완료 → {bag_dir}')

    if to_csv:
        b2c = _bag_to_csv_path()
        if b2c is None:
            print('  ⚠ bag_to_csv.py 를 못 찾음 — CSV 변환 생략')
        else:
            csv_dir = os.path.join(out_dir, 'csv')
            subprocess.run(['python3', b2c, bag_dir, csv_dir])
            print(f'  CSV → {csv_dir}')
    return out_dir


def main():
    p = argparse.ArgumentParser(description='궤적별 비행 데이터 수집 (rosbag)')
    p.add_argument('--shapes', nargs='+',
                   default=['circle', 'oval', 'figure8', 'clover', 'star'],
                   help='수집할 도형들')
    p.add_argument('--speeds', nargs='+', type=float, default=[1.0],
                   help='목표 최대 속도 목록 [m/s]')
    p.add_argument('--laps', type=int, default=3, help='런당 바퀴 수 (--seconds 없을 때)')
    p.add_argument('--seconds', type=float, default=None,
                   help='런당 목표 스트리밍 시간 [s]. 주면 도형·속도별로 laps 자동 계산')
    p.add_argument('--yaw', choices=['forward', 'constant'], default='forward')
    p.add_argument('--cf', default='cf231', help='기체 이름(토픽 네임스페이스)')
    p.add_argument('--out', default='traj_data', help='출력 루트 폴더')
    p.add_argument('--to-csv', action='store_true', help='bag 을 토픽별 CSV 로 변환')
    p.add_argument('--gt-topic', default='/poses',
                   help='ground-truth 위치 토픽(mocap NamedPoseArray). '
                        "빈 값(--gt-topic '')이면 미기록 — sim 등 mocap 없을 때")
    p.add_argument('--height', type=float, default=None, help='비행 고도 [m] (비행에 전달)')
    p.add_argument('--scale', type=float, default=None, help='도형 크기 배율 (비행에 전달)')
    args, _ = p.parse_known_args()

    extra = []
    if args.height is not None:
        extra += ['--height', str(args.height)]
    if args.scale is not None:
        extra += ['--scale', str(args.scale)]

    out_root = os.path.abspath(args.out)
    print(f'[collect] 출력 루트 {out_root}')
    print(f'[collect] 도형 {args.shapes} × 속도 {args.speeds} '
          f'(laps={args.laps}, yaw={args.yaw}, cf={args.cf})')
    print(f'[collect] GT 위치 토픽: {args.gt_topic or "(미기록)"}')
    print('[collect] ⚠ crazyflie 서버가 떠 있어야 한다(sim 은 IMU/PWM 확장 반영본).')

    done = []
    for shape in args.shapes:
        for speed in args.speeds:
            laps = _laps_for(shape, speed, args.seconds) if args.seconds \
                else args.laps
            done.append(record_one(
                shape, speed, laps, args.yaw, args.cf,
                out_root, extra, args.to_csv, args.gt_topic))
    print(f'\n[collect] 완료 — {len(done)} 조합')
    for d in done:
        print('  ', d)


if __name__ == '__main__':
    main()
