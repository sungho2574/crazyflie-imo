#!/usr/bin/env python3
"""게이트 코스 → 크플 궤적 CSV (오프라인 경로계획 파이프라인).

    config/gates.yaml  ──►  TOGT track yaml  ──►  togt_plan  ──►  config/gate_trajectory.csv
                                             (시간최적 MINCO)      (펌웨어가 실행하는 다항식)

TOGT-Planner(FSC-Lab)를 오프라인 도구로만 쓴다. 이 스크립트가
    1. gates.yaml 을 TOGT 레이스트랙 yaml 로 변환하고
    2. togt_tools 의 togt_plan 을 (필요하면) 빌드한 뒤 실행하고
    3. 결과 다항식 CSV 를 검사(방 경계 / 게이트 프레임 여유 / 조각 수 / 최대 속도)한다.

TOGT 소스는 별도로 clone 해 둔다(서브모듈로 넣지 않음):
    git clone https://github.com/FSC-Lab/TOGT-Planner

사용:
    python3 -m crazyflie_test.plan_gate_trajectory --togt-dir ~/TOGT-Planner
    # 빌드 후에는: ros2 run crazyflie_test plan_gate_trajectory --togt-dir ~/TOGT-Planner
    # 또는 환경변수 TOGT_DIR 로 지정

게이트를 바꿨을 때만 다시 돌리면 된다. 결과 CSV 는 패키지에 커밋돼 있어,
평소 비행에는 TOGT 빌드가 필요 없다.
"""
import argparse
import os
import subprocess
import sys

import numpy as np

from . import gate_course as gc


def pkg_dir():
    """소스 트리 기준 crazyflie_test 패키지 루트 (togt_tools/, config/ 가 있는 곳).

    이 파일은 crazyflie_test/crazyflie_test/ 안에 있으므로 두 번 올라간다.
    symlink-install 이면 __file__ 이 소스를 가리켜 소스 트리 루트가 나온다.
    """
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def gates_to_track(course, track_path):
    """gates.yaml(Course) → TOGT 레이스트랙 yaml.

    TOGT 게이트 자세 규약: rpy=[roll, pitch, yaw] 를 body→world 로 적용하고, 개구부
    평면은 body xy, 법선은 body z 다. 우리 게이트는 수직 개구부에 법선이 수평
    yaw_deg(CW) 이므로 pitch=-90 으로 평면을 세우고, world yaw 를 180-yaw_deg 로 맞춘다.
    (게이트 통과 방향 = (cos(-yaw_deg), sin(-yaw_deg)) 와 일치하도록.)
    """
    hover = course.start_hover().tolist()
    inner = course.inner_size
    lines = []

    def state(tag):
        lines.extend([
            f'{tag}:',
            '  pos: [%g, %g, %g]' % (hover[0], hover[1], hover[2]),
            '  vel: [0, 0, 0]', '  acc: [0, 0, 0]', '  jer: [0, 0, 0]',
            '  rot: [1, 0, 0, 0]', '  cthrustmass: 9.8066', '  euler: [0, 0, 0]', ''])

    state('initState')
    state('endState')

    # TOGT 유효 창 = inner − marginW. 궤적(기체 중심)이 이 창 안으로만 지나도록
    # 좁혀, 프로펠러 끝이 프레임에서 frame_margin 이상 뜨게 한다(gates.yaml 참고).
    margin = inner - 2.0 * course.traversal_half
    names = ['Gate%d' % g.id for g in course.gates]
    lines.append('orders: [%s]' % ', '.join("'%s'" % n for n in names))
    for g in course.gates:
        z = g.mount_z + inner / 2.0
        lines += [
            '', 'Gate%d:' % g.id,
            "  type: 'RectanglePrisma'",
            "  name: 'cf_gate'",
            '  position: [%g, %g, %g]' % (g.x, g.y, z),
            '  rpy: [0.0, -90.0, %g]' % (180.0 - g.yaw_deg),
            '  width: %g' % inner,
            '  height: %g' % inner,
            '  marginW: %g' % margin,
            '  marginH: %g' % margin,
            '  length: 0.0', '  midpoints: 0', '  stationary: true']
    with open(track_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def ensure_togt_plan(togt_dir, tools_dir):
    """togt_plan 실행파일 경로. 없으면 빌드한다.

    TOGT 소스는 기본적으로 togt_tools 의 **git 서브모듈** TOGT-Planner/ 를 쓴다.
    --togt-dir / 환경변수 TOGT_DIR 가 있으면 그걸 우선한다.
    """
    build_dir = os.path.join(tools_dir, 'build')
    exe = os.path.join(build_dir, 'togt_plan')
    if os.path.exists(exe):
        return exe
    submodule = os.path.join(tools_dir, 'TOGT-Planner')
    togt_dir = togt_dir or submodule
    if not os.path.isdir(os.path.join(togt_dir, 'include', 'drolib')):
        sys.exit(
            f'TOGT-Planner 소스를 찾지 못했다({togt_dir}).\n'
            '  서브모듈을 받으세요:  git submodule update --init --recursive\n'
            '  또는 --togt-dir / 환경변수 TOGT_DIR 로 경로를 지정하세요.')
    print(f'[plan] togt_plan 빌드: {build_dir} (TOGT_DIR={togt_dir})')
    subprocess.run(
        ['cmake', '-S', tools_dir, '-B', build_dir, f'-DTOGT_DIR={togt_dir}'],
        check=True)
    subprocess.run(['cmake', '--build', build_dir, '-j'], check=True)
    return exe


def validate(course, csv_path):
    """gate_flight 가 실행 전에 하는 것과 같은 검사(gate_course.preview 재사용)."""
    n_pieces = len(np.atleast_2d(
        np.loadtxt(csv_path, delimiter=',', skiprows=1, usecols=range(33))))
    pts = gc.sample_trajectory(csv_path)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    info = gc.preview(pts, course, home_xy=course.start[:2])
    print(f'[check] 조각 수      : {n_pieces} (펌웨어 궤적 메모리 한계 ~32 이하 권장)'
          f' -> {"OK" if n_pieces <= 32 else "확인 필요"}')
    print(f'[check] path x/y/z   : x[{lo[0]:.2f},{hi[0]:.2f}] '
          f'y[{lo[1]:.2f},{hi[1]:.2f}] z[{lo[2]:.2f},{hi[2]:.2f}]')
    print(f'[check] 벽까지 여유  : {info["wall"]:.2f} m '
          f'(기준 {course.room.safety_margin:.2f}) -> '
          f'{"OK" if info["wall"] >= course.room.safety_margin else "NG"}')
    print(f'[check] 방 이탈      : {info["escaped"]} 점 -> '
          f'{"OK" if not info["escaped"] else "NG"}')
    # 게이트별 프로펠러 끝 ~ 프레임 여유 (= 중심여유 − drone_radius)
    edge = gc.gate_edge_clearance(pts, course)
    worst = min(edge.values()) if edge else float('inf')
    print(f'[check] 프레임 여유  : 최소 {worst:.3f} m (목표 {course.frame_margin:.2f} m, '
          f'프로펠러 끝 기준) -> {"OK" if worst >= course.frame_margin - 0.01 else "NG"}')
    for gid, m in sorted(edge.items()):
        print(f'            G{gid}: {m:+.3f} m')
    for w in info['warnings']:
        print(f'  ⚠ {w}')


def main():
    p = argparse.ArgumentParser(description='게이트 코스 → 크플 궤적 CSV (TOGT 오프라인)')
    p.add_argument('--gates', default=None, help='gates.yaml (기본: 패키지 config)')
    p.add_argument('--out', default=None,
                   help='출력 CSV (기본: 패키지 config/gate_trajectory.csv)')
    p.add_argument('--togt-dir', default=os.environ.get('TOGT_DIR', ''),
                   help='clone 한 TOGT-Planner 경로 (또는 환경변수 TOGT_DIR)')
    args, _ = p.parse_known_args()

    root = pkg_dir()
    tools_dir = os.path.join(root, 'togt_tools')
    course = gc.load_course(args.gates)
    out_csv = args.out or os.path.join(root, 'config', 'gate_trajectory.csv')
    track_path = os.path.join(tools_dir, 'build', 'gate_track.yaml')
    os.makedirs(os.path.dirname(track_path), exist_ok=True)

    print('[plan] gates.yaml → TOGT 트랙 변환')
    gates_to_track(course, track_path)

    exe = ensure_togt_plan(args.togt_dir, tools_dir)
    print(f'[plan] 계획 실행: {exe}')
    subprocess.run(
        [exe, os.path.join(tools_dir, 'params'), 'cf_setups.yaml',
         track_path, out_csv], check=True)

    print('[plan] 검증')
    validate(course, out_csv)
    print(f'[plan] 완료 → {out_csv}')


if __name__ == '__main__':
    main()
