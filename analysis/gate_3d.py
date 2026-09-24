#!/usr/bin/env python3
"""게이트 비행 rosbag 중 **마지막 게이트까지 도달**한 것을 3D 궤적으로 시각화·저장.

각 그림에 그린다:
  · 궤적 (GT /poses, 시간=색)
  · 게이트 7개 (개구부 사각형, gates.yaml 의 위치·yaw·inner_size 로)
  · 땅 격자 4×4 m (z=0, 1 m 간격)

'마지막 게이트 도달' = 궤적 최근접이 마지막 게이트 중심에서 0.6 m 이내.

실행:
    source /opt/ros/humble/setup.bash
    PYTHONNOUSERSITE=1 python3 gate_3d.py      # apt matplotlib(3D 정상)+numpy 로 실행
"""
import glob
import os
import sqlite3

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (3d projection 등록)

from rclpy.serialization import deserialize_message
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from geometry_msgs.msg import PoseStamped

CFG = os.path.expanduser('~/Workspace/cf_ws/src/crazyflie-imo/crazyflie_test/config')
LOGS = os.path.expanduser('~/flight_logs')
OUT = os.path.join(LOGS, 'gate_3d')
GATES_YAML = os.path.join(CFG, 'gates.yaml')
GATE_TRAJ = os.path.join(CFG, 'gate_trajectory.csv')   # 계획(원래) 궤적 = TOGT 다항식
REACH = 0.6      # 마지막 게이트 중심 이 거리 이내면 '도달' [m]


def load_gates():
    y = yaml.safe_load(open(GATES_YAML))
    return y['gates'], y['gate']['inner_size'], y['start']


def read_poses(db):
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    try:
        c = con.cursor()
        r = c.execute("SELECT id FROM topics WHERE name='/poses'").fetchone()
        if not r:
            return None
        P = []
        for _, data in c.execute(
                'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                (r[0],)):
            m = deserialize_message(bytes(data), NamedPoseArray)
            if m.poses:
                p = m.poses[0].pose.position
                P.append([p.x, p.y, p.z])
        return np.array(P) if P else None
    finally:
        con.close()


def read_onboard(db):
    """/cf231/pose (온보드 상태추정) 궤적."""
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    try:
        c = con.cursor()
        r = c.execute("SELECT id FROM topics WHERE name='/cf231/pose'").fetchone()
        if not r:
            return None
        P = []
        for _, data in c.execute(
                'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                (r[0],)):
            m = deserialize_message(bytes(data), PoseStamped)
            p = m.pose.position
            P.append([p.x, p.y, p.z])
        return np.array(P) if P else None
    finally:
        con.close()


def sample_planned(csv_path, n=4000):
    """계획 궤적(gate_trajectory.csv) 다항식을 조각별로 샘플. (gate_course.sample_trajectory 방식)

    CSV = duration, x^0..x^7, y^0..y^7, z^0..z^7, yaw^0..yaw^7 (오름차순). np.polyval 은 내림차순.
    """
    rows = np.atleast_2d(np.loadtxt(csv_path, delimiter=',', skiprows=1, usecols=range(33)))
    total = rows[:, 0].sum()
    pts = []
    for r in rows:
        dur = r[0]
        cx, cy, cz = r[1:9][::-1], r[9:17][::-1], r[17:25][::-1]
        k = max(2, int(round(n * dur / total)))
        for t in np.linspace(0, dur, k):
            pts.append([np.polyval(cx, t), np.polyval(cy, t), np.polyval(cz, t)])
    return np.array(pts)


def gate_square(g, inner):
    """게이트 개구부(수직 사각형)의 꼭짓점 루프와 중심 반환."""
    a = -np.deg2rad(g['yaw_deg'])                 # 통과방향(법선) 방위: x기준 CW → 수학각
    w = np.array([-np.sin(a), np.cos(a), 0.0])    # 수평 폭방향 (법선에 수직)
    up = np.array([0.0, 0.0, 1.0])
    c = np.array([g['x'], g['y'], g['mount_z'] + inner / 2])
    h = inner / 2
    loop = np.array([c + h*w + h*up, c - h*w + h*up,
                     c - h*w - h*up, c + h*w - h*up, c + h*w + h*up])
    return loop, c


def draw_scene(ax, gates, inner, start):
    # 4×4 m 땅 격자 (z=0, 1 m 간격), -2..2
    for v in np.arange(-2, 2.01, 1.0):
        ax.plot([-2, 2], [v, v], [0, 0], color='0.75', lw=.6, zorder=1)
        ax.plot([v, v], [-2, 2], [0, 0], color='0.75', lw=.6, zorder=1)
    ax.plot([-2, 2, 2, -2, -2], [-2, -2, 2, 2, -2], [0, 0, 0, 0, 0],
            color='0.35', lw=1.4, zorder=1)                     # 4×4 외곽
    # 게이트
    for g in gates:
        loop, c = gate_square(g, inner)
        ax.plot(loop[:, 0], loop[:, 1], loop[:, 2], color='tab:green', lw=2, zorder=5)
        ax.text(c[0], c[1], c[2] + 0.18, f"G{g['id']}", color='darkgreen', fontsize=8)
    ax.scatter([start['x']], [start['y']], [0], color='k', marker='^', s=40,
               label='start', zorder=6)


def main():
    os.makedirs(OUT, exist_ok=True)
    gates, inner, start = load_gates()
    g_last = gates[-1]
    G_last = np.array([g_last['x'], g_last['y'], g_last['mount_z'] + inner / 2])

    planned = sample_planned(GATE_TRAJ)              # 계획(원래) 궤적 — 모든 비행 공통

    sel = []
    for d in sorted(glob.glob(os.path.join(LOGS, 'gate_*'))):
        dbs = glob.glob(os.path.join(d, '*.db3'))
        if not dbs:
            continue
        P = read_poses(dbs[0])                        # GT 로 '마지막 게이트 도달' 판정
        if P is None:
            continue
        fly = P[P[:, 2] > 0.3]
        if len(fly) < 10:
            continue
        dmin = float(np.linalg.norm(fly - G_last, axis=1).min())
        if dmin < REACH:
            sel.append((os.path.basename(d), dbs[0], dmin))

    print(f"마지막 게이트(G{g_last['id']}) 도달 비행 {len(sel)}개 → 3D 저장")
    for name, db, dmin in sel:
        onb = read_onboard(db)
        if onb is None:
            print(f'  {name}: 온보드 pose 없음 — 건너뜀'); continue
        onf = onb[onb[:, 2] > 0.3]                    # 비행 구간(z>0.3)

        fig = plt.figure(figsize=(9, 8))
        ax = fig.add_subplot(111, projection='3d')
        draw_scene(ax, gates, inner, start)
        # 원래(계획) 궤적 — 주황 실선
        ax.plot(planned[:, 0], planned[:, 1], planned[:, 2],
                color='tab:orange', lw=2.2, label='planned', zorder=4)
        # 실제 온보드 궤적 — 파랑
        ax.plot(onf[:, 0], onf[:, 1], onf[:, 2],
                color='tab:blue', lw=1.2, label='onboard', zorder=5)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]')
        ax.set_xlim(-2.5, 2.5); ax.set_ylim(-2.5, 2.5); ax.set_zlim(0, 2)
        ax.set_title(f'{name}\nplanned vs onboard  (last-gate min {dmin:.2f} m)', fontsize=10)
        ax.legend(loc='upper left', fontsize=8)
        try:
            ax.set_box_aspect((5, 5, 2))
        except Exception:
            pass
        ax.view_init(elev=25, azim=-120)             # 기존 -60 을 y축 대칭으로 반사
        out = os.path.join(OUT, name + '_planned_vs_onboard.png')
        fig.savefig(out, dpi=110); plt.close(fig)
        print('  saved', out)
    print(f'\n결과 폴더: {OUT}')


if __name__ == '__main__':
    main()
