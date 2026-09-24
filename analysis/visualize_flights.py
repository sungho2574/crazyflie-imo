#!/usr/bin/env python3
"""수집한 비행 rosbag(.db3) 을 한 번에 훑어 궤적·고도를 시각화하고 품질을 선별한다.

각 .db3(개별 비행 기록)마다 `/cf231/pose` 를 **sqlite3 로 직접** 읽어(metadata.yaml
없어도 됨) 위치 시계열을 뽑고, 아래 세트를 PNG 로 저장한다:
    [궤적 XY] + [궤적 3D] + [고도 z(t) 시계열]
전체를 한 장에 요약한 overview 두 장(XY / 고도)과, 자동 품질 플래그가 담긴
summary.csv 도 만든다.

품질 플래그(비행이 제대로 됐는지 한눈에 거르기용):
    OK          정상
    FEW         pose 샘플이 너무 적음(<100)
    NO_TAKEOFF  이륙 흔적 없음(z 변화 < 임계)
    SHORT       기록은 긴데 실제 비행(공중) 시간이 짧음
    DIVERGED    상태추정 발산(위치가 방 밖으로, |xy|>5m 또는 z>3m)
    NO_POSE     /cf231/pose 토픽이 없음
    READ_ERR    읽기 실패(손상 등)

실행 (ROS + apt matplotlib 스택 필요):
    source /opt/ros/humble/setup.bash
    python3 visualize_flights.py                    # 기본 ~/Workspace/cf_ws/flight_data
    python3 visualize_flights.py --data-dir <경로> --out-dir <경로>
"""
import os
import sys

# ~/.local 에 설치된 최신 numpy/scipy/matplotlib 가 시스템 numpy 1.21 과 충돌한다.
# user-site 를 끄고(=apt 호환 스택) 스스로 다시 실행한다.
if os.environ.get('PYTHONNOUSERSITE') != '1':
    os.environ['PYTHONNOUSERSITE'] = '1'
    os.execv(sys.executable, [sys.executable] + sys.argv)

import argparse
import csv
import glob
import math

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (3d projection 등록용)

import sqlite3
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

POSE_TOPIC = '/cf231/pose'
AIR = 0.20          # 이륙 판정: 지면(z_min) 대비 이 이상 높으면 '공중' [m]
FEW = 100           # 이보다 pose 샘플 적으면 FEW
ROOM = 5.0          # |x|,|y| 이 넘으면 발산으로 본다 [m]
ZMAX_OK = 3.0       # z 가 이 넘으면 발산 [m]
TARGET = 6000       # bag 당 최대 사용 점 수(다운샘플). 12만 점 3D 플롯이 죽는 것 방지


def read_pose(db3):
    """.db3 에서 /cf231/pose 를 직접 읽어 (t_ns, x, y, z) 반환. 없으면 None.

    큰 기록은 TARGET 개로 균등 다운샘플한다(역직렬화·플롯 비용 폭발 방지).
    비유한(nan/inf) 점은 버린다.
    """
    con = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
    try:
        cur = con.cursor()
        row = cur.execute('SELECT id FROM topics WHERE name=?', (POSE_TOPIC,)).fetchone()
        if row is None:
            return None
        tid = row[0]
        cnt = cur.execute('SELECT count(*) FROM messages WHERE topic_id=?', (tid,)).fetchone()[0]
        if cnt == 0:
            return None
        stride = max(1, cnt // TARGET)
        t, x, y, z = [], [], [], []
        for i, (ts, data) in enumerate(cur.execute(
                'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                (tid,))):
            if i % stride:
                continue
            m = deserialize_message(bytes(data), PoseStamped)
            t.append(ts)
            x.append(m.pose.position.x)
            y.append(m.pose.position.y)
            z.append(m.pose.position.z)
        t = np.array(t, dtype=np.float64)
        x, y, z = np.array(x), np.array(y), np.array(z)
        good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        t, x, y, z = t[good], x[good], y[good], z[good]
        if len(t) < 2:
            return None
        return (t, x, y, z, cnt)
    finally:
        con.close()


def analyze(t_ns, x, y, z, n):
    """시계열에서 요약 통계와 품질 플래그 계산. n=실제(원본) 샘플 수."""
    t = (t_ns - t_ns[0]) / 1e9
    dur = float(t[-1]) if len(t) > 1 else 0.0
    zmin, zmax = float(z.min()), float(z.max())
    air = z > (zmin + AIR)
    if air.any():
        idx = np.where(air)[0]
        flight = float(t[idx[-1]] - t[idx[0]])       # 첫 공중 ~ 마지막 공중 구간
    else:
        flight = 0.0
    path = float(np.sum(np.hypot(np.diff(x), np.diff(y)))) if n > 1 else 0.0
    span = float(max(np.ptp(x), np.ptp(y))) if n else 0.0
    rate = n / dur if dur > 0 else 0.0

    flags = []
    if n < FEW:
        flags.append('FEW')
    if zmax - zmin < AIR:
        flags.append('NO_TAKEOFF')
    if dur > 0 and (flight < 3.0 or flight / dur < 0.3):
        flags.append('SHORT')
    if max(abs(x).max(), abs(y).max()) > ROOM or zmax > ZMAX_OK:
        flags.append('DIVERGED')
    flag = 'OK' if not flags else '+'.join(flags)
    return dict(n=n, dur=dur, rate=rate, zmin=zmin, zmax=zmax,
                flight=flight, path=path, span=span, flag=flag, t=t)


def label_of(db3, data_dir):
    """경로에서 사람이 읽을 데이터셋 라벨 생성: <root>/<combo>/<bagdir>."""
    rel = os.path.relpath(db3, data_dir)
    parts = rel.split(os.sep)
    root = parts[0]
    combo = next((p for p in parts if 'maxSpeed' in p), parts[-2])
    bagdir = parts[-2]
    return f'{root}/{combo}/{bagdir}'


def plot_one(label, x, y, z, st, out_png):
    """한 비행: XY 궤적 + 3D 궤적 + 고도 시계열 세트."""
    fig = plt.figure(figsize=(15, 4.6))

    ax1 = fig.add_subplot(1, 3, 1)
    sc = ax1.scatter(x, y, c=st['t'], cmap='viridis', s=4)
    ax1.plot(x[0], y[0], 'o', color='lime', ms=9, label='start')
    ax1.plot(x[-1], y[-1], 's', color='red', ms=8, label='end')
    ax1.set_aspect('equal', 'datalim')
    ax1.set_xlabel('x [m]'); ax1.set_ylabel('y [m]')
    ax1.set_title('Trajectory (XY, color=time)')
    ax1.legend(loc='best', fontsize=7); ax1.grid(alpha=.3)
    fig.colorbar(sc, ax=ax1, label='t [s]', shrink=.85)

    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    ax2.plot(x, y, z, lw=.7)
    ax2.scatter(x[0], y[0], z[0], color='lime', s=25)
    ax2.scatter(x[-1], y[-1], z[-1], color='red', s=25)
    ax2.set_xlabel('x'); ax2.set_ylabel('y'); ax2.set_zlabel('z [m]')
    ax2.set_title('Trajectory (3D)')

    ax3 = fig.add_subplot(1, 3, 3)
    ax3.plot(st['t'], z, lw=.7)
    ax3.axhline(st['zmin'] + AIR, color='orange', ls='--', lw=.8, label='takeoff thr')
    ax3.set_xlabel('t [s]'); ax3.set_ylabel('altitude z [m]')
    ax3.set_title('Altitude vs time')
    ax3.grid(alpha=.3); ax3.legend(fontsize=7)

    color = 'green' if st['flag'] == 'OK' else 'crimson'
    fig.suptitle(
        f"{label}   [{st['flag']}]\n"
        f"rec {st['dur']:.1f}s / flight {st['flight']:.1f}s · "
        f"{st['n']} samples {st['rate']:.0f}Hz · "
        f"z[{st['zmin']:.2f},{st['zmax']:.2f}] · xy-span {st['span']:.2f}m",
        fontsize=10, color=color)
    fig.tight_layout(rect=[0, 0, 1, 0.90])
    fig.savefig(out_png, dpi=90)
    plt.close(fig)


def plot_overview(items, out_png, kind):
    """모든 비행을 한 장에. kind='xy' 궤적 그리드 / kind='alt' 고도 그리드."""
    n = len(items)
    cols = 6
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 2.6, rows * 2.4))
    axes = np.atleast_1d(axes).ravel()
    for ax, it in zip(axes, items):
        lab, x, y, z, st = it['label'], it['x'], it['y'], it['z'], it['st']
        ok = st['flag'] == 'OK'
        if kind == 'xy':
            ax.plot(x, y, lw=.6, color='tab:blue' if ok else 'tab:red')
            ax.plot(x[0], y[0], 'o', color='lime', ms=3)
            ax.set_aspect('equal', 'datalim')
        else:
            ax.plot(st['t'], z, lw=.6, color='tab:blue' if ok else 'tab:red')
            ax.axhline(st['zmin'] + AIR, color='orange', ls='--', lw=.5)
        short = lab.split('/')[0] + '/' + lab.split('/')[-1]
        ax.set_title(f"{short}\n[{st['flag']}] {st['flight']:.0f}/{st['dur']:.0f}s",
                     fontsize=6, color='green' if ok else 'crimson')
        ax.tick_params(labelsize=5)
    for ax in axes[n:]:
        ax.axis('off')
    fig.suptitle(f"ALL FLIGHTS — {'XY trajectory' if kind == 'xy' else 'altitude z(t)'} "
                 f"(blue=OK, red=flagged)  n={n}", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description='비행 rosbag 궤적·고도 일괄 시각화/선별')
    p.add_argument('--data-dir', default=os.path.expanduser('~/Workspace/cf_ws/flight_data'))
    p.add_argument('--out-dir', default=None, help='기본: <data-dir>/_analysis')
    args = p.parse_args()

    data_dir = os.path.abspath(args.data_dir)
    out_dir = args.out_dir or os.path.join(data_dir, '_analysis')
    fig_dir = os.path.join(out_dir, 'figures')
    os.makedirs(fig_dir, exist_ok=True)

    db3s = sorted(glob.glob(os.path.join(data_dir, '**', '*.db3'), recursive=True))
    print(f'[viz] {data_dir} 에서 .db3 {len(db3s)} 개 발견')

    rows, items = [], []
    for i, db3 in enumerate(db3s, 1):
        label = label_of(db3, data_dir)
        try:
            res = read_pose(db3)
        except Exception as e:                       # noqa: BLE001
            print(f'  [{i}/{len(db3s)}] {label}  READ_ERR: {e}')
            rows.append(dict(label=label, flag='READ_ERR', n=0, dur=0, flight=0,
                             rate=0, zmin=0, zmax=0, span=0, path=0, db3=db3))
            continue
        if res is None:
            print(f'  [{i}/{len(db3s)}] {label}  NO_POSE')
            rows.append(dict(label=label, flag='NO_POSE', n=0, dur=0, flight=0,
                             rate=0, zmin=0, zmax=0, span=0, path=0, db3=db3))
            continue
        t_ns, x, y, z, n_true = res
        st = analyze(t_ns, x, y, z, n_true)
        fname = label.replace('/', '__') + '.png'
        plot_one(label, x, y, z, st, os.path.join(fig_dir, fname))
        items.append(dict(label=label, x=x, y=y, z=z, st=st))
        rows.append(dict(label=label, flag=st['flag'], n=st['n'], dur=st['dur'],
                         flight=st['flight'], rate=st['rate'], zmin=st['zmin'],
                         zmax=st['zmax'], span=st['span'], path=st['path'], db3=db3))
        print(f"  [{i}/{len(db3s)}] {label}  [{st['flag']}]  "
              f"rec {st['dur']:.0f}s flight {st['flight']:.0f}s z[{st['zmin']:.2f},{st['zmax']:.2f}]")

    if items:
        plot_overview(items, os.path.join(out_dir, 'overview_xy.png'), 'xy')
        plot_overview(items, os.path.join(out_dir, 'overview_alt.png'), 'alt')

    # summary.csv
    cols = ['label', 'flag', 'n', 'dur', 'flight', 'rate',
            'zmin', 'zmax', 'span', 'path', 'db3']
    with open(os.path.join(out_dir, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{r[k]:.3f}' if isinstance(r[k], float) else r[k]) for k in cols})

    # 콘솔 요약
    print('\n===== 요약 (플래그별) =====')
    by = {}
    for r in rows:
        by.setdefault(r['flag'], []).append(r['label'])
    for flag in sorted(by):
        print(f'  {flag:12s} {len(by[flag]):2d} 개')
    ok = [r for r in rows if r['flag'] == 'OK']
    print(f'\n정상(OK) {len(ok)} / 전체 {len(rows)}')
    print(f'결과: {out_dir}')
    print('  · figures/<데이터셋>.png  (개별 궤적+고도 세트)')
    print('  · overview_xy.png / overview_alt.png  (전체 한 장)')
    print('  · summary.csv  (플래그·통계 표)')


if __name__ == '__main__':
    main()
