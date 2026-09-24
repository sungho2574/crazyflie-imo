#!/usr/bin/env python3
"""선택한 학습 비행을 전처리한다 (pandas 없이 numpy 만).

파이프라인 (데이터셋마다):
  1. 토픽 CSV 로드 (pose / cmd_full_state / motor_pwm / imu_raw / status)
  2. 이착륙 제거: cmd_full_state 스트리밍 구간 [t0,t1] 으로 자른다
     (이륙·goTo·착륙은 High-Level 이라 cmd_full_state 가 안 나감 → 궤적 추종 구간만 남음)
  3. 타임스탬프 통일: [t0,t1] 을 100 Hz 균등 격자로 만들고 각 신호를 보간해 한 표로 병합
  4. unified.csv 저장
품질 점검:
  · pose 타임스탬프 간격(dt) 으로 드롭/지연 탐지 (nominal 대비 큰 gap)
배터리 분석:
  · status 있는 셋에서 battery_voltage 와 모터 출력(mean PWM) 의 시간 추세·상관

실행:  PYTHONNOUSERSITE=1 python3 preprocess_trainset.py
"""
import csv
import os
import re

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

TRAIN = os.path.expanduser('~/Workspace/cf_ws/flight_data/trainset')
OUT = os.path.expanduser('~/Workspace/cf_ws/flight_data/_analysis/preprocess')
RATE = 100.0        # 통일 격자 [Hz]
GAP_K = 2.5         # nominal dt 의 이 배 넘으면 '지연/드롭'

SELECTED = [
    'circle_v1.0_5lap_bag_165219',      # 1
    'circle_v2.0_5lap_bag',             # 2  (battery)
    'clover_v1.0_5lap_bag_171804',      # 5  (battery)
    'clover_v2.0_5lap_bag_171633',      # 8  (battery)
    'figure8_v1.0_5lap_bag_165948',     # 9
    'oval_v1.0_5lap_bag',               # 10
    'oval_v2.0_5lap_bag_171439',        # 11 (battery)
]

_ARR = re.compile(r'\[([^\]]*)\]')


def _load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def _col(rows, key):
    return np.array([float(r[key]) for r in rows])


def _t(rows):
    return np.array([int(r['timestamp_ns']) for r in rows], dtype=np.float64) / 1e9


def _values(rows):
    """'array(\\'f\\', [a, b, ..])' 문자열 컬럼 → (N, k) 배열."""
    out = []
    for r in rows:
        m = _ARR.search(r['values'])
        out.append([float(v) for v in m.group(1).split(',')] if m else [])
    return np.array(out, dtype=np.float64)


def load_dataset(d):
    """csv 폴더에서 필요한 토픽을 읽어 dict 반환 (없는 토픽은 None)."""
    c = os.path.join(TRAIN, d, 'csv')
    def p(name):
        f = os.path.join(c, name)
        return _load(f) if os.path.exists(f) else None
    gt = p('poses.csv')                      # mocap ground-truth (/poses)
    est = p('cf231_pose.csv')                # 온보드 추정 (GT 드롭 구간 교차채움용)
    cmd = p('cf231_cmd_full_state.csv')
    pwm = p('cf231_motor_pwm.csv')
    imu = p('cf231_imu_raw.csv')
    status = p('cf231_status.csv')
    return dict(gt=gt, est=est, cmd=cmd, pwm=pwm, imu=imu, status=status)


def flight_window(cmd):
    """cmd_full_state 스트리밍 구간 = 궤적 추종 구간(이착륙 제외)."""
    t = _t(cmd)
    return float(t[0]), float(t[-1])


def resample(t_src, y_src, grid):
    return np.interp(grid, t_src, y_src)


def unify(ds):
    """[t0,t1] 100Hz 격자에 모든 신호 보간 병합. (grid_rel, table dict, meta) 반환."""
    t0, t1 = flight_window(ds['cmd'])
    grid = np.arange(t0, t1, 1.0 / RATE)
    T = {'t': grid - t0}                      # 0 부터 시작하는 상대시간

    # pose = mocap ground-truth (/poses 의 cf231 = poses.0).
    # ① GT·추정 모두 트래킹 상실 시 (0,0,0) 원점 센티넬을 내므로 그 샘플을 먼저 버린다.
    # ② GT 드롭 구간(실제 미측정)은 그 구간을 덮는 온보드 추정으로 교차채움, gap=1 표시.
    # ③ GT·추정 둘 다 없는 구간은 보간(부드럽게). (header.stamp 재배치는 불가 — 몰아 오는 게 아님)
    def _valid_t(rows, px, py, pz):
        t = _t(rows)
        X, Y, Z = _col(rows, px), _col(rows, py), _col(rows, pz)
        good = ~((np.abs(X) < 0.03) & (np.abs(Y) < 0.03) & (Z < 0.10))   # 원점 센티넬 제거
        return t, good
    tg, gg = _valid_t(ds['gt'], 'poses.0.pose.position.x',
                      'poses.0.pose.position.y', 'poses.0.pose.position.z')
    te, ge = _valid_t(ds['est'], 'pose.position.x',
                      'pose.position.y', 'pose.position.z')
    tgv, tev = tg[gg], te[ge]
    nom = float(np.median(np.diff(tgv)))
    idx = np.clip(np.searchsorted(tgv, grid), 1, len(tgv) - 1)
    gap = (tgv[idx] - tgv[idx - 1]) > GAP_K * nom           # 유효 GT 기준 드롭
    T['gap'] = gap.astype(int)
    for ax in 'xyz':
        g = resample(tgv, _col(ds['gt'], f'poses.0.pose.position.{ax}')[gg], grid)
        e = resample(tev, _col(ds['est'], f'pose.position.{ax}')[ge], grid)
        T['pose_' + ax] = np.where(gap, e, g)
    for q in ('x', 'y', 'z', 'w'):
        g = resample(tgv, _col(ds['gt'], f'poses.0.pose.orientation.{q}')[gg], grid)
        e = resample(tev, _col(ds['est'], f'pose.orientation.{q}')[ge], grid)
        T['quat_' + q] = np.where(gap, e, g)

    tc = _t(ds['cmd'])
    for ax in 'xyz':
        T['cmd_' + ax] = resample(tc, _col(ds['cmd'], f'pose.position.{ax}'), grid)
        T['cmd_v' + ax] = resample(tc, _col(ds['cmd'], f'twist.linear.{ax}'), grid)

    vp = _values(ds['pwm']); tw = _t(ds['pwm'])
    for i in range(vp.shape[1]):
        T[f'm{i + 1}'] = resample(tw, vp[:, i], grid)

    vi = _values(ds['imu']); ti = _t(ds['imu'])
    for i, name in enumerate(['ax', 'ay', 'az', 'gx', 'gy', 'gz'][:vi.shape[1]]):
        T[name] = resample(ti, vi[:, i], grid)

    if ds['status'] is not None:
        tsb = _t(ds['status'])
        T['battery'] = resample(tsb, _col(ds['status'], 'battery_voltage'), grid)

    return grid - t0, T, dict(t0=t0, t1=t1, n=len(grid))


def gap_report(t_ns_sec, name, nominal=None):
    """연속 타임스탬프 dt 로 드롭/지연 탐지."""
    dt = np.diff(t_ns_sec)
    nom = nominal if nominal else float(np.median(dt))
    big = dt > GAP_K * nom
    return dict(topic=name, n=len(t_ns_sec), rate=1.0 / nom if nom else 0,
                dt_med=nom, dt_max=float(dt.max()) if len(dt) else 0,
                n_gap=int(big.sum()), gap_time_s=float(dt[big].sum()) if big.any() else 0.0)


def main():
    os.makedirs(OUT, exist_ok=True)
    quality, batt = [], []

    # ── 이착륙 제거 확인용: 고도 z(t) + 잘라낸 창 ──
    fig1, axes1 = plt.subplots(2, 4, figsize=(16, 6))
    axes1 = axes1.ravel()
    # ── pose dt(지연/드롭) ──
    fig2, axes2 = plt.subplots(2, 4, figsize=(16, 6))
    axes2 = axes2.ravel()

    for k, d in enumerate(SELECTED):
        ds = load_dataset(d)
        t0, t1 = flight_window(ds['cmd'])
        tp = _t(ds['gt'])                                # GT 타임스탬프
        zp = _col(ds['gt'], 'poses.0.pose.position.z')

        # (1) 이착륙 제거 시각화
        ax = axes1[k]
        ax.plot(tp - tp[0], zp, lw=.6, color='gray', label='full')
        m = (tp >= t0) & (tp <= t1)
        ax.plot(tp[m] - tp[0], zp[m], lw=.8, color='tab:blue', label='kept (cmd window)')
        ax.axvspan(t0 - tp[0], t1 - tp[0], color='tab:blue', alpha=.08)
        ax.set_title(d, fontsize=7); ax.set_xlabel('t [s]'); ax.set_ylabel('z')
        ax.grid(alpha=.3)
        if k == 0:
            ax.legend(fontsize=6)

        # (2) 통일 저장
        _, T, meta = unify(ds)
        cols = list(T.keys())
        outcsv = os.path.join(TRAIN, d, 'unified.csv')
        with open(outcsv, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(cols)
            w.writerows(np.column_stack([T[c] for c in cols]))

        # (3) GT(pose) 지연/드롭 — 창 안에서, nominal 은 자동(중앙값)
        g = gap_report(tp[m], 'gt_pose', nominal=None)
        g['dataset'] = d
        quality.append(g)
        dtp = np.diff(tp[m])
        nom_ms = float(np.median(dtp)) * 1000
        axp = axes2[k]
        axp.plot(tp[m][1:] - tp[m][0], dtp * 1000, lw=.5)
        axp.axhline(nom_ms, color='green', ls='--', lw=.6)
        axp.axhline(GAP_K * nom_ms, color='red', ls='--', lw=.6)
        axp.set_title(f"{d}\ngaps: {g['n_gap']}, max {g['dt_max']*1000:.0f}ms", fontsize=6)
        axp.set_xlabel('t [s]'); axp.set_ylabel('GT dt [ms]'); axp.grid(alpha=.3)

        # (4) 배터리 분석용 수집
        if 'battery' in T:
            mean_pwm = np.mean([T[f'm{i}'] for i in (1, 2, 3, 4)], axis=0)
            v = T['battery']; tt = T['t']
            fit_v = np.polyfit(tt, v, 1)[0]
            fit_p = np.polyfit(tt, mean_pwm, 1)[0]
            corr = float(np.corrcoef(v, mean_pwm)[0, 1])
            batt.append(dict(dataset=d, t=tt, v=v, pwm=mean_pwm,
                             dvdt=fit_v, dpdt=fit_p, corr=corr,
                             v0=float(v[0]), v1=float(v[-1])))

    for ax in axes1[len(SELECTED):]:
        ax.axis('off')
    for ax in axes2[len(SELECTED):]:
        ax.axis('off')
    fig1.suptitle('Takeoff/landing removed: gray=full, blue=kept (cmd_full_state window)',
                  fontsize=11)
    fig1.tight_layout(rect=[0, 0, 1, .96]); fig1.savefig(os.path.join(OUT, 'trim.png'), dpi=100)
    fig2.suptitle('pose timestamp gap dt (green=nominal 20ms, red=drop thr 50ms)', fontsize=11)
    fig2.tight_layout(rect=[0, 0, 1, .96]); fig2.savefig(os.path.join(OUT, 'pose_gaps.png'), dpi=100)

    # ── 배터리 vs 출력 ──
    if batt:
        fig3, axes3 = plt.subplots(1, len(batt), figsize=(6 * len(batt), 4), squeeze=False)
        for j, b in enumerate(batt):
            a = axes3[0][j]; a2 = a.twinx()
            a.plot(b['t'], b['v'], color='tab:red', label='battery [V]')
            a2.plot(b['t'], b['pwm'], color='tab:blue', alpha=.6, label='mean PWM')
            a.set_xlabel('t [s]'); a.set_ylabel('battery [V]', color='tab:red')
            a2.set_ylabel('mean motor PWM', color='tab:blue')
            a.set_title(f"{b['dataset']}\ndV/dt={b['dvdt']:.4f} V/s, "
                        f"dPWM/dt={b['dpdt']:.0f}/s, corr={b['corr']:+.2f}", fontsize=8)
            a.grid(alpha=.3)
        fig3.tight_layout(); fig3.savefig(os.path.join(OUT, 'battery_vs_output.png'), dpi=100)

    # ── 리포트 출력 ──
    print('\n===== 통일·저장 =====')
    for d in SELECTED:
        u = os.path.join(TRAIN, d, 'unified.csv')
        n = sum(1 for _ in open(u)) - 1
        print(f'  {d:32s} unified.csv {n} 행 @ {int(RATE)}Hz')

    print('\n===== pose 지연/드롭 =====')
    print(f'  {"dataset":32s} {"n":>5} {"Hz":>5} {"maxdt(ms)":>9} {"gaps":>5} {"lost(s)":>7}')
    for g in quality:
        print(f"  {g['dataset']:32s} {g['n']:>5} {g['rate']:>5.0f} "
              f"{g['dt_max']*1000:>9.0f} {g['n_gap']:>5} {g['gap_time_s']:>7.2f}")

    print('\n===== 배터리 vs 출력 (status 있는 셋만) =====')
    if not batt:
        print('  (없음)')
    for b in batt:
        trend = '전압↓+출력↓(동반)' if b['dvdt'] < 0 and b['dpdt'] < 0 else \
                '전압↓/출력↑(보상)' if b['dvdt'] < 0 < b['dpdt'] else '뚜렷치 않음'
        print(f"  {b['dataset']:32s} V {b['v0']:.2f}->{b['v1']:.2f} "
              f"(dV/dt {b['dvdt']:+.4f}V/s) · dPWM/dt {b['dpdt']:+.0f}/s · "
              f"corr {b['corr']:+.2f} · {trend}")

    # 품질 csv 저장
    with open(os.path.join(OUT, 'quality.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['dataset', 'topic', 'n', 'rate',
                                          'dt_med', 'dt_max', 'n_gap', 'gap_time_s'])
        w.writeheader()
        for g in quality:
            w.writerow(g)
    print(f'\n결과 → {OUT}  (trim.png, pose_gaps.png, battery_vs_output.png, quality.csv)')
    print(f'통일 데이터 → 각 trainset/<셋>/unified.csv')


if __name__ == '__main__':
    main()
