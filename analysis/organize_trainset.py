#!/usr/bin/env python3
"""summary.csv 의 플래그로 비행을 선별해 정리한다.

  OK        →  flight_data/trainset/<shape>_v<speed>_<laps>lap_<bag>/
              (해당 .db3 를 옮기고, metadata 없어도 sqlite3 로 직접 읽어 토픽별 csv 새로 생성)
  그 외     →  flight_data/_rejected/<원경로>/   (bag 그대로 격리)
  남은 껍데기 →  flight_data/_rejected/_leftover/  (빈 폴더·중복 csv)

궤적/속도/바퀴는 폴더명에서 파싱한다: <shape>_<speed>_<laps>  (traj_data* 는 laps=3 기본).

실행:
    source ~/Workspace/cf_ws/install/setup.bash    # 커스텀 메시지 타입(get_message)
    python3 organize_trainset.py
"""
import csv
import os
import shutil
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DATA = os.path.expanduser('~/Workspace/cf_ws/flight_data')
SUMMARY = os.path.join(DATA, '_analysis', 'summary.csv')
TRAIN = os.path.join(DATA, 'trainset')
REJECT = os.path.join(DATA, '_rejected')


def flatten(msg, prefix=''):
    out = {}
    for field in msg.get_fields_and_field_types():
        v = getattr(msg, field)
        if hasattr(v, 'get_fields_and_field_types'):
            out.update(flatten(v, prefix + field + '.'))
        elif isinstance(v, (list, tuple)):
            for i, e in enumerate(v):
                if hasattr(e, 'get_fields_and_field_types'):
                    out.update(flatten(e, f'{prefix}{field}.{i}.'))
                else:
                    out[f'{prefix}{field}.{i}'] = e
        else:
            out[prefix + field] = v
    return out


def bag_to_csv(db3, out_dir):
    """metadata 없어도 되는 sqlite3 직접 변환. 토픽별 csv 생성."""
    con = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
    try:
        cur = con.cursor()
        os.makedirs(out_dir, exist_ok=True)
        for tid, name, typ in cur.execute('SELECT id,name,type FROM topics').fetchall():
            try:
                cls = get_message(typ)
            except Exception:                       # noqa: BLE001
                continue
            rows = []
            for ts, data in cur.execute(
                    'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                    (tid,)):
                r = {'timestamp_ns': ts}
                r.update(flatten(deserialize_message(bytes(data), cls)))
                rows.append(r)
            if not rows:
                continue
            fn = name.strip('/').replace('/', '_') + '.csv'
            with open(os.path.join(out_dir, fn), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
    finally:
        con.close()


def parse_meta(label):
    """label = <root>/<combo>/<bagdir> → (shape, speed, laps, bag)."""
    root, combo, bagdir = label.split('/')
    shape = combo.split('_maxSpeed')[0]
    speed = float(combo.split('maxSpeed')[1].replace('p', '.'))
    toks = root.split('_')
    laps = int(toks[-1]) if toks[-1].isdigit() else 3        # traj_data* → 기본 3랩
    return shape, speed, laps, bagdir


def main():
    rows = list(csv.DictReader(open(SUMMARY)))
    os.makedirs(TRAIN, exist_ok=True)
    os.makedirs(REJECT, exist_ok=True)

    table = []
    for r in rows:
        label, flag, db3 = r['label'], r['flag'], r['db3']
        bag_dir = os.path.dirname(db3)                        # .db3 가 든 폴더
        if not os.path.isdir(bag_dir):
            continue
        shape, speed, laps, bag = parse_meta(label)

        if flag == 'OK':
            name = f'{shape}_v{speed:.1f}_{laps}lap_{bag}'
            dest = os.path.join(TRAIN, name)
            shutil.move(bag_dir, os.path.join(dest, 'bag'))
            new_db3 = os.path.join(dest, 'bag', os.path.basename(db3))
            bag_to_csv(new_db3, os.path.join(dest, 'csv'))
            table.append((name, shape, speed, laps,
                          float(r['dur']), float(r['flight']), int(r['n'])))
            print(f'  [OK→trainset] {name}')
        else:
            dest = os.path.join(REJECT, label.replace('/', '__'))
            shutil.move(bag_dir, dest)
            print(f'  [{flag}→rejected] {label.replace("/", "__")}')

    # 남은 껍데기(빈 폴더·중복 csv) 격리
    leftover = os.path.join(REJECT, '_leftover')
    os.makedirs(leftover, exist_ok=True)
    for root in sorted(os.listdir(DATA)):
        p = os.path.join(DATA, root)
        if root in ('trainset', '_rejected', '_analysis') or not os.path.isdir(p):
            continue
        shutil.move(p, os.path.join(leftover, root))

    # 표 출력
    print('\n===== 학습셋(OK) 표 =====')
    hdr = ('name', 'shape', 'speed', 'laps', 'rec_s', 'flight_s', 'samples')
    print('{:<34} {:<8} {:>5} {:>4} {:>7} {:>8} {:>8}'.format(*hdr))
    for t in sorted(table):
        print('{:<34} {:<8} {:>5.1f} {:>4d} {:>7.1f} {:>8.1f} {:>8d}'.format(*t))
    print(f'\n학습셋 {len(table)} 개 → {TRAIN}')
    print(f'격리 {len(rows) - len(table)} 개 → {REJECT}')

    # 표를 csv 로도 저장
    with open(os.path.join(TRAIN, 'trainset_index.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(hdr)
        w.writerows(sorted(table))


if __name__ == '__main__':
    main()
