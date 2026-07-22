#!/usr/bin/env python3
"""rosbag2에서 지정 토픽을 CSV로 추출.

사용법:
    python3 bag_to_csv.py <bag_dir> <output_dir>
"""
import argparse
import csv
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    data = {}
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(raw, msg_type)
        data.setdefault(topic, []).append((timestamp, msg))
    return data


def flatten(msg, prefix=''):
    """메시지를 평탄한 dict로 변환."""
    out = {}
    for field in msg.get_fields_and_field_types().keys():
        value = getattr(msg, field)
        if hasattr(value, 'get_fields_and_field_types'):
            out.update(flatten(value, prefix + field + '.'))
        elif isinstance(value, (list, tuple)):
            for i, v in enumerate(value):
                # 배열 원소가 메시지인 경우(예: NamedPoseArray.poses)도 재귀로 평탄화
                if hasattr(v, 'get_fields_and_field_types'):
                    out.update(flatten(v, f'{prefix}{field}.{i}.'))
                else:
                    out[f'{prefix}{field}.{i}'] = v
        else:
            out[prefix + field] = value
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_dir')
    parser.add_argument('output_dir')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    data = read_bag(args.bag_dir)

    for topic, records in data.items():
        if not records:
            continue
        name = topic.strip('/').replace('/', '_') + '.csv'
        path = os.path.join(args.output_dir, name)

        rows = []
        for timestamp, msg in records:
            row = {'timestamp_ns': timestamp}
            row.update(flatten(msg))
            rows.append(row)

        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f'{topic} -> {path} ({len(rows)} rows)')


if __name__ == '__main__':
    main()
