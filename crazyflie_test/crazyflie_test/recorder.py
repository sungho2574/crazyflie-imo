"""비행 스크립트 공용 rosbag 기록기 (crazyflie_racing · crazyflie_traj 가 쓴다).

    rec = Recorder(bag_dir, topics)   # 백그라운드로 ros2 bag record 시작
    ...비행...
    rec.stop()                        # SIGINT 로 bag 을 깔끔히 닫는다

저장 형식은 sqlite3 고정: jazzy 기본(mcap)이어도 analysis/ 스크립트와 bag_to_csv.py
가 읽는 .db3 로 남긴다. 없는 토픽(예: mocap 이 아닐 때 /poses)은 bag 에서 빠질 뿐이다.
"""
import os
import signal
import subprocess


class Recorder:

    def __init__(self, bag_dir, topics):
        self.bag_dir = bag_dir
        os.makedirs(os.path.dirname(bag_dir) or '.', exist_ok=True)
        # 별도 프로세스 그룹: 터미널 Ctrl+C 가 레코더를 먼저 죽이지 않게 하고,
        # stop() 에서 그룹 전체에 SIGINT 를 보내 flush 후 종료시킨다
        self.proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', bag_dir] + list(topics),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        print(f'  ● 기록 시작 → {bag_dir}')

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        print(f'  ■ 기록 종료 → {self.bag_dir}')
