"""학습된 레이싱 정책(body-rate PPO actor) — numpy 추론 (JAX·flax 불필요).

체크포인트는 flax `serialization.to_bytes` 로 저장된 msgpack 이다. ndarray 는 msgpack
ext type 1 = (shape, dtype, buffer) 로 인코딩돼 있어 msgpack 만으로 풀 수 있다.

    actor:  obs(49) → Dense(256)+tanh → Dense(256)+tanh → Dense(4) = mean
    action = tanh(mean)                 (평가·배포 시 결정적, 학습 코드 evaluate.py 와 같음)
    command = bias + scale * action     (racing.py RacingEnv.command)
            = [ωx, ωy, ωz  [rad/s, 기체 좌표계],  총추력 [N]]

모델 폴더(models/<이름>/)에 필요한 파일
    best.msgpack         가중치 (필수)
    config.yaml          학습 설정 (필수) — control_mode·control_hz·각속도 한계·관측 옵션 확인용
    best_evaluation.json 학습 평가 (선택) — 있으면 최대 기울기로 rl_flight 안전 기준을 잡는다
"""
import json
import os

import msgpack
import numpy as np
import yaml

# crazyflow cf2x_L250 모터 추력 한계 [N/모터] — 학습 환경(racing.py)의 추력 스케일 기준
MOTOR_THRUST_MIN = 0.0128175784
MOTOR_THRUST_MAX = 0.12
OBS_DIM = 49


def _ext_hook(code, data):
    if code == 1:                                   # flax: ndarray
        shape, dtype, buf = msgpack.unpackb(data)
        return np.frombuffer(buf, dtype=dtype).reshape(shape).astype(np.float64)
    return msgpack.ExtType(code, data)


DEFAULT_MODEL = 'racing-body-rate-10s-dr-final'
EVAL_FILES = ('best_evaluation.json', 'selection_evaluation.json')


def models_root():
    """설치된 share → 소스 트리 순으로 models/ 폴더를 찾는다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('crazyflie_rl'), 'models')
        if os.path.isdir(path):
            return path
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'models')


def list_models():
    """best.msgpack·config.yaml 이 있는 모델 이름들."""
    root = models_root()
    if not os.path.isdir(root):
        return []
    return sorted(d for d in os.listdir(root)
                  if all(os.path.isfile(os.path.join(root, d, f))
                         for f in ('best.msgpack', 'config.yaml')))


def resolve_model(name=None, model_dir=None):
    """모델 폴더 경로. model_dir 가 있으면 그대로, 아니면 이름(정확히 또는 고유한 일부)으로 찾는다.

    예) --model 5s → racing-body-rate-5s-dr-final
    """
    if model_dir:
        return model_dir
    name = name or DEFAULT_MODEL
    names = list_models()
    if name in names:
        return os.path.join(models_root(), name)
    hits = [n for n in names if name in n]
    if len(hits) == 1:
        return os.path.join(models_root(), hits[0])
    raise ValueError(f'모델 "{name}" 을 {"하나로 정할 수 없다" if hits else "찾지 못했다"}. '
                     f'사용 가능: {", ".join(names)}')


def default_model_dir():
    return resolve_model()


class RacingPolicy:
    """body_rate 레이싱 actor. act(obs) → action[-1,1]^4, command(action) → 제어 목표."""

    def __init__(self, model_dir=None):
        model_dir = model_dir or default_model_dir()
        self.name = os.path.basename(os.path.normpath(model_dir))
        with open(os.path.join(model_dir, 'config.yaml')) as f:
            self.cfg = yaml.safe_load(f)
        # racing_obs 가 옮긴 관측·명령과 다른 설정이면 거부 (학습 코드 racing.py 기준)
        if self.cfg.get('control_mode') != 'body_rate':
            raise ValueError(f'body_rate 정책만 지원한다: {self.cfg.get("control_mode")}')
        if self.cfg.get('delay_gate_target_until_clear', True):
            raise ValueError('delay_gate_target_until_clear=true 정책은 관측이 달라 지원하지 않는다')
        self.eval = {}
        for fname in EVAL_FILES:
            path = os.path.join(model_dir, fname)
            if os.path.isfile(path):
                with open(path) as f:
                    self.eval = json.load(f)
                break
        with open(os.path.join(model_dir, 'best.msgpack'), 'rb') as f:
            ckpt = msgpack.unpackb(f.read(), ext_hook=_ext_hook, strict_map_key=False)
        p = ckpt['train']['params']['params']
        self.layers = [(p['actor_0']['kernel'], p['actor_0']['bias']),
                       (p['actor_1']['kernel'], p['actor_1']['bias'])]
        self.out = (p['mean']['kernel'], p['mean']['bias'])
        if self.layers[0][0].shape[0] != OBS_DIM:
            raise ValueError(f'관측 차원이 {OBS_DIM} 이 아니다: {self.layers[0][0].shape}')
        self.iteration = ckpt.get('iteration')

        rates = np.asarray(self.cfg.get('racing_body_rate_rad_s', [12., 12., 8.]), dtype=float)
        lo, hi = 4 * MOTOR_THRUST_MIN, 4 * MOTOR_THRUST_MAX
        self.scale = np.array([*rates, (hi - lo) / 2])
        self.bias = np.array([0., 0., 0., (hi + lo) / 2])
        self.control_hz = int(self.cfg['control_hz'])

    def act(self, obs):
        h = np.asarray(obs, dtype=np.float64)
        for w, b in self.layers:
            h = np.tanh(h @ w + b)
        return np.tanh(h @ self.out[0] + self.out[1])

    def command(self, action):
        """action → [ωx, ωy, ωz (rad/s, 기체 좌표계), 총추력 (N)]."""
        return self.bias + self.scale * np.clip(action, -1.0, 1.0)
