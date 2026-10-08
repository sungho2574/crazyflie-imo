"""Read-only offline estimator validation on an existing IMO data.hdf5 sequence."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .estimator import OnlineEstimator


def main():
    import h5py
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--imo-repo', required=True)
    p.add_argument('--sequence', required=True, help='HDF5 data file')
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--model-parameters', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--seconds', type=float, default=10.)
    args = p.parse_args()
    with h5py.File(args.sequence) as f:
        ts = f['ts'][:].ravel()
        n = min(len(ts), int(args.seconds*100))
        gyro, accel, thrust = f['gyro_raw'][:n], f['accel_raw'][:n], f['i_thrust'][:n]
        gt = f['traj_target'][:n]
        velocity = f['vel'][:n] if 'vel' in f else np.gradient(gt[:, :3], ts[:n], axis=0)
    est = OnlineEstimator(args.imo_repo, args.checkpoint, args.model_parameters, gt[0, :3])
    est.set_initial_attitude(Rotation.from_quat(gt[0, 3:7]).as_matrix())
    poses = []
    start = time.monotonic()
    for i in range(n):
        state = est.feed_si(int(round((ts[i]-ts[0])*1e6)), gyro[i], accel[i], thrust[i])
        if i == 0:
            # Offline replay only: match the original benchmark's initial GT
            # velocity. FlightNode never reads ground truth or this code path.
            est.runner.filter.state.s_v = velocity[0].reshape(3, 1).copy()
        poses.append(state['position'])
    error = np.linalg.norm(np.array(poses)-gt[:, :3], axis=1)
    result = dict(samples=n, learned_updates=est.updates, ate_m=float(np.sqrt(np.mean(error**2))),
                  wall_seconds=time.monotonic()-start, initialization='GT pose/velocity at t0 only')
    Path(args.output).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
