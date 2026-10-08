from pathlib import Path
import numpy as np
import pytest
from crazyflie_imo_racing.controller import PositionController
from crazyflie_imo_racing.motor import collective_pwm, thrust_acceleration
from crazyflie_imo_racing.sensors import SensorPairer
from crazyflie_imo_racing.trajectory import Trajectory, FlightPlan


def test_motor_inverse_and_units():
    pwm = collective_pwm(.034*9.81)
    np.testing.assert_allclose(thrust_acceleration([pwm]*4), [0, 0, 9.81], atol=1e-8)
    np.testing.assert_array_equal(thrust_acceleration([0]*4), [0, 0, 0])


def test_pairing_jitter_missing_and_reset():
    p = SensorPairer(5)
    assert p.add('imu', 10, np.ones(6)) == []
    pairs = p.add('pwm', 12, np.ones(4))
    assert pairs[0][0] == 10000
    p.add('imu', 20, np.ones(6))
    p.add('imu', 30, np.ones(6))
    assert len(p.add('pwm', 30, np.ones(4))) == 1
    assert p.dropped == 1
    with pytest.raises(ValueError, match='reset'):
        p.add('imu', 0, np.ones(6))


@pytest.mark.parametrize('kind', ['circle', 'figure8', 'hover'])
def test_reference_derivatives_and_endpoints(kind):
    t = Trajectory(kind, period=12., laps=2)
    for x in [0., t.duration]:
        p, v, a, _, _ = t.sample(x)
        np.testing.assert_allclose(p, t.home, atol=1e-10)
        np.testing.assert_allclose(v, 0, atol=1e-10)
        np.testing.assert_allclose(a, 0, atol=1e-10)
    dt = 1e-4
    for x in [3., 10., 17.]:
        _, v, a, _, _ = t.sample(x)
        np.testing.assert_allclose((t.sample(x+dt)[0]-t.sample(x-dt)[0])/(2*dt), v, atol=1e-6)
        np.testing.assert_allclose((t.sample(x+dt)[1]-t.sample(x-dt)[1])/(2*dt), a, atol=1e-6)


def test_controller_cflib_legacy_axis_mapping():
    c = PositionController()
    state = dict(rotation=np.eye(3), position=np.zeros(3), velocity=np.zeros(3))
    hover = c.command(state, (np.zeros(3), np.zeros(3), np.zeros(3), 0.))
    assert hover['roll_deg'] == hover['pitch_deg'] == 0
    forward = c.command(state, (np.array([1., 0, 0]), np.zeros(3), np.zeros(3), 0.))
    left = c.command(state, (np.array([0., 1, 0]), np.zeros(3), np.zeros(3), 0.))
    assert forward['pitch_deg'] > 0  # server negates to firmware legacy pitch
    assert left['roll_deg'] < 0
    assert hover['pwm'] < 60000


def test_csv_timescale_and_origin_validation(tmp_path):
    row = np.zeros((1, 33)); row[0, 0] = 2
    row[0, 2] = 1  # x(t)=t
    row[0, 17] = 1  # z=1
    path = tmp_path/'traj.csv'
    np.savetxt(path, row, delimiter=',', header='coefficients', comments='')
    t = Trajectory('csv', csv=str(path), timescale=2)
    p, v, a, _, _ = t.sample(2.)
    np.testing.assert_allclose(p, [1, 0, 1])
    np.testing.assert_allclose(v, [.5, 0, 0])
    np.testing.assert_allclose(a, 0)
    with pytest.raises(ValueError, match='endpoint'):
        Trajectory('csv', csv=str(path), laps=2)
    with pytest.raises(ValueError, match='initial_position'):
        Trajectory('csv', csv=str(path), origin=[1., 0, 0])


def test_flight_plan_continuity():
    plan = FlightPlan(Trajectory('circle'))
    for t in [plan.takeoff, plan.takeoff+plan.settle,
              plan.takeoff+plan.settle+plan.traj.duration]:
        before, after = plan.sample(t-1e-6)[1], plan.sample(t+1e-6)[1]
        for i in range(3):
            np.testing.assert_allclose(before[i], after[i], atol=1e-5)
    assert plan.sample(plan.duration)[0] == 'complete'


def test_actual_model_streaming_jitter_and_gap():
    from crazyflie_imo_racing.estimator import OnlineEstimator
    root = Path(__file__).resolve().parents[1]
    repo = Path('/home/artemis-2/crazyflie/learned_inertial_model_odometry')
    if not repo.exists():
        pytest.skip('Set up the external IMO source repository for this integration test')
    est = OnlineEstimator(repo, root/'models/cfsim_best.pt', root/'models/cfsim_best.json')
    for i in range(90):
        t = i*10000 + (1000 if i % 2 else 0)
        state = est.feed_raw(t, [0, 0, 1, 0, 0, 0], [collective_pwm(.034*9.81)]*4)
    assert est.updates + est.rejected_updates >= 5
    assert np.isfinite(state['position']).all()
    assert est.runner.inputs_buffer.net_t_us.size < 100
    with pytest.raises(ValueError, match='gap'):
        est.feed_raw(1100000, [0, 0, 1, 0, 0, 0], [40000]*4)


def test_pid_attitude_convention_at_nonzero_yaw():
    from crazyflie_imo_racing.controller import firmware_attitude
    from scipy.spatial.transform import Rotation
    for rpy in [[-5, -2, 90], [10, 15, -135], [0, 0, 0]]:
        q = Rotation.from_euler('xyz', rpy, degrees=True).as_quat()
        np.testing.assert_allclose(firmware_attitude(q[[3, 0, 1, 2]]),
                                   np.array(rpy)*[1, -1, 1], atol=1e-10)


def test_forward_yaw_and_rate_match_path_tangent():
    for kind in ('circle', 'figure8'):
        traj = Trajectory(kind)
        for t in [1., 4., 8., 11.]:
            p, v, a, yaw, rate = traj.sample(t)
            np.testing.assert_allclose([np.cos(yaw), np.sin(yaw)], v[:2]/np.linalg.norm(v[:2]), atol=1e-8)
            assert rate == pytest.approx((v[0]*a[1]-v[1]*a[0])/np.dot(v[:2], v[:2]))
