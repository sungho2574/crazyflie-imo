"""ROS message/lifecycle tests without starting nodes or touching a radio."""
from types import SimpleNamespace
from unittest.mock import Mock
import time
import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('crazyflie_interfaces.srv')
from crazyflie_imo_racing.flight_node import FlightNode


def make_bootstrap(tmp_path):
    future = Mock()
    future.done.return_value = True
    return SimpleNamespace(
        bootstrap_land_us=None, takeoff_future=None, start_us=0,
        cfg={'takeoff_duration': 4., 'settle_duration': 2., 'land_duration': 4.},
        align_start_us=None, align_future=None,
        goto_client=SimpleNamespace(call_async=lambda req: future),
        trajectory=SimpleNamespace(home=np.array([0., 0., 1.]),
                                   sample=lambda t: (None, None, None, .7, 0.)),
        set_phase=Mock(), estimator=SimpleNamespace(set_initial_attitude=Mock()),
        handoff_us=None, bootstrap_pose=(0, np.array([0., 0., 1.]), np.eye(3)),
        bootstrap_velocity=np.zeros(3), pose_wall=time.monotonic(), folder=tmp_path)


def test_estimator_starts_only_after_onboard_takeoff_and_settle(tmp_path):
    b = make_bootstrap(tmp_path)
    FlightNode.bootstrap(b, 1000000)
    assert b.handoff_us is None
    b.estimator.set_initial_attitude.assert_not_called()
    FlightNode.bootstrap(b, 4000000)
    FlightNode.bootstrap(b, 6000000)
    assert b.handoff_us is None
    FlightNode.bootstrap(b, 7010000)
    assert b.handoff_us == 7010000
    np.testing.assert_allclose(b.estimator.initial_position, [0, 0, 1])
    b.estimator.set_initial_attitude.assert_called_once()
    assert (tmp_path/'handoff.json').is_file()


def test_unsettled_pose_prevents_handoff(tmp_path):
    b = make_bootstrap(tmp_path)
    FlightNode.bootstrap(b, 4000000)
    b.bootstrap_velocity = np.array([1., 0, 0])
    with pytest.raises(ValueError, match='not settled'):
        FlightNode.bootstrap(b, 7010000)
    assert b.handoff_us is None


def test_onboard_pose_is_ignored_after_handoff():
    b = SimpleNamespace(handoff_us=1000000, bootstrap_pose='unchanged')
    # No fields of the incoming pose are even read after handoff.
    FlightNode.pose_cb(b, object())
    assert b.bootstrap_pose == 'unchanged'


def test_firmware_manual_control_does_not_use_position_or_velocity():
    firm = pytest.importorskip('cffirmware')
    from crazyflie_sim.crazyflie_sil import CrazyflieSIL
    from crazyflie_sim.sim_data_types import State
    outputs = []
    for position, velocity in [([0, 0, 0], [0, 0, 0]), ([10, -5, 2], [3, 4, -1])]:
        cf = CrazyflieSIL('test', [0., 0., 0.], 'mellinger', lambda: .002)
        cf.setState(State(position, velocity))
        cf.mode = CrazyflieSIL.MODE_LOW_FULLSTATE
        s = cf.setpoint
        s.mode.x = s.mode.y = s.mode.z = firm.modeDisable
        s.mode.roll = s.mode.pitch = firm.modeAbs
        s.mode.yaw = firm.modeVelocity
        s.mode.quat = firm.modeDisable
        s.attitude.roll, s.attitude.pitch = 5., -3.
        s.attitudeRate.yaw = 20.
        s.thrust = 49000.
        outputs.append(cf.executeController().rpm)
    np.testing.assert_allclose(outputs[0], outputs[1], atol=1e-6)
