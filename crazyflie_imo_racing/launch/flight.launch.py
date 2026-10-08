from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    get = lambda k: LaunchConfiguration(k).perform(context)
    share = Path(get_package_share_directory('crazyflie_imo_racing'))
    c = yaml.safe_load(Path(get('config')).expanduser().read_text())['imo_flight']['ros__parameters']
    for key in ('backend', 'trajectory', 'trajectory_csv', 'checkpoint', 'model_parameters', 'imo_repo'):
        if get(key):
            c[key] = get(key)
    for key in ('period', 'radius', 'height', 'timescale'):
        if get(key):
            c[key] = float(get(key))
    if get('laps'):
        c['laps'] = int(get('laps'))
    if get('autostart'):
        c['autostart'] = get('autostart').lower() == 'true'
    if c['backend'] == 'sim' and not get('autostart'):
        c['autostart'] = True
    c['exit_on_complete'] = True
    flight = Node(package='crazyflie_imo_racing', executable='imo_flight', output='screen', parameters=[c],
                  additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'})
    return [flight, RegisterEventHandler(OnProcessExit(target_action=flight,
        on_exit=[EmitEvent(event=Shutdown(reason='IMO flight process ended'))]))]


def generate_launch_description():
    share = Path(get_package_share_directory('crazyflie_imo_racing'))
    defaults = {'config': str(share/'config/flight.yaml'),
                'backend': '', 'trajectory': '', 'trajectory_csv': '', 'checkpoint': '',
                'model_parameters': '', 'imo_repo': '', 'period': '', 'radius': '', 'height': '',
                'timescale': '', 'laps': '', 'autostart': ''}
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
                              OpaqueFunction(function=launch_nodes)])
