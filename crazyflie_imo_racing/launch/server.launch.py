from pathlib import Path
import time
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    get = lambda k: LaunchConfiguration(k).perform(context)
    share = Path(get_package_share_directory('crazyflie_imo_racing'))
    c = yaml.safe_load(Path(get('config')).expanduser().read_text())['imo_flight']['ros__parameters']
    if get('backend'):
        c['backend'] = get('backend')
    backend = c['backend']
    if backend == 'sim':
        sim_log = str(Path(c['log_dir']).expanduser()/f'ground_truth_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        server = Node(package='crazyflie_imo_racing', executable='imo_sim', output='screen',
            parameters=[{'robot': c['robot'], 'initial_position': c['initial_position'], 'log_file': sim_log,
                         'firmware_controller': c['firmware_controller']}])
    elif backend == 'cflib':
        if c['sensor_profile'] != 'hardware' or c['autostart']:
            raise ValueError('Hardware requires sensor_profile=hardware and autostart=false before connecting')
        robots = yaml.safe_load(Path(get('crazyflies')).expanduser().read_text())
        robot = robots['robots'][c['robot']]
        controller_id = {'pid': 1, 'mellinger': 2}[c['firmware_controller']]
        robots['all']['firmware_params']['stabilizer']['controller'] = controller_id
        if not robot['enabled'] or any(abs(a-b) > .001 for a, b in zip(robot['initial_position'], c['initial_position'])):
            raise ValueError('Robot must be enabled with matching initial_position in both configs')
        server_share = Path(get_package_share_directory('crazyflie'))
        params = yaml.safe_load((server_share/'config/server.yaml').read_text())['/crazyflie_server']['ros__parameters']
        desc = Path(get_package_share_directory('crazyflie_description'))/'urdf/crazyflie_description.urdf'
        params['robot_description'] = desc.read_text()
        server = Node(package='crazyflie_server_py', executable='crazyflie_server', output='screen',
                      parameters=[robots, params])
    else:
        raise ValueError('backend must be sim or cflib')
    visualization = Node(package='crazyflie_imo_racing', executable='imo_visualization',
                         output='screen', parameters=[c])
    displays = [visualization]
    if get('rviz').lower() == 'true':
        displays.append(Node(package='rviz2', executable='rviz2', output='screen',
                             arguments=['-d', str(share/'config/imo.rviz')]))
    return [server, *displays]


def generate_launch_description():
    share = Path(get_package_share_directory('crazyflie_imo_racing'))
    defaults = {'config': str(share/'config/flight.yaml'),
                'crazyflies': str(share/'config/crazyflies.yaml'), 'backend': '', 'rviz': 'true'}
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
                              OpaqueFunction(function=launch_nodes)])
