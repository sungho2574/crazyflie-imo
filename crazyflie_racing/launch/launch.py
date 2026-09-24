import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

PACKAGE_NAME = 'crazyflie_racing'


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='gate',
        description='gate(기본, Flow deck·sim: initial_position=start), mocap',
    )
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='cflib',
        description='cflib, cpp, sim 중 하나',
    )

    mode = LaunchConfiguration('mode')
    backend = LaunchConfiguration('backend')

    # gate 설정은 이 패키지, mocap 설정(기체 URI·QTM)은 crazyflie_test 것을 그대로 쓴다
    racing_config = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config')
    test_config = os.path.join(
        get_package_share_directory('crazyflie_test'), 'config')

    crazyflies_yaml_file = PythonExpression(
        ["'", test_config, "/crazyflies_mocap.yaml' if '", mode,
         "' == 'mocap' else '", racing_config, "/crazyflies_gate.yaml'"])
    motion_capture_yaml_file = os.path.join(test_config, 'motion_capture.yaml')

    use_mocap = PythonExpression(
        ["'True' if '", mode, "' == 'mocap' else 'False'"])

    crazyflie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('crazyflie'), 'launch', 'launch.py')
        ),
        launch_arguments={
            'crazyflies_yaml_file': crazyflies_yaml_file,
            'motion_capture_yaml_file': motion_capture_yaml_file,
            'mocap': use_mocap,
            'backend': backend,
        }.items(),
    )

    return LaunchDescription([
        mode_arg,
        backend_arg,
        crazyflie_launch,
    ])
