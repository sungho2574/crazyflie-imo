import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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

    markers_arg = DeclareLaunchArgument(
        'markers',
        default_value='true',
        description='gate_markers(게이트·계획 궤적·실궤적 마커) 실행',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='gate_course.rviz 로 rviz2 실행',
    )
    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        default_value='',
        description='마커로 그릴 궤적 CSV. 비우면 패키지 config/gate_trajectory.csv',
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='true 면 연속 비행 궤적(gate_loop_*.csv)을 그린다 (gate_flight --loop 용)',
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

    # 게이트·궤적 마커. 노드가 하나만 떠야 rviz 경로가 깜빡이지 않는다
    gate_markers = Node(
        package=PACKAGE_NAME,
        executable='gate_markers',
        name='gate_markers',
        output='screen',
        parameters=[{
            'trajectory': ParameterValue(
                LaunchConfiguration('trajectory'), value_type=str),
            'loop': ParameterValue(LaunchConfiguration('loop'), value_type=bool),
        }],
        condition=IfCondition(LaunchConfiguration('markers')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(racing_config, 'gate_course.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        mode_arg,
        backend_arg,
        markers_arg,
        rviz_arg,
        trajectory_arg,
        loop_arg,
        crazyflie_launch,
        gate_markers,
        rviz,
    ])
