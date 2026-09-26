import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'crazyflie_rl'


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'backend', default_value='cflib', description='cflib 또는 cpp (sim 은 레거시 명령 미지원)')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='게이트·실궤적 rviz 같이 실행')

    config = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
    # 모캡 연결 설정(QTM IP·rigid body)은 crazyflie_test 것을 그대로 쓴다
    motion_capture_yaml = os.path.join(
        get_package_share_directory('crazyflie_test'), 'config', 'motion_capture.yaml')

    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('crazyflie'), 'launch', 'launch.py')),
        launch_arguments={
            'crazyflies_yaml_file': os.path.join(config, 'crazyflies_rl.yaml'),
            'motion_capture_yaml_file': motion_capture_yaml,
            'mocap': 'True',
            'backend': LaunchConfiguration('backend'),
        }.items(),
    )

    # 게이트 + 실제 비행 자취. 계획 궤적(TOGT)은 정책 경로와 다르므로 끈다
    markers = Node(
        package='crazyflie_racing', executable='gate_markers', name='gate_markers',
        output='screen', parameters=[{'show_path': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('crazyflie_racing'),
                                      'config', 'gate_course.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([backend_arg, rviz_arg, server, markers, rviz])
