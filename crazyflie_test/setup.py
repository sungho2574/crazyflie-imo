import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'crazyflie_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'patches'), glob('patches/*')),
        # TOGT 오프라인 도구(소스·크플 파라미터). colcon 빌드에는 넣지 않고 참고용으로만 설치.
        (os.path.join('share', package_name, 'togt_tools'),
            glob('togt_tools/*.cpp') + glob('togt_tools/*.txt')),
        (os.path.join('share', package_name, 'togt_tools', 'params'),
            glob('togt_tools/params/*.yaml')),
        (os.path.join('share', package_name, 'togt_tools', 'params', 'init'),
            glob('togt_tools/params/init/*.yaml')),
        (os.path.join('share', package_name, 'togt_tools', 'params', 'refine'),
            glob('togt_tools/params/refine/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Crazyswarm2 기반 Crazyflie 비행 테스트 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = crazyflie_test.hello_world:main',
            'goto_square = crazyflie_test.goto_square:main',
            'multi_square = crazyflie_test.multi_square:main',
            'gate_flight = crazyflie_test.gate_flight:main',
            'gate_markers = crazyflie_test.gate_markers:main',
            'plan_gate_trajectory = crazyflie_test.plan_gate_trajectory:main',
            # Blackbird 스타일 주기 궤적 — 도형별 연속-랩 비행
            'circle = crazyflie_test.traj.entry:circle',
            'oval = crazyflie_test.traj.entry:oval',
            'figure8 = crazyflie_test.traj.entry:figure8',
            'clover = crazyflie_test.traj.entry:clover',
            'star = crazyflie_test.traj.entry:star',
            'collect_traj_data = crazyflie_test.collect_data:main',
            'traj_gen = crazyflie_test.traj.generator:main',
            'traj_markers = crazyflie_test.traj.markers:main',
        ],
    },
)
