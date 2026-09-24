import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'crazyflie_racing'

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
    description='Crazyswarm2 기반 Crazyflie 게이트 레이싱 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gate_flight = crazyflie_racing.gate_flight:main',
            'gate_markers = crazyflie_racing.gate_markers:main',
            'plan_gate_trajectory = crazyflie_racing.plan_gate_trajectory:main',
        ],
    },
)
