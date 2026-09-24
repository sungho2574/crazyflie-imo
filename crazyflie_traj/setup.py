import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'crazyflie_traj'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Crazyswarm2 기반 Crazyflie 주기 궤적 비행·데이터 수집 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Blackbird 스타일 주기 궤적 — 도형별 연속-랩 비행
            'circle = crazyflie_traj.entry:circle',
            'oval = crazyflie_traj.entry:oval',
            'figure8 = crazyflie_traj.entry:figure8',
            'clover = crazyflie_traj.entry:clover',
            'star = crazyflie_traj.entry:star',
            'collect_traj_data = crazyflie_traj.collect_data:main',
            'traj_gen = crazyflie_traj.generator:main',
            'traj_markers = crazyflie_traj.markers:main',
        ],
    },
)
