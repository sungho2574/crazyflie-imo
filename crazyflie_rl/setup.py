import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'crazyflie_rl'
# models/<이름>/ 마다 share 에 설치 (rl_flight --model-dir 로 고른다)
models = [(os.path.join('share', package_name, d), glob(os.path.join(d, '*')))
          for d in sorted(glob(os.path.join('models', '*'))) if os.path.isdir(d)]

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
    ] + models,
    install_requires=['setuptools'],
    zip_safe=True,
    description='Crazyswarm2 기반 Crazyflie 강화학습 레이싱 정책 실행 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_flight = crazyflie_rl.rl_flight:main',
            'rl_sim_check = crazyflie_rl.sim_check:main',
        ],
    },
)
