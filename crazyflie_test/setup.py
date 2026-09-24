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
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'patches'), glob('patches/*')),
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
        ],
    },
)
