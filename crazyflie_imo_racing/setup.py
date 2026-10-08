from glob import glob
from pathlib import Path
from setuptools import setup

setup(
    name='crazyflie_imo_racing', version='0.1.0',
    packages=['crazyflie_imo_racing'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/crazyflie_imo_racing']),
        ('share/crazyflie_imo_racing', ['package.xml', 'README.md', 'VALIDATION.md', 'LICENSE']),
        *[(f'share/crazyflie_imo_racing/{d}', [p for p in glob(f'{d}/*') if Path(p).is_file()])
          for d in ('launch', 'config', 'models')],
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    entry_points={'console_scripts': [
        'imo_flight = crazyflie_imo_racing.flight_node:main',
        'imo_visualization = crazyflie_imo_racing.visualization_node:main',
        'imo_sim = crazyflie_imo_racing.sim_node:main',
        'imo_replay = crazyflie_imo_racing.replay:main',
        'imo_evaluate = crazyflie_imo_racing.evaluate:main',
    ]},
)
