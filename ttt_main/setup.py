from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ttt_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson-nx@example.com',
    description='Tic-Tac-Toe Main Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'game_logic_node = ttt_main.game_logic_node:main',
        'test_game_logic = ttt_main.test_game_logic:main',
        'main_orchestrator = ttt_main.main_orchestrator:main',
        
        ],
    },
)
