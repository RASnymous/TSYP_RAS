from setuptools import setup
import os
from glob import glob

package_name = 'writer_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models', 'lora_beacon'),
         glob('models/lora_beacon/*')),
        (os.path.join('share', package_name, 'models', 'treated_marker'),
         glob('models/treated_marker/*')),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TSYP14 Team',
    maintainer_email='team@example.com',
    description='Writer Robot - TSYP14 Living Map (autonomous explorer, '
                'vision event detection, servo beacon deposition)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'event_detection_node = writer_robot.event_detection_node:main',
            'vision_event_detector = writer_robot.vision_event_detector:main',
            'beacon_drop_node = writer_robot.beacon_drop_node:main',
            'frame_translator_node = writer_robot.frame_translator_node:main',
            'frontier_explorer = writer_robot.frontier_explorer:main',
            'mission_recorder = writer_robot.mission_recorder:main',
            'beacon_radio_sim = writer_robot.beacon_radio_sim:main',
            'executor_node = writer_robot.executor_node:main',
            'mission_world = writer_robot.mission_world:main',
            'ona_link = writer_robot.ona_link_node:main',
            'mine_sensors = writer_robot.mine_sensors_node:main',
        ],
    },
)
