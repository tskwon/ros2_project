from setuptools import setup
import os
from glob import glob

package_name = 'robot_arm_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot arm vision test package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_arm_vision_test = robot_arm_test.robot_arm_vision_test:main',
            'test_commander = robot_arm_test.test_commander:main',
            'simple_test = robot_arm_test.simple_test:main',
        ],
    },
)