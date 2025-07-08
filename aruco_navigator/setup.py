from setuptools import setup
from glob import glob
import os

package_name = 'aruco_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/aruco_navigation_launch.py']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ArUco 마커 감지 및 내비게이션',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_navigator.aruco_detector:main', # 새 스크립트 추가
            'marker_navigator = aruco_navigator.marker_navigator:main',
            'robot_display = aruco_navigator.robot_display:main',
            'camera_calibrator = aruco_navigator.camera_calibrator_node:main',
            'stable_aruco_detection_node = aruco_navigator.stable_aruco_marker:main',
        ],
    },
)