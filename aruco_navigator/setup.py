from setuptools import setup
from glob import glob
import os

package_name = 'aruco_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resourc_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/aruco_navigation_launch.py']),
        ('share/' + package_name + '/launch', ['launch/aruco_marker_navigation_launch.py']),
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
            'aruco_marker_detector = aruco_navigator.aruco_marker_detector:main'    ,
            'marker_navigator = aruco_navigator.marker_navigator:main',
            'aruco_marker_navigation = aruco_navigator.aruco_marker_naviagtion:main',
        ],
    },
)