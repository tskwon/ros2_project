from setuptools import find_packages, setup

package_name = 'realsense_data_collector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/data_collector_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xotn',
    maintainer_email='james190414@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_display_node = realsense_data_collector.rgb_display_node:main',
        ],
    },
)
