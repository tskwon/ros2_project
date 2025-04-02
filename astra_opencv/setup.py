from setuptools import find_packages, setup

package_name = 'astra_opencv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'astra_opencv = astra_opencv.astra_opencv:main',
            '3d_position = astra_opencv.3d_position:main'
        ],
    },
)
