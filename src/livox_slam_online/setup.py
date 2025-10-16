from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'livox_slam_online'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['livox_slam_online', 'livox_slam_online.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Livox SLAM system integrating Mid-360 LiDAR with Cartographer',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_recorder = scripts.data_recorder:main',
            'slam_manager = scripts.slam_manager:main',
            'simple_imu_filter = scripts.simple_imu_filter:main',
        ],
    },
)
