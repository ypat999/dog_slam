from setuptools import setup

package_name = 'livox_slam_online'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'livox_cartographer_launch.py',
            'launch/full_slam_system.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/mid360_config.json',
            'config/livox_mid360_cartographer.lua'
        ]),
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
            'data_recorder = livox_slam_online.scripts.data_recorder:main',
            'slam_manager = livox_slam_online.scripts.slam_manager:main',
            'test_system = livox_slam_online.scripts.test_system:main',
        ],
    },
)