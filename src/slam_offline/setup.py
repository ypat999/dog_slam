from setuptools import setup

package_name = 'my_cartographer_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cartographer_3d.launch.py']),
        ('share/' + package_name + '/config', [
            'config/cartographer_3d.lua',
            'config/cartographer_3d_no_imu.lua',
            'config/cartographer_3d_optimized.lua',
            'config/cartographer_3d_with_imu.lua',
            'config/cartographer.rviz'
        ]),
    ],
    install_requires=['setuptools', 'pandas'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Cartographer 3D launch with CSV to PointCloud2 playback',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
