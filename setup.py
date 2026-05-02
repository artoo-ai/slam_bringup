from setuptools import setup
from glob import glob
import os

package_name = 'slam_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='rico',
    maintainer_email='rico@makeorcode.com',
    description='Portable sensor-rig SLAM stack (Mid-360 + D435 + WitMotion) for multiple mobile platforms',
    license='MIT',
    entry_points={
        'console_scripts': [
            # WitMotion WT901C / WT9011 0x61 packet → sensor_msgs/Imu publisher.
            # See slam_bringup/wt901c_imu_node.py for the protocol rationale.
            'wt901c_imu = slam_bringup.wt901c_imu_node:main',
            # Visualization-only z-clip republisher. See slam_bringup/viz_clip_node.py
            # for why we do this in pure Python instead of pcl_ros::PassThrough.
            'viz_clip = slam_bringup.viz_clip_node:main',
            # /livox/imu (units of g) → /livox/imu_ms2 (m/s²) republisher.
            # See slam_bringup/imu_units_node.py for the rationale.
            'imu_units_g_to_ms2 = slam_bringup.imu_units_node:main',
        ],
    },
)
