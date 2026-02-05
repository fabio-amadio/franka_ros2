from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'franka_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'std_msgs',
        'franka_msgs'
    ],
    zip_safe=True,
    maintainer='Fabio Amadio',
    maintainer_email='fabioamadio93@gmail.com',
    description='A collection of small ROS 2 tools and helper nodes for Franka robots (pose publishing, interactive markers, and utilities).',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_marker_pose_publisher = franka_tools.interactive_marker_pose_publisher:main',
            'interactive_marker_pose_publisher_gripper = franka_tools.interactive_marker_pose_publisher_gripper:main',
            'lin_traj_pos_pub = franka_tools.lin_traj_pos_pub:main',
            'collision_behavior_setter = franka_tools.collision_behavior_setter:main',
        ],
    },
)
