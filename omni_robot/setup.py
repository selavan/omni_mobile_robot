from setuptools import setup
import os
import glob  # Import glob

package_name = 'omni_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/omni_robot_launch.py']),  # Launch directory
        (os.path.join('share', package_name), glob.glob('launch/*.py')),  # Include all launch files
        (os.path.join('share', package_name, 'srv'), glob.glob('srv/*.srv')),  # Include srv files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    maintainer='Your Name',
    maintainer_email='you@example.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A simple ROS2 package for an omni-directional robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'body_velocity_node = omni_robot.body_velocity_node:main',
            'global_velocity_node = omni_robot.global_velocity_node:main',
            'odom_node = omni_robot.odometry:main',
            'twist_publisher = omni_robot.twist_publisher:main',
            'key_twist_publisher = omni_robot.key_twist_publisher:main',
            # Add other nodes here
        ],
    },
)

