from setuptools import setup

package_name = 'ds4_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='youremail@example.com',
    maintainer='Your Name',
    maintainer_email='youremail@example.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
    ],
    description='DS4 Teleoperation Package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
	    'console_scripts': [
		'controller_node = ds4_teleop.controller_node:main',
		'imu_controller_node = ds4_teleop.imu_controller_node:main',
		'combined_controller_node = ds4_teleop.combined_controller_node:main',
		'controller_node_without_imu = ds4_teleop.controller_node_without_imu:main',
	    ],
	},


)

