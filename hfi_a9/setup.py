from setuptools import setup

package_name = 'hfi_a9'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A simple ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hfi_a9_node = hfi_a9.hfi_a9_node:main',
            'hfi_a9_EKF_node = hfi_a9.hfi_a9_EKF_node:main',
        ],
    },
)

