from setuptools import setup

package_name = 'path_tracking'

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
    maintainer='miccy',
    maintainer_email='miccy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = path_tracking.pure_pursuit:main',
            'mpc = path_tracking.mpc:main',
            'Pos_estimate = path_tracking.Pos_estimate:main',
            'path_reader = path_tracking.path_reader:main',
        ],
    },
)
