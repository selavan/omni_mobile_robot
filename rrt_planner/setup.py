from setuptools import setup

package_name = 'rrt_planner'

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
            'rrt_planner = rrt_planner.rrt_planner:main',
            'rrt_planner_v1 = rrt_planner.rrt_planner_v1:main',
            'rrt_planner_v2 = rrt_planner.rrt_planner_v2:main',
            'waypoints = rrt_planner.waypoints:main',
            'pure_pursuit = rrt_planner.pure_pursuit:main',
            'lqr = rrt_planner.lqr:main',
            'map_ideal = rrt_planner.map_ideal:main',
        ],
    },
)
