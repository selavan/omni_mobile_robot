from setuptools import setup

package_name = 'rrt_star_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='RRT* Path Planner for TurtleBot3 Simulation',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_star_planner = rrt_star_planner.rrt_star_planner:main',
            'rrt_star_local_planner = rrt_star_planner.rrt_star_local_planner:main',
            'mod_rrt_star_local_planner = rrt_star_planner.mod_rrt_star_local_planner:main',
            'A_star_V2 = rrt_star_planner.A_star_V2:main',
            'A_star_V3 = rrt_star_planner.A_star_V3:main',
        ],
    },
)

