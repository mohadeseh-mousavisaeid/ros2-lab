from setuptools import setup

package_name = 'simple_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A simple ROS2 node that plans a path using Nav2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run simple_path_planner simple_path_planner
            'simple_path_planner = simple_path_planner.simple_path_planner:main',
        ],
    },
)
