from setuptools import find_packages, setup

package_name = 'obstacle_avoidance_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='projectlab3_ss25',
    maintainer_email='projectlab3_ss25@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'obstacle_avoidance = obstacle_avoidance_controller.controller_node:main',
          'obstacle_avoidance_2 = obstacle_avoidance_controller.nav2_node:main', 
        ],
    },
)
