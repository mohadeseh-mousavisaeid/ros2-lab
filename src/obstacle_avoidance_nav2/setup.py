from setuptools import setup, find_packages

package_name = 'obstacle_avoidance_nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='projectlab3_ss25',
    maintainer_email='projectlab3_ss25@todo.todo',
    description='Standalone Nav2 obstacle avoidance node',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'nav2_node = obstacle_avoidance_nav2.nav2_node:main',
        ],
    },
)
