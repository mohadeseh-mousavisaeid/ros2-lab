from glob import glob
from setuptools import setup

package_name = 'imitation_control_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/models', glob('models/*.pt')),  # ✅ This line installs the .pt file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imitation_control_package = imitation_control_package.imitation_control_package:main',
        ],
    },
)
