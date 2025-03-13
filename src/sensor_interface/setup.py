from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junyan',
    maintainer_email='chua.junyan0612@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar = sensor_interface.lidar:main',
            'ultrasonic_dual = sensor_interface.ultrasonic_dual_node:main',
        ],
    },
)
