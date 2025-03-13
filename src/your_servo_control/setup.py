# setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'your_servo_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Mark the package in the ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include our package.xml file
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Description of your package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'servo_controller = your_servo_control.servo_controller:main',
        ],
    },
)